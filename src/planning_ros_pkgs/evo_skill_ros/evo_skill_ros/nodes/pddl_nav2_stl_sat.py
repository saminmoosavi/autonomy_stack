#!/usr/bin/env python3
"""
PDDL -> Nav2 goals -> Nav2 path -> STL satisfiability monitor.

Example:
ros2 run evo_skill_ros pddl_nav2_stl_sat --ros-args \
  -p ns:=/a200_0000 \
  -p pose_topic:=/a200_0000/platform/odom \
  -p pose_msg_type:=odometry \
  -p target_region:=R10
"""
import json
import math
import re
import shlex
import subprocess
import tempfile
import time
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from vision_msgs.msg import Detection3D

from evo_skill_ros.pddl_stl.pipeline import (
    GroundAction,
    ProblemState,
    parse_domain,
    validate_plan,
)


@dataclass
class StlMonitorResult:
    satisfied: bool
    min_obstacle_margin: float = float("inf")
    final_goal_margin: float = -float("inf")
    goal_checked: bool = False
    checked_trajectory_points: int = 0
    closest_obstacle: str | None = None
    closest_obstacle_xy: tuple[float, float] | None = None
    closest_obstacle_type: str | None = None
    violations: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class ObstacleConstraint:
    name: str
    xy: tuple[float, float]
    obstacle_type: str
    clearance: float


@dataclass
class TrackedObject:
    label: str
    xy: tuple[float, float]
    stamp: Time


class PpddlNav2StlSat(Node):
    def __init__(self):
        super().__init__("ppddl_nav2_stl_sat")

        self.ns = self.declare_parameter("ns", "/a200_0000").value
        self.pose_topic = self.declare_parameter("pose_topic", f"{self.ns}/platform/odom").value
        self.pose_msg_type = self.declare_parameter("pose_msg_type", "odometry").value
        self.map_topic = self.declare_parameter("map_topic", f"{self.ns}/map").value
        self.nav2_plan_topic = self.declare_parameter("nav2_plan_topic", f"{self.ns}/plan").value
        self.tracks_topic = self.declare_parameter("tracks_topic", f"{self.ns}/tracks").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.robot_name = self.declare_parameter("robot_name", "jackal1").value.lower()
        self.target_region = self.declare_parameter("target_region", "R10").value.lower()
        self.graph_file = self.resolve_path(self.declare_parameter("graph_file", "config/graph.json").value)
        self.domain_file = self.resolve_path(self.declare_parameter("domain_file", "config/factory_sim_domain.pddl").value)
        self.domain_name = self.read_pddl_domain_name(self.domain_file)

        self.fast_downward_cmd = self.declare_parameter("fast_downward_cmd", "fast-downward.py").value
        self.fast_downward_search = self.declare_parameter("fast_downward_search", "astar(lmcut())").value
        self.fast_downward_timeout_s = float(
            self.declare_parameter("fast_downward_timeout_s", 30.0).value
        )
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 3.0).value)
        self.map_timeout_s = float(self.declare_parameter("map_timeout_s", 5.0).value)
        self.require_map = bool(self.declare_parameter("require_map", False).value)
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.75).value)
        self.eventual_goal_check_distance = float(
            self.declare_parameter("eventual_goal_check_distance", 2.5).value
        )
        self.obstacle_clearance = float(self.declare_parameter("obstacle_clearance", 0.45).value)
        self.frisbe_clearance = float(self.declare_parameter("frisbe_clearance", 0.00).value)

        self.human_clearance = float(self.declare_parameter("human_clearance", 1.0).value)
        self.chair_clearance = float(self.declare_parameter("chair_clearance", 0.1).value)
        self.shelf_clearance = float(self.declare_parameter("shelf_clearance", 0.1).value)
        self.column_clearance = float(self.declare_parameter("column_clearance", 0.10).value)
        self.table_clearance = float(self.declare_parameter("table_clearance", 0.10).value)
        self.occupied_threshold = int(self.declare_parameter("occupied_threshold", 65).value)
        self.map_obstacle_stride = max(1, int(self.declare_parameter("map_obstacle_stride", 4).value))
        self.map_obstacle_corridor_width = float(
            self.declare_parameter("map_obstacle_corridor_width", 2.0).value
        )
        self.max_map_obstacles = max(1, int(self.declare_parameter("max_map_obstacles", 2500).value))
        self.tracked_object_timeout_s = float(
            self.declare_parameter("tracked_object_timeout_s", 2.0).value
        )
        self.waypoints_topic = self.declare_parameter("waypoints_topic", "ppddl_nav2_goals").value
        self.waypoints_publish_period_s = float(
            self.declare_parameter("waypoints_publish_period_s", 1.0).value
        )
        self.enable_json_log = bool(self.declare_parameter("enable_json_log", True).value)
        self.json_log_file = Path(
            self.declare_parameter(
                "json_log_file",
                str(Path.home() / "autonomy_stack_ros_humble" / "pddl_nav2_stl_sat_log.json"),
            ).value
        )
        self.enable_stl_replan = bool(self.declare_parameter("enable_stl_replan", True).value)
        self.max_nav2_replans = max(0, int(self.declare_parameter("max_nav2_replans", 3).value))
        self.stl_replan_cooldown_s = float(
            self.declare_parameter("stl_replan_cooldown_s", 2.0).value
        )
        self.enable_stl_costmap_edit = bool(
            self.declare_parameter("enable_stl_costmap_edit", True).value
        )
        self.costmap_edit_topic = self.declare_parameter("costmap_edit_topic", self.map_topic).value
        self.costmap_edit_min_radius = float(
            self.declare_parameter("costmap_edit_min_radius", 1.0).value
        )
        self.costmap_edit_padding = float(
            self.declare_parameter("costmap_edit_padding", 0.75).value
        )
        self.costmap_edit_occupied_value = max(
            0,
            min(100, int(self.declare_parameter("costmap_edit_occupied_value", 100).value)),
        )
        self.costmap_edit_publish_repeats = max(
            1,
            int(self.declare_parameter("costmap_edit_publish_repeats", 3).value),
        )
        self.costmap_edit_replan_delay_s = max(
            0.0,
            float(self.declare_parameter("costmap_edit_replan_delay_s", 0.2).value),
        )

        self.world, self.regions, self.objects = self.load_graph(self.graph_file)
        if self.target_region not in self.regions:
            raise ValueError(f"target_region '{self.target_region}' is not in {self.graph_file}")

        self.current_xy = None
        self.current_region = None
        self.map_msg = None
        self.tracked_objects = {}
        self.last_logged_map_shape = None
        self.last_waypoints_path = None
        self.last_nav2_path = None
        self.active_plan_report = None
        self.active_plan_actions = None
        self.active_waypoints = None
        self.last_monitor_signature = None
        self.sent_goal = False
        self.active_goal_handle = None
        self.replan_in_progress = False
        self.nav2_replan_count = 0
        self.last_stl_replan_time = None
        self.json_log_events = []
        self.json_log_failed = False
        self.published_costmap_edit_stamps = set()
        self.last_tracked_obstacle_log_time = None
        self.last_monitor_obstacle_log_time = None
        self.started_at = self.get_clock().now()

        self.client = ActionClient(self, FollowWaypoints, f"{self.ns}/follow_waypoints")
        path_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.waypoints_pub = self.create_publisher(NavPath, self.waypoints_topic, path_qos)
        self.costmap_edit_pub = self.create_publisher(
            OccupancyGrid,
            self.costmap_edit_topic,
            path_qos,
        )
        self.create_pose_subscription()
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, path_qos)
        self.create_subscription(NavPath, self.nav2_plan_topic, self.nav2_plan_callback, 10)
        self.create_subscription(Detection3D, self.tracks_topic, self.track_callback, 10)
        self.timer = self.create_timer(0.5, self.plan_once_when_ready)
        self.waypoints_timer = self.create_timer(
            self.waypoints_publish_period_s,
            self.republish_waypoints_path,
        )

        self.get_logger().info(
            f"Waiting for {self.pose_msg_type} pose on {self.pose_topic}; "
            f"target={self.target_region}; monitor_map={self.map_topic}; "
            f"costmap_edit_topic={self.costmap_edit_topic}; "
            f"tracks={self.tracks_topic}; nav2_goals_topic={self.waypoints_topic}; "
            f"nav2_plan_topic={self.nav2_plan_topic}"
        )
        self.record_json_event(
            "node_started",
            {
                "target_region": self.target_region,
                "pose_topic": self.pose_topic,
                "map_topic": self.map_topic,
                "costmap_edit_topic": self.costmap_edit_topic,
                "tracks_topic": self.tracks_topic,
                "nav2_plan_topic": self.nav2_plan_topic,
                "json_log_file": str(self.json_log_file),
            },
        )

    def resolve_path(self, value):
        path = Path(value)
        if path.is_absolute():
            if path.exists():
                return path
            package_path = self.resolve_package_file(path.name)
            if package_path is not None:
                return package_path
        if path.exists():
            return path
        package_path = self.resolve_package_file(str(path))
        if package_path is not None:
            return package_path
        package_path = self.resolve_package_file(path.name)
        if package_path is not None:
            return package_path
        return Path.cwd() / value

    def resolve_package_file(self, value):
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = Path(get_package_share_directory("evo_skill_ros"))
            candidates = [share_dir / value, share_dir / "config" / Path(value).name]
            for candidate in candidates:
                if candidate.exists():
                    return candidate
        except Exception:
            return None
        return None

    def load_graph(self, path):
        with open(path, "r") as stream:
            world = json.load(stream)
        regions = {region["name"].lower(): tuple(region["coords"]) for region in world.get("regions", [])}
        objects = {obj["name"].lower(): tuple(obj["coords"]) for obj in world.get("objects", [])}
        return world, regions, objects

    def read_pddl_domain_name(self, path):
        try:
            text = Path(path).read_text()
        except OSError:
            return "unknown"
        match = re.search(r"\(\s*domain\s+([^\s()]+)", text, re.IGNORECASE)
        return match.group(1).lower() if match else "unknown"

    def format_fact(self, fact):
        return "(" + " ".join(fact) + ")"

    def format_plan(self, plan):
        if not plan:
            return "<empty plan>"
        return " -> ".join(action.text() for action in plan)

    def record_json_event(self, event_type, data=None):
        if not self.enable_json_log:
            return
        event = {
            "stamp": datetime.now(timezone.utc).isoformat(),
            "ros_time_sec": self.get_clock().now().nanoseconds / 1e9,
            "event": event_type,
            "data": data or {},
        }
        self.json_log_events.append(event)
        payload = {
            "node": self.get_name(),
            "target_region": self.target_region,
            "events": self.json_log_events,
        }
        try:
            self.json_log_file.parent.mkdir(parents=True, exist_ok=True)
            self.json_log_file.write_text(json.dumps(payload, indent=2))
        except OSError as exc:
            if not self.json_log_failed:
                self.json_log_failed = True
                self.get_logger().warn(f"Could not write JSON log {self.json_log_file}: {exc}")

    def format_classical_problem(self, problem):
        object_names = " ".join(sorted(problem.objects))
        init_lines = [f"    {self.format_fact(fact)}" for fact in sorted(problem.facts)]
        goal_lines = [f"      {self.format_fact(fact)}" for fact in sorted(problem.goals)]
        return "\n".join([
            "(define (problem generated_factory_nav)",
            f"  (:domain {self.domain_name}_classical)",
            f"  (:objects {object_names})",
            "  (:init",
            *init_lines,
            "  )",
            "  (:goal (and",
            *goal_lines,
            "  ))",
            ")",
            "",
        ])

    def format_classical_domain(self, domain, problem):
        predicate_arities = {}
        for schema in domain.actions.values():
            for fact in (
                schema.positive_preconditions
                + schema.negative_preconditions
                + schema.add_effects
                + schema.del_effects
            ):
                predicate_arities[fact[0]] = max(predicate_arities.get(fact[0], 0), len(fact) - 1)
        for fact in problem.facts | problem.goals:
            predicate_arities[fact[0]] = max(predicate_arities.get(fact[0], 0), len(fact) - 1)

        lines = [
            f"(define (domain {self.domain_name}_classical)",
            "  (:requirements :strips :negative-preconditions)",
            "  (:predicates",
        ]
        for name, arity in sorted(predicate_arities.items()):
            args = " ".join(f"?x{idx}" for idx in range(arity))
            lines.append(f"    ({name}{(' ' + args) if args else ''})")
        lines.append("  )")

        for schema in domain.actions.values():
            params = " ".join(schema.parameters)
            preconditions = [
                f"      {self.format_fact(fact)}"
                for fact in schema.positive_preconditions
            ]
            preconditions.extend(
                f"      (not {self.format_fact(fact)})"
                for fact in schema.negative_preconditions
            )
            add_effects = set(schema.add_effects)
            effective_del_effects = [
                fact for fact in schema.del_effects
                if fact not in add_effects
            ]
            effects = [
                f"      (not {self.format_fact(fact)})"
                for fact in effective_del_effects
            ]
            effects.extend(
                f"      {self.format_fact(fact)}"
                for fact in schema.add_effects
            )
            lines.extend([
                f"  (:action {schema.name}",
                f"    :parameters ({params})",
                "    :precondition (and",
                *(preconditions or ["      "]),
                "    )",
                "    :effect (and",
                *(effects or ["      "]),
                "    )",
                "  )",
            ])
        lines.append(")")
        lines.append("")
        return "\n".join(lines)

    def parse_fast_downward_plan(self, path):
        actions = []
        for line in Path(path).read_text().splitlines():
            line = line.strip().lower()
            if not line or line.startswith(";"):
                continue
            match = re.search(r"\(([^)]+)\)", line)
            if not match:
                continue
            parts = match.group(1).split()
            actions.append(GroundAction(parts[0], tuple(parts[1:])))
        return actions

    def run_fast_downward(self, domain_text, problem_text):
        with tempfile.TemporaryDirectory(prefix="pddl_nav2_fd_") as tmpdir:
            tmpdir_path = Path(tmpdir)
            domain_path = tmpdir_path / "domain.pddl"
            problem_path = tmpdir_path / "problem.pddl"
            plan_path = tmpdir_path / "sas_plan"
            domain_path.write_text(domain_text)
            problem_path.write_text(problem_text)

            command = (
                shlex.split(str(self.fast_downward_cmd))
                + [
                    "--plan-file",
                    str(plan_path),
                    str(domain_path),
                    str(problem_path),
                    "--search",
                    str(self.fast_downward_search),
                ]
            )
            self.get_logger().info("Running Fast Downward: " + " ".join(shlex.quote(part) for part in command))
            self.record_json_event(
                "fast_downward_started",
                {
                    "command": command,
                    "domain_file": str(domain_path),
                    "problem_file": str(problem_path),
                    "plan_file": str(plan_path),
                },
            )
            try:
                completed = subprocess.run(
                    command,
                    cwd=tmpdir,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=self.fast_downward_timeout_s,
                )
            except FileNotFoundError as exc:
                raise RuntimeError(
                    f"Fast Downward command not found: {self.fast_downward_cmd}. "
                    "Set ROS parameter fast_downward_cmd to the fast-downward.py path."
                ) from exc
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError(
                    f"Fast Downward timed out after {self.fast_downward_timeout_s:.1f}s"
                ) from exc

            stdout_tail = "\n".join(completed.stdout.splitlines()[-20:])
            stderr_tail = "\n".join(completed.stderr.splitlines()[-20:])
            if completed.returncode != 0:
                self.record_json_event(
                    "fast_downward_failed",
                    {
                        "returncode": completed.returncode,
                        "stdout_tail": stdout_tail,
                        "stderr_tail": stderr_tail,
                    },
                )
                raise RuntimeError(
                    "Fast Downward failed with exit code "
                    f"{completed.returncode}\nstdout:\n{stdout_tail}\nstderr:\n{stderr_tail}"
                )
            if not plan_path.exists():
                self.record_json_event(
                    "fast_downward_no_plan",
                    {
                        "stdout_tail": stdout_tail,
                        "stderr_tail": stderr_tail,
                    },
                )
                raise RuntimeError(
                    "Fast Downward completed without producing a plan\n"
                    f"stdout:\n{stdout_tail}\nstderr:\n{stderr_tail}"
                )
            plan = self.parse_fast_downward_plan(plan_path)
            self.record_json_event(
                "fast_downward_finished",
                {
                    "returncode": completed.returncode,
                    "plan": [action.text() for action in plan],
                },
            )
            return plan

    def create_pose_subscription(self):
        if self.pose_msg_type.lower() in {"pose", "amcl", "pose_with_covariance"}:
            self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10)
        else:
            self.create_subscription(Odometry, self.pose_topic, self.odom_callback, 10)

    def odom_callback(self, msg):
        self.update_current_pose(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def pose_callback(self, msg):
        self.update_current_pose(msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update_current_pose(self, x, y):
        self.current_xy = (float(x), float(y))
        self.current_region, distance = self.nearest_region(self.current_xy)
        self.get_logger().debug(
            f"Current pose ({x:.2f}, {y:.2f}) snapped to {self.current_region} at distance {distance:.2f}"
        )

    def map_callback(self, msg):
        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp_key in self.published_costmap_edit_stamps:
            return
        self.map_msg = msg
        map_shape = (msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id)
        if map_shape != self.last_logged_map_shape:
            self.last_logged_map_shape = map_shape
            self.get_logger().info(
                f"Received ROS map {msg.info.width}x{msg.info.height}, "
                f"resolution={msg.info.resolution:.3f}, frame={msg.header.frame_id or self.frame_id}"
            )

    def nav2_plan_callback(self, msg):
        self.last_nav2_path = msg
        if not self.active_waypoints:
            return
        path_points = self.path_msg_to_xy(msg)
        if len(path_points) < 2:
            return
        signature = (
            len(path_points),
            round(path_points[0][0], 2),
            round(path_points[0][1], 2),
            round(path_points[-1][0], 2),
            round(path_points[-1][1], 2),
        )
        if signature == self.last_monitor_signature:
            return
        self.last_monitor_signature = signature
        monitor_result = self.monitor_nav2_trajectory(path_points)
        self.log_monitor_result(monitor_result)
        self.refine_pddl_plan_from_monitor(self.active_plan_report, monitor_result)

    def track_callback(self, msg):
        if not msg.results:
            return
        result = msg.results[0]
        label = self.normalize_track_label(result.hypothesis.class_id)
        track_id = self.normalize_track_label(msg.id)
        name = f"{label}_{track_id}" if track_id else label
        msg_stamp = Time.from_msg(msg.header.stamp)
        received_at = self.get_clock().now()

        position = result.pose.pose.position
        self.tracked_objects[name] = TrackedObject(
            label=label,
            xy=(float(position.x), float(position.y)),
            stamp=received_at,
        )
        msg_age_s = (
            (received_at - msg_stamp).nanoseconds / 1e9
            if msg_stamp.nanoseconds != 0
            else None
        )
        msg_age_text = f"{msg_age_s:.2f} s" if msg_age_s is not None else "<zero stamp>"
        self.get_logger().info(
            f"Track received: name={name}, label={label}, "
            f"frame={msg.header.frame_id or '<empty>'}, "
            f"xy=({position.x:.2f}, {position.y:.2f}), "
            f"type={self.object_type_from_name(label)}, "
            f"clearance={self.clearance_for_object(label):.2f} m, "
            f"msg_age={msg_age_text}"
        )
        self.get_logger().debug(
            f"Tracked object {name}: label={label}, xy=({position.x:.2f}, {position.y:.2f})"
        )

        if self.active_waypoints and self.last_nav2_path is not None:
            path_points = self.path_msg_to_xy(self.last_nav2_path)
            if len(path_points) >= 2:
                monitor_result = self.monitor_nav2_trajectory(path_points)
                self.log_monitor_result(monitor_result)
                self.refine_pddl_plan_from_monitor(self.active_plan_report, monitor_result)

    def nearest_region(self, xy):
        x, y = xy
        best_region = None
        best_distance = float("inf")
        for name, (rx, ry) in self.regions.items():
            distance = math.hypot(x - rx, y - ry)
            if distance < best_distance:
                best_region = name
                best_distance = distance
        return best_region, best_distance

    def plan_once_when_ready(self):
        if self.sent_goal:
            return
        if self.current_region is None:
            elapsed_s = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
            if elapsed_s > self.pose_timeout_s:
                fallback = self.world.get("robot_location", "").lower()
                if fallback in self.regions:
                    self.current_region = fallback
                    self.current_xy = self.regions[fallback]
                    self.get_logger().warn(
                        f"No pose received; falling back to graph robot_location={self.current_region}"
                    )
                else:
                    return
            else:
                return
        if self.require_map and self.map_msg is None:
            elapsed_s = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
            if elapsed_s > self.map_timeout_s:
                self.get_logger().warn(f"Waiting for occupancy grid on {self.map_topic}")
            return
        if not self.client.wait_for_server(timeout_sec=0.2):
            self.get_logger().info(f"Waiting for Nav2 action server {self.ns}/follow_waypoints")
            return

        self.get_logger().info(
            f"Starting PDDL -> Nav2 -> STL monitor flow from {self.current_region} to {self.target_region}"
        )
        try:
            plan_report = self.compute_pddl_plan()
            if not plan_report["ok"]:
                self.get_logger().error(f"No valid PDDL plan found: {plan_report.get('history', [])[-3:]}")
                self.sent_goal = True
                return

            waypoints = self.plan_to_nav2_goals(plan_report["plan_actions"])
            self.active_plan_report = plan_report
            self.active_plan_actions = plan_report["plan_actions"]
            self.active_waypoints = waypoints

            self.get_logger().info("PDDL plan: " + " -> ".join(plan_report["plan"]))
            self.get_logger().info(
                "Nav2 goals: " + ", ".join(f"({x:.2f}, {y:.2f})" for x, y in waypoints)
            )
            self.record_json_event(
                "initial_plan_selected",
                {
                    "pddl_plan": plan_report["plan"],
                    "nav2_goals": [{"x": x, "y": y} for x, y in waypoints],
                },
            )
            self.get_logger().info(
                f"Waiting for Nav2-created trajectory on {self.nav2_plan_topic} for STL monitoring"
            )
            self.publish_waypoints_path(waypoints)
            self.send_waypoints(waypoints)
            self.sent_goal = True
        except Exception as exc:
            self.get_logger().error(f"PPDDL Nav2 STL-SAT flow failed: {exc}")
            self.record_json_event("pipeline_failed", {"error": str(exc)})
            self.sent_goal = True

    def make_problem_from_graph(self):
        objects = {self.robot_name: "robot"}
        objects.update({name: "location" for name in self.regions})

        facts = {
            ("at", self.robot_name, self.current_region),
            ("localized", self.robot_name),
            ("battery-ok", self.robot_name),
            ("available", self.robot_name),
        }
        for region in self.regions:
            facts.add(("safe", region))
        for src, dst in self.world.get("region_connections", []):
            src = src.lower()
            dst = dst.lower()
            if src in self.regions and dst in self.regions:
                facts.add(("connected", src, dst))
                facts.add(("connected", dst, src))

        return ProblemState(
            objects=objects,
            facts=facts,
            goals={("visited", self.target_region)},
        )

    def compute_pddl_plan(self):
        self.get_logger().info(f"Loading PDDL domain from {self.domain_file}")
        domain = parse_domain(self.domain_file)
        problem = self.make_problem_from_graph()
        classical_domain = self.format_classical_domain(domain, problem)
        classical_problem = self.format_classical_problem(problem)
        candidate = self.run_fast_downward(classical_domain, classical_problem)
        result = validate_plan(domain, problem, candidate, require_goal=True)
        history = [{
            "planner": "fast_downward",
            "plan": [action.text() for action in candidate],
            "ok": result.ok,
            "errors": result.errors,
        }]

        if result.ok:
            self.get_logger().info(f"Selected Fast Downward PDDL plan: {self.format_plan(candidate)}")
            self.record_json_event(
                "pddl_plan_validated",
                {"pddl_plan": [action.text() for action in candidate]},
            )
            return {
                "ok": True,
                "plan": [action.text() for action in candidate],
                "plan_actions": candidate,
                "history": history,
            }

        self.get_logger().error(
            f"Fast Downward returned a plan that failed local validation: "
            f"{self.format_plan(candidate)} | errors={result.errors}"
        )
        self.record_json_event(
            "pddl_plan_validation_failed",
            {
                "pddl_plan": [action.text() for action in candidate],
                "errors": result.errors,
            },
        )
        return {"ok": False, "history": history}

    def plan_to_nav2_goals(self, plan):
        waypoints = []
        for action in plan:
            if not action.name.startswith("move") or len(action.args) < 3:
                continue
            goal_region = action.args[-1].lower()
            if goal_region not in self.regions:
                self.get_logger().warn(
                    f"Skipping move action with unknown goal region '{goal_region}': {action.text()}"
                )
                continue
            self.append_waypoint(waypoints, self.regions[goal_region])

        if not waypoints:
            raise RuntimeError("PDDL plan did not contain move actions that map to graph regions")
        return waypoints

    def monitor_nav2_trajectory(self, path_points):
        obstacles = self.build_monitor_obstacles(path_points)
        result = StlMonitorResult(satisfied=True)
        result.checked_trajectory_points = len(path_points)

        target_xy = self.regions[self.target_region]
        goal_check_xy = self.current_xy or path_points[-1]
        distance_to_target = math.hypot(
            goal_check_xy[0] - target_xy[0],
            goal_check_xy[1] - target_xy[1],
        )
        if distance_to_target <= self.eventual_goal_check_distance:
            result.goal_checked = True
            goal_margins = [
                self.goal_tolerance - math.hypot(x - target_xy[0], y - target_xy[1])
                for x, y in path_points
            ]
            result.final_goal_margin = max(goal_margins)
            if result.final_goal_margin < 0.0:
                result.satisfied = False
                result.violations.append(
                    f"eventually goal violated on Nav2 path: best goal margin={result.final_goal_margin:.3f} m"
                )

        for obstacle in obstacles:
            distance = self.distance_to_polyline(obstacle.xy, path_points)
            margin = distance - obstacle.clearance
            if margin < result.min_obstacle_margin:
                result.closest_obstacle = obstacle.name
                result.closest_obstacle_xy = obstacle.xy
                result.closest_obstacle_type = obstacle.obstacle_type
            result.min_obstacle_margin = min(result.min_obstacle_margin, margin)
            if margin < 0.0:
                result.satisfied = False
                result.violations.append(
                    f"always avoid {obstacle.obstacle_type} violated near {obstacle.name}: "
                    f"required={obstacle.clearance:.2f} m, margin={margin:.3f} m"
                )

        if not obstacles:
            result.min_obstacle_margin = float("inf")
        return result

    def build_monitor_obstacles(self, path_points):
        obstacles = [
            ObstacleConstraint(
                name=name,
                xy=xy,
                obstacle_type=self.object_type_from_name(name),
                clearance=self.clearance_for_object(name),
            )
            for name, xy in self.objects.items()
        ]
        obstacles.extend(self.build_tracked_obstacles())
        self.log_monitor_obstacles(obstacles)
        if self.map_msg is None:
            return obstacles

        msg = self.map_msg
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        for row in range(0, height, self.map_obstacle_stride):
            for col in range(0, width, self.map_obstacle_stride):
                idx = row * width + col
                if data[idx] < self.occupied_threshold:
                    continue
                xy = self.map_cell_center_to_world(msg, col, row)
                if self.distance_to_polyline(xy, path_points) > self.map_obstacle_corridor_width:
                    continue
                obstacles.append(
                    ObstacleConstraint(
                        name=f"map_occ_{col}_{row}",
                        xy=xy,
                        obstacle_type="map_obstacle",
                        clearance=self.obstacle_clearance,
                    )
                )
                if len(obstacles) >= self.max_map_obstacles:
                    break
            if len(obstacles) >= self.max_map_obstacles:
                break

        return obstacles

    def build_tracked_obstacles(self):
        now = self.get_clock().now()
        obstacles = []
        stale_names = []
        for name, tracked in self.tracked_objects.items():
            age_s = (now - tracked.stamp).nanoseconds / 1e9
            if age_s > self.tracked_object_timeout_s:
                stale_names.append(name)
                continue
            obstacles.append(
                ObstacleConstraint(
                    name=f"track_{name}",
                    xy=tracked.xy,
                    obstacle_type=self.object_type_from_name(tracked.label),
                    clearance=self.clearance_for_object(tracked.label),
                )
            )
        for name in stale_names:
            self.get_logger().info(f"Dropping stale tracked obstacle {name}")
            self.tracked_objects.pop(name, None)
        self.log_tracked_obstacles(obstacles)
        return obstacles

    def should_log_periodically(self, attr_name, period_s):
        now = self.get_clock().now()
        last_time = getattr(self, attr_name)
        if last_time is not None:
            age_s = (now - last_time).nanoseconds / 1e9
            if age_s < period_s:
                return False
        setattr(self, attr_name, now)
        return True

    def log_tracked_obstacles(self, obstacles):
        if not self.should_log_periodically("last_tracked_obstacle_log_time", 1.0):
            return
        if not obstacles:
            self.get_logger().info("Tracked STL obstacles: none")
            return
        summary = ", ".join(
            f"{obs.name}:{obs.obstacle_type}@({obs.xy[0]:.2f},{obs.xy[1]:.2f})/"
            f"clearance={obs.clearance:.2f}"
            for obs in obstacles
        )
        self.get_logger().info(f"Tracked STL obstacles: {summary}")

    def log_monitor_obstacles(self, obstacles):
        if not self.should_log_periodically("last_monitor_obstacle_log_time", 2.0):
            return
        tracked_count = sum(1 for obs in obstacles if obs.name.startswith("track_"))
        human_count = sum(1 for obs in obstacles if obs.obstacle_type == "human")
        self.get_logger().info(
            f"STL monitor obstacles before map expansion: total={len(obstacles)}, "
            f"tracked={tracked_count}, humans={human_count}"
        )

    def log_monitor_result(self, result):
        self.get_logger().info(
            f"STL monitor satisfied={result.satisfied}; "
            f"trajectory_points={result.checked_trajectory_points}; "
            f"min_obstacle_margin={result.min_obstacle_margin:.3f}; "
            f"eventually_goal_checked={result.goal_checked}; "
            f"eventually_goal_margin={result.final_goal_margin:.3f}; "
            f"closest_obstacle={result.closest_obstacle}"
        )
        for violation in result.violations:
            self.get_logger().warn(f"STL monitor violation: {violation}")
        self.record_json_event(
            "stl_monitor_result",
            {
                "satisfied": result.satisfied,
                "trajectory_points": result.checked_trajectory_points,
                "min_obstacle_margin": result.min_obstacle_margin,
                "goal_checked": result.goal_checked,
                "final_goal_margin": result.final_goal_margin,
                "closest_obstacle": result.closest_obstacle,
                "closest_obstacle_xy": (
                    {"x": result.closest_obstacle_xy[0], "y": result.closest_obstacle_xy[1]}
                    if result.closest_obstacle_xy is not None
                    else None
                ),
                "closest_obstacle_type": result.closest_obstacle_type,
                "violations": result.violations,
            },
        )

    def object_type_from_name(self, name):
        normalized = name.lower()
        if normalized.startswith(("person", "human")):
            return "human"
        if normalized.startswith("chair"):
            return "chair"
        if "shelf" in normalized:
            return "shelf"
        if normalized.startswith("column"):
            return "column"
        if normalized.startswith("table"):
            return "table"
        if normalized.startswith("frisbe"):
            return "frisbe"
        return "obstacle"

    def normalize_track_label(self, value):
        normalized = str(value).strip().lower()
        normalized = re.sub(r"[^a-z0-9_]+", "_", normalized)
        return normalized.strip("_")

    def clearance_for_object(self, name):
        object_type = self.object_type_from_name(name)
        if object_type == "human":
            return self.human_clearance
        if object_type == "chair":
            return self.chair_clearance
        if object_type == "shelf":
            return self.shelf_clearance
        if object_type == "column":
            return self.column_clearance
        if object_type == "table":
            return self.table_clearance
        if object_type == "frisbe":
            return self.frisbe_clearance
        return self.obstacle_clearance

    def path_msg_to_xy(self, msg):
        return [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]

    def refine_pddl_plan_from_monitor(self, plan_report, monitor_result):
        if monitor_result.satisfied:
            return plan_report
        if monitor_result.min_obstacle_margin >= 0.0:
            return plan_report
        return self.replan_nav2_from_stl_feedback(plan_report, monitor_result)

    def replan_nav2_from_stl_feedback(self, plan_report, monitor_result):
        if not self.enable_stl_replan:
            return plan_report
        if self.replan_in_progress:
            return plan_report
        if self.nav2_replan_count >= self.max_nav2_replans:
            self.get_logger().warn(
                f"STL feedback Nav2 replan limit reached ({self.max_nav2_replans}); "
                "keeping current Nav2 goal"
            )
            return plan_report

        now = self.get_clock().now()
        if self.last_stl_replan_time is not None:
            age_s = (now - self.last_stl_replan_time).nanoseconds / 1e9
            if age_s < self.stl_replan_cooldown_s:
                return plan_report

        if not self.active_waypoints:
            self.get_logger().warn(
                "STL obstacle violation detected, but no active Nav2 waypoints are available to replan"
            )
            return plan_report

        self.nav2_replan_count += 1
        self.last_stl_replan_time = now
        self.replan_in_progress = True
        self.get_logger().warn(
            f"STL obstacle violation near {monitor_result.closest_obstacle}; "
            "requesting Nav2 replan for the existing PDDL waypoints "
            f"({self.nav2_replan_count}/{self.max_nav2_replans})"
        )
        self.record_json_event(
            "nav2_replan_requested",
            {
                "reason": "stl_obstacle_violation",
                "closest_obstacle": monitor_result.closest_obstacle,
                "min_obstacle_margin": monitor_result.min_obstacle_margin,
                "attempt": self.nav2_replan_count,
                "max_attempts": self.max_nav2_replans,
            },
        )

        try:
            self.last_monitor_signature = None
            self.last_nav2_path = None
            self.get_logger().info(
                "Reusing PDDL plan for Nav2 replan: " + " -> ".join(plan_report.get("plan", []))
            )
            costmap_edit_published = self.publish_costmap_edit_for_replan(monitor_result)
            if costmap_edit_published and self.costmap_edit_replan_delay_s > 0.0:
                time.sleep(self.costmap_edit_replan_delay_s)
            self.cancel_active_nav2_goal()
            self.publish_waypoints_path(self.active_waypoints)
            self.send_waypoints(self.active_waypoints)
            self.record_json_event(
                "nav2_replan_sent",
                {
                    "pddl_plan": plan_report.get("plan", []),
                    "nav2_goals": [
                        {"x": x, "y": y}
                        for x, y in self.active_waypoints
                    ],
                },
            )
            return plan_report
        except Exception as exc:
            self.get_logger().error(f"STL feedback Nav2 replan failed: {exc}")
            self.record_json_event("nav2_replan_failed", {"error": str(exc)})
            return plan_report
        finally:
            self.replan_in_progress = False

    def publish_costmap_edit_for_replan(self, monitor_result):
        if not self.enable_stl_costmap_edit:
            return False
        if self.map_msg is None:
            self.get_logger().warn(
                "STL feedback requested a Nav2 replan, but no OccupancyGrid is available to edit"
            )
            return False
        if monitor_result.closest_obstacle_xy is None:
            self.get_logger().warn(
                "STL feedback requested a Nav2 replan, but the violating obstacle has no position"
            )
            return False

        edited_map = deepcopy(self.map_msg)
        edited_map.header.stamp = self.get_clock().now().to_msg()
        stamp_key = (edited_map.header.stamp.sec, edited_map.header.stamp.nanosec)
        self.published_costmap_edit_stamps.add(stamp_key)
        if len(self.published_costmap_edit_stamps) > 20:
            self.published_costmap_edit_stamps = set(list(self.published_costmap_edit_stamps)[-20:])

        radius = max(
            self.costmap_edit_min_radius,
            self.clearance_for_obstacle_type(monitor_result.closest_obstacle_type)
            + self.costmap_edit_padding,
            -monitor_result.min_obstacle_margin + self.costmap_edit_padding,
        )
        self.log_costmap_inflation_reason(monitor_result, radius)
        changed_cells = self.paint_occupied_disk(
            edited_map,
            monitor_result.closest_obstacle_xy,
            radius,
            self.costmap_edit_occupied_value,
        )
        if changed_cells == 0:
            self.get_logger().warn(
                "STL costmap edit did not touch any cells; obstacle may be outside the map"
            )
            return False

        for _ in range(self.costmap_edit_publish_repeats):
            self.costmap_edit_pub.publish(edited_map)

        self.get_logger().warn(
            f"Published STL costmap edit on {self.costmap_edit_topic}: "
            f"center=({monitor_result.closest_obstacle_xy[0]:.2f}, "
            f"{monitor_result.closest_obstacle_xy[1]:.2f}), radius={radius:.2f} m, "
            f"cells={changed_cells}"
        )
        self.record_json_event(
            "stl_costmap_edit_published",
            {
                "topic": self.costmap_edit_topic,
                "center": {
                    "x": monitor_result.closest_obstacle_xy[0],
                    "y": monitor_result.closest_obstacle_xy[1],
                },
                "radius": radius,
                "occupied_value": self.costmap_edit_occupied_value,
                "changed_cells": changed_cells,
                "closest_obstacle": monitor_result.closest_obstacle,
                "closest_obstacle_type": monitor_result.closest_obstacle_type,
            },
        )
        return True

    def log_costmap_inflation_reason(self, monitor_result, radius):
        obstacle_type = monitor_result.closest_obstacle_type or "unknown"
        required_clearance = self.clearance_for_obstacle_type(obstacle_type)
        violations = "; ".join(monitor_result.violations) if monitor_result.violations else "none"
        self.get_logger().warn(
            "Inflating costmap for STL-triggered Nav2 replan: "
            f"reason=obstacle_clearance_violation, "
            f"closest_obstacle={monitor_result.closest_obstacle}, "
            f"type={obstacle_type}, "
            f"location=({monitor_result.closest_obstacle_xy[0]:.2f}, "
            f"{monitor_result.closest_obstacle_xy[1]:.2f}), "
            f"min_margin={monitor_result.min_obstacle_margin:.3f} m, "
            f"required_clearance={required_clearance:.2f} m, "
            f"padding={self.costmap_edit_padding:.2f} m, "
            f"inflation_radius={radius:.2f} m, "
            f"violations={violations}"
        )
        self.record_json_event(
            "stl_costmap_inflation_reason",
            {
                "reason": "obstacle_clearance_violation",
                "closest_obstacle": monitor_result.closest_obstacle,
                "closest_obstacle_type": obstacle_type,
                "location": {
                    "x": monitor_result.closest_obstacle_xy[0],
                    "y": monitor_result.closest_obstacle_xy[1],
                },
                "min_margin": monitor_result.min_obstacle_margin,
                "required_clearance": required_clearance,
                "padding": self.costmap_edit_padding,
                "inflation_radius": radius,
                "violations": monitor_result.violations,
            },
        )

    def paint_occupied_disk(self, map_msg, center_xy, radius, occupied_value):
        center_cell = self.world_to_map_cell(map_msg, center_xy)
        if center_cell is None:
            return 0

        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        if width == 0 or height == 0 or resolution <= 0.0:
            return 0

        data = list(map_msg.data)
        center_col, center_row = center_cell
        radius_cells = max(1, int(math.ceil(radius / resolution)))
        radius_sq = radius * radius
        changed_cells = 0

        for row in range(
            max(0, center_row - radius_cells),
            min(height, center_row + radius_cells + 1),
        ):
            for col in range(
                max(0, center_col - radius_cells),
                min(width, center_col + radius_cells + 1),
            ):
                xy = self.map_cell_center_to_world(map_msg, col, row)
                if (xy[0] - center_xy[0]) ** 2 + (xy[1] - center_xy[1]) ** 2 > radius_sq:
                    continue
                idx = row * width + col
                if data[idx] < occupied_value:
                    data[idx] = occupied_value
                    changed_cells += 1

        map_msg.data = data
        return changed_cells

    def world_to_map_cell(self, msg, xy):
        resolution = msg.info.resolution
        if resolution <= 0.0:
            return None
        origin = msg.info.origin
        yaw = self.yaw_from_quaternion(origin.orientation)
        dx = xy[0] - origin.position.x
        dy = xy[1] - origin.position.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        col = int(math.floor(local_x / resolution))
        row = int(math.floor(local_y / resolution))
        if col < 0 or row < 0 or col >= msg.info.width or row >= msg.info.height:
            return None
        return (col, row)

    def clearance_for_obstacle_type(self, obstacle_type):
        if obstacle_type == "human":
            return self.human_clearance
        if obstacle_type == "chair":
            return self.chair_clearance
        if obstacle_type == "shelf":
            return self.shelf_clearance
        if obstacle_type == "column":
            return self.column_clearance
        if obstacle_type == "table":
            return self.table_clearance
        if obstacle_type == "frisbe":
            return self.frisbe_clearance
        return self.obstacle_clearance

    def todo_update_skill_library_from_monitor(self, plan_report, monitor_result):
        """TODO: learn/update low-level action contracts from monitor margins."""
        return plan_report

    def map_cell_center_to_world(self, msg, col, row):
        resolution = msg.info.resolution
        origin = msg.info.origin
        local_x = (col + 0.5) * resolution
        local_y = (row + 0.5) * resolution
        yaw = self.yaw_from_quaternion(origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_x = origin.position.x + local_x * cos_yaw - local_y * sin_yaw
        world_y = origin.position.y + local_x * sin_yaw + local_y * cos_yaw
        return (float(world_x), float(world_y))

    def yaw_from_quaternion(self, quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def distance_to_polyline(self, point_xy, path_points):
        if not path_points:
            return float("inf")
        if len(path_points) == 1:
            return math.hypot(point_xy[0] - path_points[0][0], point_xy[1] - path_points[0][1])
        return min(
            self.point_to_segment_distance(point_xy, path_points[idx], path_points[idx + 1])
            for idx in range(len(path_points) - 1)
        )

    def point_to_segment_distance(self, point_xy, start_xy, goal_xy):
        px, py = point_xy
        ax, ay = start_xy
        bx, by = goal_xy
        abx = bx - ax
        aby = by - ay
        denom = abx * abx + aby * aby
        if denom == 0.0:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / denom))
        closest_x = ax + t * abx
        closest_y = ay + t * aby
        return math.hypot(px - closest_x, py - closest_y)

    def append_waypoint(self, waypoints, xy):
        point = (float(xy[0]), float(xy[1]))
        if not waypoints or math.hypot(point[0] - waypoints[-1][0], point[1] - waypoints[-1][1]) > 1e-6:
            waypoints.append(point)

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def build_poses(self, waypoints):
        poses = []
        for idx, (x, y) in enumerate(waypoints):
            if idx < len(waypoints) - 1:
                nx, ny = waypoints[idx + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            poses.append(self.make_pose(x, y, yaw))
        return poses

    def publish_waypoints_path(self, waypoints):
        path_msg = NavPath()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.build_poses(waypoints)
        self.last_waypoints_path = path_msg
        self.waypoints_pub.publish(path_msg)
        self.get_logger().info(f"Published {len(path_msg.poses)} Nav2 goal poses to {self.waypoints_topic}")
        self.record_json_event(
            "nav2_waypoints_published",
            {
                "topic": self.waypoints_topic,
                "goals": [{"x": x, "y": y} for x, y in waypoints],
            },
        )

    def republish_waypoints_path(self):
        if self.last_waypoints_path is None:
            return
        self.last_waypoints_path.header.stamp = self.get_clock().now().to_msg()
        for pose in self.last_waypoints_path.poses:
            pose.header.stamp = self.last_waypoints_path.header.stamp
        self.waypoints_pub.publish(self.last_waypoints_path)

    def send_waypoints(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.build_poses(waypoints)
        future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 waypoint goal rejected")
            self.record_json_event("nav2_goal_rejected")
            return
        self.active_goal_handle = goal_handle
        self.get_logger().info("Nav2 waypoint goal accepted")
        self.record_json_event("nav2_goal_accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def cancel_active_nav2_goal(self):
        if self.active_goal_handle is None:
            return
        self.get_logger().info("Canceling active Nav2 waypoint goal before STL feedback replan")
        self.record_json_event("nav2_goal_cancel_requested")
        cancel_future = self.active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_callback)

    def cancel_callback(self, future):
        try:
            cancel_response = future.result()
            if cancel_response.goals_canceling:
                self.get_logger().info("Active Nav2 waypoint goal cancel accepted")
                self.record_json_event("nav2_goal_cancel_accepted")
            else:
                self.get_logger().warn("Active Nav2 waypoint goal cancel returned no canceling goals")
                self.record_json_event("nav2_goal_cancel_empty")
        except Exception as exc:
            self.get_logger().warn(f"Nav2 waypoint goal cancel failed: {exc}")
            self.record_json_event("nav2_goal_cancel_failed", {"error": str(exc)})

    def feedback_callback(self, feedback_msg):
        pass

    def result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f"Missed waypoints: {result.missed_waypoints}")
            self.record_json_event(
                "nav2_goal_finished",
                {"missed_waypoints": list(result.missed_waypoints)},
            )
        else:
            self.get_logger().info("All PPDDL Nav2 STL-SAT waypoints completed successfully")
            self.record_json_event("nav2_goal_finished", {"missed_waypoints": []})


def main(args=None):
    rclpy.init(args=args)
    node = PpddlNav2StlSat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
