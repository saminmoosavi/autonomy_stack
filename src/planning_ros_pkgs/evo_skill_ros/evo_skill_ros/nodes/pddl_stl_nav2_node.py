#!/usr/bin/env python3
'''
ros2 run evo_skill_ros pddl_stl_nav2 --ros-args \
  -p ns:=/a200_0000 \
  -p pose_topic:=/a200_0000/platform/odom \
  -p pose_msg_type:=odometry \
  -p target_region:=R10

  
ros2 run evo_skill_ros pddl_stl_nav2 --ros-args \
  -p pose_topic:=/amcl_pose \
  -p pose_msg_type:=amcl \
  -p target_region:=R10

'''
import json
import math
import re
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath

from evo_skill_ros.pddl_stl.pipeline import (
    EvolutionaryPlanEditor,
    ProblemState,
    SkillLibrary,
    dispatch_receding_horizon,
    parse_domain,
    solve_symbolic_prefix,
    validate_plan,
)


class GraphPddlStlNav2(Node):
    def __init__(self):
        super().__init__("graph_pddl_stl_nav2")

        self.ns = self.declare_parameter("ns", "/a200_0000").value
        self.pose_topic = self.declare_parameter("pose_topic", f"{self.ns}/platform/odom").value
        self.pose_msg_type = self.declare_parameter("pose_msg_type", "odometry").value
        self.map_topic = self.declare_parameter("map_topic", f"{self.ns}/map").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.robot_name = self.declare_parameter("robot_name", "jackal1").value.lower()
        self.target_region = self.declare_parameter("target_region", "R10").value.lower()
        self.graph_file = self.resolve_path(self.declare_parameter("graph_file", "config/graph.json").value)
        self.domain_file = self.resolve_path(self.declare_parameter("domain_file", "config/factory_sim_domain.pddl").value)
        self.domain_name = self.read_pddl_domain_name(self.domain_file)
        self.skill_library_file = self.resolve_path(
            self.declare_parameter("skill_library_file", "config/pddl_stl_skill_library.json").value
        )
        self.generations = int(self.declare_parameter("generations", 4).value)
        self.population = int(self.declare_parameter("population", 8).value)
        self.symbolic_prefix = int(self.declare_parameter("symbolic_prefix", 8).value)
        self.horizon = int(self.declare_parameter("horizon", 60).value)
        self.dt = float(self.declare_parameter("dt", 0.1).value)
        self.dispatch_steps = int(self.declare_parameter("dispatch_steps", 20).value)
        self.waypoint_stride = max(1, int(self.declare_parameter("waypoint_stride", 5).value))
        self.min_waypoint_distance = float(self.declare_parameter("min_waypoint_distance", 0.25).value)
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 3.0).value)
        self.map_timeout_s = float(self.declare_parameter("map_timeout_s", 5.0).value)
        self.require_map = bool(self.declare_parameter("require_map", True).value)
        self.occupied_threshold = int(self.declare_parameter("occupied_threshold", 65).value)
        self.map_obstacle_stride = max(1, int(self.declare_parameter("map_obstacle_stride", 2).value))
        self.map_obstacle_corridor_width = float(
            self.declare_parameter("map_obstacle_corridor_width", 2.0).value
        )
        self.max_map_obstacles = max(1, int(self.declare_parameter("max_map_obstacles", 2500).value))
        self.waypoints_topic = self.declare_parameter("waypoints_topic", "pddl_stl_waypoints").value
        self.waypoints_publish_period_s = float(
            self.declare_parameter("waypoints_publish_period_s", 1.0).value
        )

        self.world, self.regions, self.objects = self.load_graph(self.graph_file)
        if self.target_region not in self.regions:
            raise ValueError(f"target_region '{self.target_region}' is not in {self.graph_file}")

        self.current_xy = None
        self.current_region = None
        self.map_msg = None
        self.map_obstacles = {}
        self.last_logged_map_shape = None
        self.last_waypoints_path = None
        self.sent_goal = False
        self.started_at = self.get_clock().now()

        self.client = ActionClient(self, FollowWaypoints, f"{self.ns}/follow_waypoints")
        path_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.waypoints_pub = self.create_publisher(NavPath, self.waypoints_topic, path_qos)
        self.create_pose_subscription()
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, path_qos)
        self.timer = self.create_timer(0.5, self.plan_once_when_ready)
        self.waypoints_timer = self.create_timer(
            self.waypoints_publish_period_s,
            self.republish_waypoints_path,
        )

        self.get_logger().info(
            f"Waiting for {self.pose_msg_type} pose on {self.pose_topic}; target={self.target_region}; "
            f"map_topic={self.map_topic}; waypoints_topic={self.waypoints_topic}"
        )

    def resolve_path(self, value):
        path = Path(value)
        if path.is_absolute() and path.exists():
            return path
        if path.exists():
            return path
        try:
            from ament_index_python.packages import get_package_share_directory

            share_path = Path(get_package_share_directory("evo_skill_ros")) / value
            if share_path.exists():
                return share_path
        except Exception:
            pass
        workspace_path = Path.cwd() / value
        return workspace_path

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

    def format_problem_state(self, problem):
        objects_by_type = {}
        for name, typ in sorted(problem.objects.items()):
            objects_by_type.setdefault(typ, []).append(name)
        object_lines = [
            f"    {' '.join(names)} - {typ}"
            for typ, names in sorted(objects_by_type.items())
        ]
        init_lines = [f"    {self.format_fact(fact)}" for fact in sorted(problem.facts)]
        goal_lines = [f"    {self.format_fact(fact)}" for fact in sorted(problem.goals)]
        return "\n".join([
            "(define (problem generated_factory_nav)",
            f"  (:domain {self.domain_name})",
            "  (:objects",
            *object_lines,
            "  )",
            "  (:init",
            *init_lines,
            "  )",
            "  (:goal (and",
            *goal_lines,
            "  ))",
            ")",
        ])

    def format_plan(self, plan):
        if not plan:
            return "<empty plan>"
        return " -> ".join(action.text() for action in plan)

    def log_stl_traces(self, traces):
        self.get_logger().info(f"STL solve produced {len(traces)} action trace(s)")
        for idx, trace_item in enumerate(traces, start=1):
            contract = trace_item.get("contract", {})
            contracts = contract.get("stl_contracts", [])
            certificates = contract.get("effect_certificates", [])
            solver = contract.get("solver", "symbolic_only" if contract.get("symbolic_only") else "unspecified")
            states = trace_item.get("trace", [])
            self.get_logger().info(
                f"STL action {idx}: {trace_item.get('action')} | solver={solver} | "
                f"solver_status={trace_item.get('solver_status', 'symbolic_only')} | states={len(states)}"
            )
            if trace_item.get("obstacle"):
                self.get_logger().info(f"  nearest obstacle considered: {trace_item['obstacle']}")
            metrics = trace_item.get("metrics", {})
            if metrics:
                self.get_logger().info(
                    "  trace metrics: "
                    f"final_goal_distance={metrics.get('final_goal_distance', float('nan')):.3f} m, "
                    f"min_obstacle_distance={metrics.get('min_obstacle_distance', float('nan')):.3f} m, "
                    f"geometric_stl_satisfied={metrics.get('geometric_stl_satisfied')}"
                )
            if contracts:
                self.get_logger().info("  STL constraints being solved: " + "; ".join(contracts))
            if certificates:
                self.get_logger().info("  effect certificates required: " + "; ".join(certificates))

    def log_dispatch_results(self, executed, certificates):
        self.get_logger().info(
            f"Dispatching receding horizon prefixes: steps_per_action={self.dispatch_steps}, "
            f"actions={len(executed)}"
        )
        for idx, (item, cert) in enumerate(zip(executed, certificates), start=1):
            self.get_logger().info(
                f"Dispatch prefix {idx}: {item.get('action')} | "
                f"states={len(item.get('states', []))} | "
                f"solver_status={item.get('solver_status')} | "
                f"effect_certificate={cert.get('effect_certificate')}"
            )

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
        self.map_msg = msg
        map_shape = (msg.info.width, msg.info.height, msg.info.resolution, msg.header.frame_id)
        if map_shape != self.last_logged_map_shape:
            self.last_logged_map_shape = map_shape
            self.get_logger().info(
                f"Received ROS map {msg.info.width}x{msg.info.height}, "
                f"resolution={msg.info.resolution:.3f}, frame={msg.header.frame_id or self.frame_id}"
            )

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
                self.get_logger().warn(
                    f"Waiting for occupancy grid on {self.map_topic}; STL obstacle constraints need the ROS map"
                )
            return
        if not self.client.wait_for_server(timeout_sec=0.2):
            self.get_logger().info(f"Waiting for Nav2 action server {self.ns}/follow_waypoints")
            return

        self.get_logger().info(
            f"Starting PDDL-STL navigation pipeline from {self.current_region} to {self.target_region}"
        )
        try:
            report = self.run_graph_pipeline()
        except Exception as exc:
            self.get_logger().error(f"PDDL-STL pipeline failed: {exc}")
            self.sent_goal = True
            return

        if not report["ok"]:
            self.get_logger().error(f"No validated PDDL-STL plan found: {report.get('history', [])[-3:]}")
            self.sent_goal = True
            return

        waypoints = self.extract_waypoints(report["executed_prefixes"])
        if not waypoints:
            self.get_logger().error("Validated plan produced no executable Nav2 waypoints")
            self.sent_goal = True
            return

        self.get_logger().info(
            "Validated plan: " + " -> ".join(report["plan"])
        )
        self.get_logger().info(f"Publishing {len(waypoints)} receding-horizon waypoints to Nav2")
        self.get_logger().info(
            "Nav2 waypoints: "
            + ", ".join(f"({x:.2f}, {y:.2f})" for x, y in waypoints)
        )
        self.publish_waypoints_path(waypoints)
        self.send_waypoints(waypoints)
        self.sent_goal = True

    def make_problem_from_graph(self):
        self.get_logger().info(
            f"Step 2/6: building PDDL problem from graph={self.graph_file}, "
            f"robot={self.robot_name}, start={self.current_region}, goal={self.target_region}"
        )
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

        problem = ProblemState(
            objects=objects,
            facts=facts,
            goals={("visited", self.target_region)},
        )
        self.get_logger().info("Generated PDDL problem:\n" + self.format_problem_state(problem))
        return problem

    def run_graph_pipeline(self):
        self.get_logger().info(f"Step 1/6: loading PDDL domain from {self.domain_file}")
        domain = parse_domain(self.domain_file)
        self.get_logger().info(
            f"Loaded PDDL domain '{self.domain_name}' with actions: "
            + ", ".join(sorted(domain.actions.keys()))
        )
        problem = self.make_problem_from_graph()
        self.get_logger().info(f"Step 3/6: loading STL skill library from {self.skill_library_file}")
        skill_library = SkillLibrary(self.skill_library_file)
        editor = EvolutionaryPlanEditor(domain, problem)

        best = None
        history = []
        self.get_logger().info(
            f"Step 4/6: searching for a valid PDDL plan with generations={self.generations}, "
            f"population={self.population}"
        )
        for generation in range(self.generations):
            self.get_logger().info(f"PDDL planning generation {generation + 1}/{self.generations}")
            for candidate in editor.propose([], self.population):
                result = validate_plan(domain, problem, candidate, require_goal=True)
                candidate_text = self.format_plan(candidate)
                if result.ok:
                    self.get_logger().info(f"Candidate PDDL plan valid: {candidate_text}")
                else:
                    self.get_logger().info(
                        f"Candidate PDDL plan rejected: {candidate_text} | errors={result.errors}"
                    )
                history.append({
                    "generation": generation,
                    "plan": [action.text() for action in candidate],
                    "ok": result.ok,
                    "errors": result.errors,
                })
                if result.ok:
                    best = (candidate, result)
                    break
            if best:
                break

        if best is None:
            self.get_logger().error("PDDL planning failed: no candidate satisfied the goal")
            return {"ok": False, "history": history}

        plan, _ = best
        self.get_logger().info("Selected PDDL plan: " + self.format_plan(plan))
        prefix_len = min(self.symbolic_prefix, len(plan))
        self.get_logger().info(
            f"Step 5/6: validating symbolic prefix length={prefix_len} "
            f"(requested={self.symbolic_prefix}, plan_length={len(plan)})"
        )
        prefix_validation = validate_plan(domain, problem, plan[:prefix_len], require_goal=False)
        self.get_logger().info(
            f"Validated symbolic prefix actions={prefix_validation.accepted_prefix_len}: "
            + self.format_plan(plan[: prefix_validation.accepted_prefix_len])
        )
        self.get_logger().info(
            f"Step 6/6: solving STL constraints for validated symbolic prefix "
            f"with horizon={self.horizon}, dt={self.dt}"
        )
        obstacle_points = self.build_stl_obstacles_from_map(plan[: prefix_validation.accepted_prefix_len])
        traces = solve_symbolic_prefix(
            plan,
            prefix_validation,
            skill_library,
            self.regions,
            obstacle_points,
            self.horizon,
            self.dt,
        )
        self.log_stl_traces(traces)
        executed, certificates = dispatch_receding_horizon(traces, self.dispatch_steps)
        self.log_dispatch_results(executed, certificates)
        return {
            "ok": True,
            "plan": [action.text() for action in plan],
            "validated_symbolic_prefix": [action.text() for action in plan[: prefix_validation.accepted_prefix_len]],
            "executed_prefixes": executed,
            "certificates": certificates,
            "start_region": self.current_region,
            "target_region": self.target_region,
        }

    def build_stl_obstacles_from_map(self, plan_prefix):
        if self.map_msg is None:
            self.get_logger().warn(
                "No ROS map available; falling back to graph.json object coordinates for STL obstacles"
            )
            return self.objects

        segments = []
        for action in plan_prefix:
            if not action.name.startswith("move") or len(action.args) < 3:
                continue
            start_region = action.args[-2]
            goal_region = action.args[-1]
            if start_region in self.regions and goal_region in self.regions:
                segments.append((start_region, goal_region, self.regions[start_region], self.regions[goal_region]))

        if not segments:
            self.get_logger().warn(
                "Validated PDDL prefix has no graph-labeled move segments; using graph.json object coordinates"
            )
            return self.objects

        obstacle_points = {}
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
                nearest = self.nearest_plan_segment(xy, segments)
                if nearest is None:
                    continue
                start_label, goal_label, distance = nearest
                if distance > self.map_obstacle_corridor_width:
                    continue
                label = f"map_occ_{start_label}_to_{goal_label}_{col}_{row}"
                obstacle_points[label] = xy
                if len(obstacle_points) >= self.max_map_obstacles:
                    break
            if len(obstacle_points) >= self.max_map_obstacles:
                break

        if not obstacle_points:
            self.get_logger().warn(
                "ROS map had no occupied cells near graph-labeled plan segments; "
                "falling back to graph.json object coordinates for STL obstacles"
            )
            return self.objects

        self.map_obstacles = obstacle_points
        self.get_logger().info(
            f"Generated STL obstacle constraints from ROS map: occupied_points={len(obstacle_points)}, "
            f"source={self.map_topic}, graph_labels="
            + ", ".join(f"{start}->{goal}" for start, goal, _, _ in segments)
        )
        return obstacle_points

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

    def nearest_plan_segment(self, xy, segments):
        best = None
        best_distance = float("inf")
        for start_label, goal_label, start_xy, goal_xy in segments:
            distance = self.point_to_segment_distance(xy, start_xy, goal_xy)
            if distance < best_distance:
                best = (start_label, goal_label, distance)
                best_distance = distance
        return best

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

    def extract_waypoints(self, executed_prefixes):
        waypoints = []
        for item in executed_prefixes:
            states = item.get("states", [])
            if not states:
                continue
            sampled = states[:: self.waypoint_stride]
            if sampled[-1] != states[-1]:
                sampled.append(states[-1])
            for state in sampled:
                self.append_waypoint(waypoints, (float(state[0]), float(state[1])))
        return waypoints

    def append_waypoint(self, waypoints, xy):
        if not waypoints:
            waypoints.append(xy)
            return
        last_x, last_y = waypoints[-1]
        if math.hypot(xy[0] - last_x, xy[1] - last_y) >= self.min_waypoint_distance:
            waypoints.append(xy)

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
        self.get_logger().info(
            f"Published {len(path_msg.poses)} waypoint poses to {self.waypoints_topic} for RViz"
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
            return
        self.get_logger().info("Nav2 waypoint goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f"Missed waypoints: {result.missed_waypoints}")
        else:
            self.get_logger().info("All PDDL-STL waypoints completed successfully")


def main(args=None):
    rclpy.init(args=args)
    node = GraphPddlStlNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
