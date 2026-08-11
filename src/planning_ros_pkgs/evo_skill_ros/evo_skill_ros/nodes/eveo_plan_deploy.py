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

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from std_msgs.msg import String
from vision_msgs.msg import Detection3D

from evo_skill_ros.pddl_stl.pipeline import (
    GroundAction,
    ProblemState,
    parse_domain,
    parse_problem,
    validate_plan,
)
from evoplan_bridge import symbolic_replan
from evoplan_bridge.inspection import inspection_pose, segment_end, yaw_error
from evoplan_bridge.object_registry import ObjectRegistry
from evoplan_bridge.observation_memory import (
    cluster_detections,
    load_observations,
    locate_object,
)
from evoplan_bridge.phi_mob_shield import PhiMobShield
from evoplan_bridge.replan_artifacts import next_index, safe_tag, write_artifacts
from evoplan_bridge.replan_client import ReplanClient
from evoplan_bridge.symbolic_replan import (
    SymbolicReplanState,
    align_plan_start,
    build_reason_text,
    derive_blocked_regions,
    filter_executable_actions,
    plan_reaches_target,
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


def _finite_or_none(value):
    """Map non-finite floats to ``None`` for the JSON event log.

    Shield margins are ``inf`` whenever a signal is undefined (an infinite TTC
    with nobody ahead is the common case). ``json.dumps`` would emit a bare
    ``Infinity`` token, which Python re-reads but which is not valid JSON and
    breaks any other consumer of ``evo_plan_deploy_log.json``. ``null`` says
    "not measured" unambiguously.
    """
    return value if isinstance(value, (int, float)) and math.isfinite(value) else None


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
        self.plan_file = self.resolve_path(self.declare_parameter("plan_file", "config/evo_plan.txt").value)
        self.domain_name = self.read_pddl_domain_name(self.domain_file)

        self.fast_downward_cmd = self.declare_parameter("fast_downward_cmd", "fast-downward.py").value
        self.fast_downward_search = self.declare_parameter("fast_downward_search", "astar(lmcut())").value
        self.fast_downward_timeout_s = float(
            self.declare_parameter("fast_downward_timeout_s", 30.0).value
        )
        # 3.0 was too impatient: odometry publishes at 30 Hz once the Gazebo
        # bridge is up, but that can take longer than three seconds after the
        # node starts, and the fallback then guesses the start region from
        # graph.json. It happened to be right (the robot spawns at r5), which is
        # exactly what makes it dangerous -- a wrong guess sends the whole plan
        # off from the wrong origin. The fallback should be a last resort.
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 20.0).value)
        self.map_timeout_s = float(self.declare_parameter("map_timeout_s", 5.0).value)
        # A lifecycle-managed action server is DISCOVERABLE from on_configure but
        # only ACCEPTS goals from on_activate, so wait_for_server() returning
        # true proves nothing about whether a goal will be taken. One rejected
        # goal used to end the mission silently: the robot never moved while
        # Gazebo, Nav2 and all four cameras looked healthy. Retry instead.
        self.goal_reject_max_retries = int(
            self.declare_parameter("goal_reject_max_retries", 15).value
        )
        self.goal_reject_retry_s = float(
            self.declare_parameter("goal_reject_retry_s", 2.0).value
        )
        self._goal_reject_retries = 0
        self._pending_waypoints = None
        self._goal_retry_timer = None
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
        self.costmap_edit_max_radius = float(
            self.declare_parameter("costmap_edit_max_radius", 1.0).value
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

        # --- Phi_mob runtime shield -------------------------------------
        # Off by default: with this false the node behaves exactly as it did
        # before the shield existed.
        self.enable_phi_mob_shield = bool(
            self.declare_parameter("enable_phi_mob_shield", False).value
        )
        self.shield_horizon_s = float(self.declare_parameter("shield_horizon_s", 3.0).value)
        self.shield_period_s = float(self.declare_parameter("shield_period_s", 0.1).value)
        # A single YOLO false positive should not be able to declare a
        # violation; the robustness must stay negative for this long first.
        self.shield_violation_persist_s = float(
            self.declare_parameter("shield_violation_persist_s", 1.0).value
        )

        # --- Tier-2 online symbolic replan ------------------------------
        # Also off by default. With both switches false this node is
        # byte-for-byte the executor it was before the merge.
        self.enable_symbolic_replan = bool(
            self.declare_parameter("enable_symbolic_replan", False).value
        )
        self.replan_service_url = self.declare_parameter(
            "replan_service_url", "http://127.0.0.1:8077"
        ).value
        self.replan_service_timeout_s = float(
            self.declare_parameter("replan_service_timeout_s", 60.0).value
        )
        # Wall seconds: the service is wall-clock, so its deadline must be too.
        self.symbolic_replan_deadline_s = float(
            self.declare_parameter("symbolic_replan_deadline_s", 45.0).value
        )
        self.max_symbolic_replans = max(
            0, int(self.declare_parameter("max_symbolic_replans", 2).value)
        )
        # Sim seconds: paced against the world the robot lives in.
        self.symbolic_replan_cooldown_s = float(
            self.declare_parameter("symbolic_replan_cooldown_s", 30.0).value
        )
        self.mission_deliberation_budget_s = float(
            self.declare_parameter("mission_deliberation_budget_s", 120.0).value
        )
        self.mission_id = self.declare_parameter("mission_id", "factory_mission_01").value
        self.planner_mode = self.declare_parameter("planner_mode", "evoplan").value
        # Identifies THIS trial to the replan service, which is host-side, long
        # lived (KEEP_SERVICE=1) and serves many runs -- mission_id alone would
        # make run N overwrite run N-1's archived problems. Set from the trial's
        # artifact stem so the .pddl/.plan/.json land beside its
        # _observations.jsonl and _belief.json. Empty is tolerated: the service
        # falls back to mission_id + job id rather than dropping the record.
        self.trial_tag = self.declare_parameter("trial_tag", "").value
        # Where <tag>_replan_N.{pddl,plan,json} go. The service writes 1..N to
        # the same directory on the host; this is the container's view of it,
        # via the repo bind-mount, so index 0 lands beside them.
        self.replan_artifact_dir = self.declare_parameter(
            "replan_artifact_dir", "results/single_trials").value
        # The authored mission problem, archived as replan_0's .pddl. Empty
        # means "find <mission_id>.pddl" in the usual two places.
        self.mission_problem_file = self.declare_parameter(
            "mission_problem_file", "").value
        self.hold_on_replan = bool(self.declare_parameter("hold_on_replan", True).value)
        # Distance to the target region within which the mission counts as
        # arrived: escalation stops and the trial may terminate.
        self.goal_arrival_radius_m = float(
            self.declare_parameter("goal_arrival_radius_m", 0.5).value
        )
        # Counting a region as VISITED is a looser test than "parked at the
        # target". Nav2's general_goal_checker uses xy_goal_tolerance: 0.3, but
        # that is applied to the controller's goal, and a FollowWaypoints
        # intermediate waypoint can be declared reached from further out -- so a
        # radius equal to the arrival radius under-counts. Keep them separate.
        # MEASURED, not guessed: region_closest_m from a real tour showed the
        # closest approach to a COMPLETED waypoint was 1.90 m, so 0.5 and 1.25
        # both scored a perfect tour as 0/14. 2.5 m clears that with margin and
        # stays far below the ~11 m inter-region spacing, so it cannot credit
        # the wrong region.
        self.region_visit_radius_m = float(
            self.declare_parameter("region_visit_radius_m", 2.5).value
        )
        # Closest approach to each required region, for diagnosing under-counts.
        self._region_closest = {}
        # Sim-time the robot finished its last plan, or None while driving.
        self._plan_finished_sim_s = None
        self.plan_exhausted = False
        # Grace period before calling a finished plan terminal: a replan may
        # still be in flight, or about to be triggered by the arrival itself.
        # 0.0 terminates the instant the plan is exhausted, with no wait for a
        # possible replan. Useful for batch runs where a hang costs more than a
        # missed late replan.
        self.plan_exhaustion_grace_s = float(
            self.declare_parameter("plan_exhaustion_grace_s", 10.0).value
        )
        # Hard mission cap in SIM seconds; 0 disables. Independent of the
        # trial-level wall timeout, which cannot distinguish "still working"
        # from "wedged".
        self.mission_timeout_s = float(
            self.declare_parameter("mission_timeout_s", 1800.0).value
        )
        # End the run as soon as the symbolic replan budget is spent and the
        # plan is finished, rather than waiting out the grace period.
        self.end_on_replans_exhausted = bool(
            self.declare_parameter("end_on_replans_exhausted", False).value
        )
        # --- find-an-object missions (search tour, then approach) ---
        # Set find_object_class to run the mission in two phases: drive the
        # tour to completion while the observation logger records what is seen
        # where, then look the object up in that log and replan an approach to
        # whichever region actually held it. The region is NOT known when the
        # mission starts -- that is the point -- so it cannot be a PDDL goal
        # written up front. Empty disables the whole thing and the mission
        # behaves exactly as before.
        self.find_object_class = str(
            self.declare_parameter("find_object_class", "").value or ""
        ).strip().lower()
        self.find_object_obs_log = str(
            self.declare_parameter("find_object_obs_log", "").value or ""
        )
        # Evidence thresholds. Deliberately the module defaults: a live run
        # produced spurious labels, and a false negative here just ends the
        # mission after the tour, while a false positive sends the robot to
        # the wrong region and calls it success.
        self.find_object_min_hits = int(
            self.declare_parameter("find_object_min_hits", 3).value
        )
        self.find_object_min_score = float(
            self.declare_parameter("find_object_min_score", 0.5).value
        )
        # How often to re-read observation memory while the survey is being
        # driven. The mission problem is unsolvable until the object's region
        # is known, so this poll is what makes it solvable -- and it runs
        # DURING the drive, not after it, so the approach can be planned
        # against a real position the moment YOLO supplies one.
        self.find_object_poll_s = float(
            self.declare_parameter("find_object_poll_s", 2.0).value
        )
        #: "disabled" | "search" | "approach" | "done" | "not_found"
        self.find_phase = "search" if self.find_object_class else "disabled"
        self.find_object_region = None
        self.find_object_evidence = None
        #: Region last reported to the replan service, so a steady stream of
        #: detections produces one report rather than one per poll.
        self._reported_object_region = None

        # --- find-and-INSPECT missions (open world) ------------------------
        # A different shape of mission from find_object_class above, not a
        # variant of it. There the mission names ONE object known to exist and
        # asks where it is; here it names one or more CLASSES and asks how many
        # there are, where each is, and requires the robot to stop and look at
        # every one. The quantity is the unknown, so no PDDL problem can declare
        # the objects up front -- the executor mints them from perception and
        # splices them into the problem as they turn up.
        #
        # Comma-separated because a mission may hunt several classes at once
        # ("traffic cone,chair"), which the single-object path cannot express.
        # Empty disables all of it.
        self.inspect_object_classes = [
            part.strip().lower()
            for part in str(
                self.declare_parameter("inspect_object_classes", "").value or ""
            ).split(",")
            if part.strip()
        ]
        # How long the robot holds still, facing an object, for the inspection
        # to count. The mission's unit of work: 5 s is long enough that a human
        # watching the run can see it happen and long enough for a stationary
        # camera to accumulate frames, and short enough that ten objects do not
        # exhaust the trial timeout.
        self.inspect_dwell_s = float(
            self.declare_parameter("inspect_dwell_s", 5.0).value
        )
        # Detections closer than this are treated as ONE object. 5 m rather than
        # the sensor-noise figure it started at: depth projection scattered one
        # cone into two clusters 2.5 m apart and the robot inspected the phantom.
        # See observation_memory.DEFAULT_LINK_RADIUS_M for the trade this makes.
        self.inspect_cluster_radius_m = float(
            self.declare_parameter("inspect_cluster_radius_m", 5.0).value
        )
        # How far a cluster centroid may move between polls and still be the
        # same object. Tracks the link radius; see object_registry.
        self.inspect_merge_radius_m = float(
            self.declare_parameter("inspect_merge_radius_m", 5.0).value
        )
        # Stand-off from the object along the line from its region's centroid.
        # 0.0 means "drive to the region centroid and turn to face the object",
        # which is the specified behaviour and the safe one: region centroids
        # are known-drivable waypoints, whereas a computed stand-off pose can
        # land inside the very obstacle being inspected. Raise it only for a
        # mission where the objects are far from their region's centre.
        self.inspect_standoff_m = float(
            self.declare_parameter("inspect_standoff_m", 0.0).value
        )
        # Yaw error above which the dwell rotates in place before it starts
        # counting. Nav2's own goal checker already aligns the robot to the
        # inspect pose, so this is a correction, not the primary mechanism --
        # but "facing the object" is the requirement, and a goal checker
        # tolerance this node does not own is not something to rely on.
        self.inspect_face_tolerance_rad = float(
            self.declare_parameter("inspect_face_tolerance_rad", 0.15).value
        )
        self.inspect_face_speed = float(
            self.declare_parameter("inspect_face_speed", 0.5).value
        )
        # Bound on the turn-to-face, so a bad yaw estimate cannot spin the robot
        # for the rest of the mission.
        self.inspect_face_timeout_s = float(
            self.declare_parameter("inspect_face_timeout_s", 8.0).value
        )
        # Evidence thresholds for admitting an object to the registry. Default
        # to the find-object ones so the two paths cannot silently disagree
        # about what counts as seen.
        self.inspect_min_hits = int(
            self.declare_parameter("inspect_min_hits", self.find_object_min_hits).value
        )
        self.inspect_min_score = float(
            self.declare_parameter("inspect_min_score", self.find_object_min_score).value
        )
        # A cap on how much the mission can grow. Each object adds a goal
        # conjunct and a leg of driving; an unbounded registry fed by a
        # mis-tuned detector turns a five-minute trial into an endless one.
        self.inspect_max_objects = int(
            self.declare_parameter("inspect_max_objects", 12).value
        )
        # Sim seconds to wait before re-asking for an inspection plan after a
        # round failed. Long enough that a 1 Hz tick cannot spam the service,
        # short enough to fit several attempts inside a mission.
        self.inspect_round_retry_s = float(
            self.declare_parameter("inspect_round_retry_s", 15.0).value
        )
        # How many inspection rounds may be attempted in total. Bounds the retry
        # loop: an object the planner simply cannot reach must not keep the run
        # alive forever. Rounds that SUCCEED consume one each too, which is
        # right -- each is a real deliberation with a real cost.
        self.inspect_max_rounds = int(
            self.declare_parameter("inspect_max_rounds", 6).value
        )
        #: "disabled" | "survey" | "inspect" | "done" | "not_planned"
        #: "done" means everything found was inspected; "not_planned" means the
        #: round budget ran out with objects still pending. The two are very
        #: different results and must not both read as "finished".
        self.inspect_phase = "survey" if self.inspect_object_classes else "disabled"
        self.object_registry = (
            ObjectRegistry(merge_radius_m=self.inspect_merge_radius_m)
            if self.inspect_object_classes else None
        )
        #: Set while the robot is standing still looking at an object.
        self._dwell = None
        self._dwell_timer = None
        #: Objects reported to the replan service, so a poll that discovers
        #: nothing new sends nothing.
        self._reported_object_names = set()
        #: True while an inspection round's replan is IN FLIGHT, so the 1 Hz
        #: exhaustion tick does not stack a second request on the first. It is
        #: cleared on failure as well as on success -- see
        #: inspection_retry_possible for why that distinction cost a whole run.
        self._inspection_replan_pending = False
        #: Sim-time of the last inspection round escalation, and how many have
        #: been attempted. A failed round is retried after a cooldown rather
        #: than either re-requested every tick or abandoned forever.
        self._last_inspection_round_sim_s = None
        self._inspection_round_attempts = 0
        #: "symbolic replan is off" is a permanent condition, so say it once.
        self._inspection_unplannable_logged = False

        self.cmd_vel_topic = self.declare_parameter(
            "cmd_vel_topic", f"{self.ns}/cmd_vel"
        ).value
        self.stuck_window_s = float(self.declare_parameter("stuck_window_s", 20.0).value)
        self.stuck_min_displacement_m = float(
            self.declare_parameter("stuck_min_displacement_m", 0.3).value
        )
        self.evoplan_status_topic = self.declare_parameter(
            "evoplan_status_topic", f"{self.ns}/evoplan/status"
        ).value
        # Debug lever: set true at runtime with `ros2 param set` to force one
        # escalation, so the hot-swap path can be exercised without having to
        # choreograph a pedestrian.
        self.declare_parameter("force_symbolic_replan", False)

        self.world, self.regions, self.objects = self.load_graph(self.graph_file)
        if self.target_region not in self.regions:
            raise ValueError(f"target_region '{self.target_region}' is not in {self.graph_file}")

        self.current_xy = None
        self.current_yaw = None
        self.current_region = None
        self.map_msg = None
        self.tracked_objects = {}
        self.last_logged_map_shape = None
        self.last_waypoints_path = None
        self.last_nav2_path = None
        self.active_plan_report = None
        self.active_plan_actions = None
        self.active_waypoints = None
        #: One entry per waypoint in ``active_waypoints``, describing what the
        #: robot is meant to DO on arrival. ``None`` for a plain move; a dict
        #: ``{"object", "class", "xy", "region"}`` for an inspection, which is
        #: what turns a waypoint into a facing pose plus a dwell. Kept parallel
        #: rather than folded into the waypoint tuple because every existing
        #: consumer -- the RViz path, the STL monitor, the reactive replan --
        #: unpacks waypoints as ``(x, y)``; ``set_route``/``slice_route`` are
        #: the only places allowed to touch the two lists, so they cannot drift.
        self.waypoint_tasks = []
        #: The action behind each waypoint of the CURRENT route, never sliced;
        #: index it with ``waypoint index + _waypoints_offset``.
        self.route_actions = []
        #: Length of the goal currently with Nav2, which is a leading SEGMENT of
        #: the route when the plan contains inspections. See ``route_segment``.
        self._segment_len = 0
        self.last_monitor_signature = None
        self.sent_goal = False
        self.active_goal_handle = None
        self.replan_in_progress = False
        self.nav2_replan_count = 0
        self.last_stl_replan_time = None
        self.json_log_events = []
        self.json_log_failed = False
        self.published_costmap_edit_stamps = set()
        self.active_costmap_edit = None
        self.last_tracked_obstacle_log_time = None
        self.last_monitor_obstacle_log_time = None
        self.started_at = self.get_clock().now()
        self._last_feedback_waypoint = None
        self._waypoints_offset = 0

        self.shield = PhiMobShield(horizon_s=self.shield_horizon_s)
        self.shield_veto_count = 0
        self.shield_violation_since = None
        self.shield_violation_active = False
        self.last_shield_result = None

        self.symbolic = SymbolicReplanState()
        self.replan_client = ReplanClient(
            self.replan_service_url,
            http_timeout_s=self.replan_service_timeout_s,
            logger=self.get_logger(),
        )
        self._replan_started_wall = None
        self._giveup_reasons_logged = set()
        # Mission-completion tracking, anchored to the ORIGINAL plan.
        self.mission_required_regions = []   # move destinations of the initial plan
        self.mission_visited_regions = set() # regions actually reached
        self.mission_complete = False
        self._stuck_anchor = None  # (x, y, sim_seconds)
        self.hold_timer = None

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
        self.costmap_restore_timer = self.create_timer(0.5, self.restore_costmap_if_obstacle_cleared)
        if self.enable_phi_mob_shield:
            self.shield_timer = self.create_timer(self.shield_period_s, self.shield_tick)
        # The status blob carries mission_complete / plan_exhausted, which
        # scand_metrics gates success and termination on. Publishing it only
        # when Tier-2 was enabled meant any SYMBOLIC_REPLAN=0 run reported
        # mission_complete: None and silently fell back to the positional test.
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.evoplan_status_topic, 10)
        self.status_timer = self.create_timer(1.0, self.publish_evoplan_status)
        if self.enable_symbolic_replan:
            # Drains the replan worker's result queue. Every mutation of plan
            # bookkeeping happens from this callback, on the executor thread.
            self.symbolic_timer = self.create_timer(0.2, self.symbolic_replan_tick)
            self.stuck_timer = self.create_timer(1.0, self.stuck_check_tick)
        self.exhaustion_timer = self.create_timer(1.0, self.plan_exhaustion_tick)
        if self.find_object_class:
            self.object_watch_timer = self.create_timer(
                self.find_object_poll_s, self.object_watch_tick)
        if self.inspect_object_classes:
            # Same cadence as the single-object watch and for the same reason:
            # discoveries must reach the problem while the survey is still
            # being driven, not once it has run out.
            self.inspection_watch_timer = self.create_timer(
                self.find_object_poll_s, self.inspection_watch_tick)
            self.get_logger().info(
                f"Find-and-inspect mission: classes={self.inspect_object_classes}, "
                f"dwell={self.inspect_dwell_s:.1f}s, standoff={self.inspect_standoff_m:.2f}m, "
                f"observations={self.find_object_obs_log!r}"
            )
            # Said now rather than twenty minutes from now, when the survey ends
            # and there is nothing that can plan the inspections.
            if not self.enable_symbolic_replan:
                self.get_logger().error(
                    "inspect_object_classes is set but enable_symbolic_replan is "
                    "false. The objects do not exist as PDDL symbols until they "
                    "are discovered, so the inspections can only be REPLANNED -- "
                    "this run will survey and then stop."
                )
            if not self.find_object_obs_log:
                self.get_logger().error(
                    "inspect_object_classes is set but find_object_obs_log is "
                    "empty; there is no observation log to search and nothing "
                    "will ever be discovered."
                )
        self.mission_timeout_timer = self.create_timer(2.0, self.mission_timeout_tick)

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

    def parse_plan_text(self, text):
        """Parse ``(action arg ...)`` lines into GroundActions.

        Split out of :meth:`parse_fast_downward_plan` so a plan arriving from
        the replan service over HTTP goes through exactly the same parser as one
        read off disk -- the two must never diverge in what they accept.
        """
        actions = []
        for line in text.splitlines():
            line = line.strip().lower()
            if not line or line.startswith(";"):
                continue
            match = re.search(r"\(([^)]+)\)", line)
            if not match:
                continue
            parts = match.group(1).split()
            if not parts:
                continue
            actions.append(GroundAction(parts[0], tuple(parts[1:])))
        return actions

    def parse_fast_downward_plan(self, path):
        actions = self.parse_plan_text(Path(path).read_text())
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
        self.update_current_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def pose_callback(self, msg):
        self.update_current_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def update_current_pose(self, x, y, yaw=None):
        self.current_xy = (float(x), float(y))
        # Kept, not just forwarded to the shield: the inspection dwell turns the
        # robot to face its object and needs to know which way it is pointing.
        if yaw is not None:
            self.current_yaw = float(yaw)
        self.current_region, distance = self.nearest_region(self.current_xy)
        self.get_logger().debug(
            f"Current pose ({x:.2f}, {y:.2f}) snapped to {self.current_region} at distance {distance:.2f}"
        )
        # Mission progress is "which required regions have actually been
        # reached", not "where am I now" -- the latter cannot distinguish the
        # first visit to a region from the last in a plan that loops back.
        if self.current_region and self.mission_required_regions:
            target_xy = self.regions.get(self.current_region)
            if target_xy is not None:
                dist = math.hypot(float(x) - target_xy[0], float(y) - target_xy[1])
                prev = self._region_closest.get(self.current_region)
                if prev is None or dist < prev:
                    self._region_closest[self.current_region] = dist
            if target_xy is not None and math.hypot(
                    float(x) - target_xy[0], float(y) - target_xy[1]
            ) <= self.region_visit_radius_m:
                if self.current_region not in self.mission_visited_regions:
                    self.mission_visited_regions.add(self.current_region)
                    remaining = self.regions_remaining()
                    self.get_logger().info(
                        f"[MISSION] reached {self.current_region} "
                        f"({len(self.mission_visited_regions)}/"
                        f"{len(set(self.mission_required_regions))} regions, "
                        f"{len(remaining)} left)"
                    )
                self.update_mission_completion()

        # Phi_mob needs heading to tell an approaching pedestrian from one the
        # robot is driving away from, so the shield is fed here rather than
        # from a pose sample that has already discarded orientation.
        if self.enable_phi_mob_shield and yaw is not None:
            self.shield.update_odom(
                float(x), float(y), float(yaw), self.sim_time_now()
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

    def sim_time_now(self):
        """Current sim time in seconds.

        The shield's horizon, the violation debounce and every cooldown are in
        *sim* seconds. Using wall time would silently rescale all of them
        whenever Gazebo runs off 1.0x real time, which it routinely does under
        a crowded world plus four YOLO instances.
        """
        return self.get_clock().now().nanoseconds / 1e9

    def human_tracks_for_shield(self):
        """Live human tracks as ``(name, x, y)``, dropping stale ones.

        Reuses the same staleness bound and human classification as
        ``build_tracked_obstacles`` so the shield and the geometric Tier-1
        monitor never disagree about who is present.
        """
        now = self.get_clock().now()
        tracks = []
        for name, tracked in self.tracked_objects.items():
            age_s = (now - tracked.stamp).nanoseconds / 1e9
            if age_s > self.tracked_object_timeout_s:
                continue
            if self.object_type_from_name(tracked.label) != "human":
                continue
            tracks.append((name, tracked.xy[0], tracked.xy[1]))
        return tracks

    def shield_tick(self):
        """Evaluate Phi_mob and record sustained violations.

        Phase 1 only observes -- it counts vetoes and logs which conjunct fired.
        The escalation to a symbolic replan hooks in here in Phase 2.
        """
        if not self.enable_phi_mob_shield:
            return
        now_s = self.sim_time_now()
        self.shield.update_tracks(self.human_tracks_for_shield(), now_s)
        result = self.shield.evaluate(now_s)
        self.last_shield_result = result

        if result.satisfied:
            if self.shield_violation_active:
                held_s = now_s - (self.shield_violation_since or now_s)
                self.shield_violation_active = False
                self.record_json_event(
                    "phi_mob_recovered",
                    {"robustness": result.robustness, "violated_for_s": held_s},
                )
                self.get_logger().info(
                    f"Phi_mob recovered after {held_s:.1f} s "
                    f"(rho={result.robustness:.3f})"
                )
            self.shield_violation_since = None
            return

        self.shield_veto_count += 1
        if self.shield_violation_since is None:
            self.shield_violation_since = now_s
            return
        if now_s - self.shield_violation_since < self.shield_violation_persist_s:
            return
        if self.shield_violation_active:
            return  # already reported this episode

        self.shield_violation_active = True
        self.record_json_event(
            "phi_mob_violation",
            {
                "robustness": result.robustness,
                "worst_conjunct": result.worst_conjunct,
                "persisted_s": now_s - self.shield_violation_since,
                "margins": {k: _finite_or_none(v) for k, v in result.margins.items()},
                "signals": {k: _finite_or_none(v) for k, v in result.signals.items()},
            },
        )
        self.get_logger().warn(
            f"Phi_mob violated: {result.worst_conjunct} "
            f"rho={result.robustness:.3f} "
            f"(sustained {now_s - self.shield_violation_since:.1f} s)"
        )
        # Trigger (a): a sustained social-compliance breach the reactive layer
        # has not resolved warrants a new symbolic route, not just a nudge.
        self.escalate_symbolic_replan(self.active_plan_report, trigger="stl_shield")

    # ==================================================================
    # Tier 2: online symbolic replan
    # ==================================================================
    def regions_remaining(self):
        """Required regions the mission has not visited yet."""
        return sorted(set(self.mission_required_regions) - self.mission_visited_regions)

    def update_mission_completion(self):
        """Mission is complete when every required region has been visited AND
        the robot is parked at the target.

        Requiring the full region set is what stops a shortened replan from
        declaring victory, and requiring the target last is what stops a
        mid-tour pass through the goal region from doing the same.
        """
        if self.mission_complete or not self.mission_required_regions:
            return
        # A find-an-object mission is not over when the tour is: the whole
        # point is the approach that follows. Without this gate the tour's
        # final waypoint would latch mission_complete and the object would
        # never be looked up.
        if self.find_phase == "search":
            return
        # Nor is an open-world mission over while an object is still waiting to
        # be looked at -- or while the survey that finds them is still running.
        # "done" is set by begin_inspection_round, which is the only thing that
        # can distinguish "nothing left to inspect" from "nothing found yet".
        if self.inspect_phase in ("survey", "inspect"):
            return
        if self.find_phase == "approach":
            if not self.at_target_region():
                return
            self.find_phase = "done"
            self.get_logger().info(
                f"[OBJECT REACHED] {self.find_object_class} at {self.find_object_region}"
            )
            self.record_json_event("find_object_reached", {
                "object": self.find_object_class,
                "region": self.find_object_region,
                "evidence": self.find_object_evidence,
            })
        if self.regions_remaining() or not self.at_target_region():
            return
        self.mission_complete = True
        self.get_logger().info(
            f"[MISSION COMPLETE] all {len(set(self.mission_required_regions))} "
            f"required regions visited and parked at {self.target_region}"
        )
        self.record_json_event("mission_complete", {
            "required_regions": sorted(set(self.mission_required_regions)),
            "visited_regions": sorted(self.mission_visited_regions),
        })

    def truncate_to_executable_prefix(self, candidate):
        """Cut a plan at the first action that cannot fire, and return the rest.

        The mission plan deliberately runs PAST what is achievable: it surveys,
        then acts on the cone, then carries on. The cone action cannot fire
        while the object's position is unknown, so everything from there on was
        planned against a state that will never exist -- including the moves
        after it, which look perfectly drivable in isolation.

        That is why this is a truncation and not a filter.
        ``filter_executable_actions`` keeps every ``move`` wherever it sits, so
        on its own it would happily drive the tail and send the robot off on a
        second lap it was never meant to run. Cutting at the first inapplicable
        action is what makes "the valid portion of the plan" mean the prefix.

        Validated against the AUTHORED mission problem with ``(at ?r ?l)``
        rewritten to where the robot actually is, since ``candidate`` has
        already been aligned to the live pose and the authored start would
        otherwise reject its first move.

        Degrades to the full plan on any error. A missing mission file must not
        stop the robot leaving the start line; the worst case is the old
        behaviour.
        """
        if not candidate:
            return candidate
        problem_path = self.find_mission_problem()
        if problem_path is None:
            self.get_logger().warn(
                f"no problem file for mission '{self.mission_id}'; cannot trim the "
                "plan to its executable prefix, driving it as written")
            return candidate
        try:
            domain = parse_domain(self.domain_file)
            problem = parse_problem(problem_path)
            if self.current_region:
                problem.facts = {f for f in problem.facts
                                 if not (len(f) == 3 and f[0] == "at"
                                         and f[1] == self.robot_name)}
                problem.facts.add(("at", self.robot_name, self.current_region))
            result = validate_plan(domain, problem, candidate, require_goal=False)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"could not compute the executable prefix ({exc}); "
                "driving the plan as written")
            return candidate

        if result.accepted_prefix_len >= len(candidate):
            return candidate

        if result.accepted_prefix_len == 0:
            # An empty prefix means "the robot does nothing", which is never a
            # better answer than the plan as written -- and it is reachable
            # from a disagreement about the START rather than anything wrong
            # with the plan: alignment could not reconcile the live pose (say
            # the robot drifted into a region the mission graph does not
            # connect to the first leg), so the very first move is judged
            # inapplicable and the whole survey disappears. Nav2 can route
            # from where the robot actually is; a stationary robot cannot.
            self.get_logger().warn(
                f"executable prefix is empty ({result.errors[:1]}); the live pose "
                f"disagrees with the plan's start rather than the plan being "
                f"wrong -- driving it as written")
            self.record_json_event("plan_prefix_empty", {
                "errors": list(result.errors),
                "current_region": self.current_region,
                "plan": [a.text() for a in candidate],
            })
            return candidate

        prefix = candidate[:result.accepted_prefix_len]
        dropped = candidate[result.accepted_prefix_len:]
        self.get_logger().warn(
            f"[PLAN TRUNCATED] driving {len(prefix)}/{len(candidate)} actions; "
            f"stopped at {dropped[0].text()} -- "
            + "; ".join(result.errors[:2])
        )
        self.record_json_event("plan_truncated_to_prefix", {
            "kept": [a.text() for a in prefix],
            "dropped": [a.text() for a in dropped],
            "stopped_at": dropped[0].text(),
            "errors": list(result.errors),
            "problem_file": str(problem_path),
        })
        return prefix

    def locate_target_candidates(self):
        """Regions the target object may be in, best first, from observation
        memory. Empty when the evidence does not clear the thresholds.

        Returns the whole ranked list rather than just the winner because the
        approach records every candidate in its event -- when the robot drives
        to the wrong region, the runner-up is the first thing worth seeing.

        Shared by the mid-drive watch and the post-survey approach so both
        apply the same thresholds: a sighting good enough to rewrite the
        problem must be good enough to drive to, and vice versa.
        """
        records = load_observations(self.find_object_obs_log)
        if not records:
            return []
        found = locate_object(
            records, self.find_object_class,
            min_score=self.find_object_min_score,
            min_hits=self.find_object_min_hits,
            regions=self.regions,
        )
        if not found or found[0]["region"] not in self.regions:
            return []
        return found

    def object_watch_tick(self):
        """While the survey is being driven, watch YOLO for the target object.

        The mission problem asserts (location-unknown cone) and a goal naming
        the cone, which has no achiever -- what the robot is driving is that
        plan's executable PREFIX. This is the loop that ends the unknown: the
        one fact needed to make the problem solvable is the object's region,
        and perception produces it partway through the survey, not at the end.
        Reporting it as soon as it exists means the replan service has an
        updated problem on disk before the survey finishes.

        It does NOT interrupt the drive. The mission is "survey every region,
        THEN approach", and stopping at first sighting would skip regions that
        were never searched -- the approach is still triggered by plan
        exhaustion. What changes is that by then the answer is already known.

        Runs only in the search phase: once the approach is planned, the region
        is settled and a late detection in another region would retarget a
        robot already driving to the first one.
        """
        if self.find_phase != "search":
            return
        try:
            candidates = self.locate_target_candidates()
        except Exception as exc:  # noqa: BLE001 - a bad log line must not stop the drive
            self.get_logger().warn(f"object watch failed to read observations: {exc}")
            return
        best = candidates[0] if candidates else None
        if not best or best["region"] == self._reported_object_region:
            return

        first = self._reported_object_region is None
        self._reported_object_region = best["region"]
        self.find_object_region = best["region"]
        self.find_object_evidence = best
        self.get_logger().warn(
            f"[OBJECT SIGHTED] {self.find_object_class} in {best['region']} "
            f"({best['hits']} detections, mean score {best['mean_score']}, "
            f"at {best['xy']}) while surveying; "
            + ("reporting to the replan service" if first
               else "region CHANGED, re-reporting")
        )
        self.record_json_event("find_object_sighted", {
            "object": self.find_object_class,
            "region": best["region"],
            "evidence": best,
            "first_sighting": first,
            "plan_still_running": True,
        })
        if self.replan_client is not None:
            self.replan_client.report_observation({
                "tag": self.trial_tag,
                "mission_id": self.mission_id,
                "robot": self.robot_name,
                "class": self.find_object_class,
                "region": best["region"],
                "current_region": self.current_region,
                "visited_regions": sorted(self.mission_visited_regions),
                "evidence": best,
            })

    def begin_object_approach(self):
        """Look the target object up in observation memory and retarget to it.

        This is the search -> approach transition, and it runs when the tour's
        plan finishes rather than the moment the object is first seen: the
        mission the user asked for is "visit all regions, log what is where,
        THEN go to the object", and stopping the tour at first sighting would
        skip regions that were never searched.

        Returns True when a replan toward the object is in flight, meaning the
        caller must not declare the plan exhausted.
        """
        self.find_phase = "not_found"          # pessimistic until proven
        records = load_observations(self.find_object_obs_log)
        if not records:
            self.get_logger().warn(
                f"[OBJECT NOT FOUND] no observation records at "
                f"{self.find_object_obs_log!r}; cannot approach "
                f"{self.find_object_class}"
            )
            self.record_json_event("find_object_failed", {
                "object": self.find_object_class,
                "reason": "no observation records",
                "obs_log": self.find_object_obs_log,
            })
            return False

        # Re-read rather than trusting what object_watch_tick last saw: the
        # survey has finished since, so this is strictly more evidence.
        found = self.locate_target_candidates()
        best = found[0] if found else None
        if best is None:
            self.get_logger().warn(
                f"[OBJECT NOT FOUND] {self.find_object_class} was never seen "
                f"with >= {self.find_object_min_hits} detections above score "
                f"{self.find_object_min_score} in any known region; the tour "
                f"finished but there is nowhere to approach"
            )
            self.record_json_event("find_object_failed", {
                "object": self.find_object_class,
                "reason": "insufficient evidence",
                "min_hits": self.find_object_min_hits,
                "min_score": self.find_object_min_score,
                "sighted_during_survey": self._reported_object_region,
            })
            return False

        self.find_object_region = best["region"]
        self.find_object_evidence = best
        self.target_region = best["region"]
        self.find_phase = "approach"
        self.get_logger().warn(
            f"[OBJECT FOUND] {self.find_object_class} in {best['region']} "
            f"({best['hits']} detections, mean score {best['mean_score']}, "
            f"at {best['xy']}); retargeting mission there"
        )
        self.record_json_event("find_object_located", {
            "object": self.find_object_class,
            "region": best["region"],
            "evidence": best,
            "all_candidates": found,
        })

        if self.at_target_region():
            # The tour already ended standing on it. Nothing to drive.
            self.find_phase = "done"
            self.get_logger().info(
                f"[OBJECT REACHED] already at {best['region']}; no approach needed"
            )
            return False

        # force=True: the approach is the mission, not a contingency. If the
        # tour spent the replan budget on obstacles, refusing to plan the
        # approach would fail the mission for a reason unrelated to the object.
        self.escalate_symbolic_replan(
            self.active_plan_report, trigger="object_found", force=True
        )
        return self.symbolic.state != symbolic_replan.IDLE

    # ==================================================================
    # open-world find-and-inspect
    # ==================================================================
    def inspection_watch_tick(self):
        """Re-cluster perception into objects and fold them into the registry.

        The counterpart of ``object_watch_tick`` for a mission whose object set
        is unknown, and it differs in what it produces: not "which region holds
        the cone" but "here are the four things worth inspecting and where each
        one is". Runs on a timer through the whole mission, not just the survey
        -- an object first seen while driving to inspect another one is a real
        discovery, and the mission is not over until a poll adds nothing.

        Discoveries do NOT interrupt what the robot is doing. A replan is asked
        for only once the current plan has run out (``begin_inspection_round``),
        for the reason the survey exists at all: stopping at the first sighting
        would leave regions unsearched.
        """
        if self.inspect_phase in ("disabled", "done"):
            return
        try:
            records = load_observations(self.find_object_obs_log)
            clusters = cluster_detections(
                records, classes=self.inspect_object_classes,
                min_score=self.inspect_min_score, min_hits=self.inspect_min_hits,
                link_radius_m=self.inspect_cluster_radius_m, regions=self.regions)
        except Exception as exc:  # noqa: BLE001 - a bad log line must not stop the drive
            self.get_logger().warn(f"inspection watch failed to read observations: {exc}")
            return

        if len(self.object_registry) >= self.inspect_max_objects:
            clusters = []       # the registry is full; see inspect_max_objects
        discovered = self.object_registry.update(clusters, now=self.sim_time_now())
        if not discovered:
            return
        for target in discovered:
            self.get_logger().warn(
                f"[OBJECT DISCOVERED] {target.name} ({target.class_name}) in "
                f"{target.region} at {target.xy} -- {target.hits} detections, "
                f"mean score {target.mean_score}"
            )
        self.record_json_event("inspection_objects_discovered", {
            "new": [t.as_dict() for t in discovered],
            "registry": self.object_registry.summary(),
            "phase": self.inspect_phase,
        })
        # Tell the service, so the problem naming these objects exists on disk
        # from the moment of discovery rather than only inside the next replan.
        names = {t.name for t in self.object_registry.all()}
        if self.replan_client is not None and names != self._reported_object_names:
            self._reported_object_names = names
            self.replan_client.report_observation({
                "tag": self.trial_tag,
                "mission_id": self.mission_id,
                "robot": self.robot_name,
                "current_region": self.current_region,
                "visited_regions": sorted(self.mission_visited_regions),
                "inspect_objects": self.object_registry.instances(),
            })

    def begin_inspection_round(self):
        """Plan the next batch of inspections. True if a replan is in flight.

        Called when the plan runs out, which happens at least twice in a normal
        run: once when the survey ends, and once after each round of
        inspections in case the driving revealed more objects. Both are the same
        question -- is anything still uninspected? -- so they are the same code
        path rather than a survey-specific transition and an inspection-specific
        one.

        Returns False when there is nothing left to inspect, which is what ends
        the mission. With an unknown quantity of objects there is no certificate
        that all of them were found; "the last poll of perception discovered
        nothing new and everything known has been inspected" is the strongest
        available statement, and it is what this returns False on.
        """
        pending = self.object_registry.pending() if self.object_registry else []
        if not pending:
            if self.inspect_phase != "done":
                self.inspect_phase = "done"
                summary = self.object_registry.summary() if self.object_registry else {}
                done = self.object_registry.all() if self.object_registry else []
                self.get_logger().warn(
                    f"[INSPECTION COMPLETE] {summary.get('inspected', 0)} object(s) "
                    "inspected: " + ", ".join(f"{t.name}@{t.region}" for t in done)
                    if done else
                    "[INSPECTION COMPLETE] the survey finished and no object of "
                    f"{self.inspect_object_classes} was ever found to inspect"
                )
                self.record_json_event("inspection_complete", {"registry": summary})
                self.update_mission_completion()
            return False

        if not self.enable_symbolic_replan:
            # The inspections are planned, not scripted -- the objects did not
            # exist when the mission plan was written. Without Tier 2 there is
            # nothing to plan them, and the run would otherwise end quietly
            # with the survey done and every object untouched.
            # Its own flag, not the in-flight latch: this is "say it once",
            # and overloading the latch for it would be a second meaning for a
            # field whose single meaning is the point of the fix above.
            if not self._inspection_unplannable_logged:
                self._inspection_unplannable_logged = True
                self.get_logger().error(
                    f"[INSPECTION] {len(pending)} object(s) found but symbolic "
                    f"replan is DISABLED, so no plan can be produced for them. "
                    f"Run with SYMBOLIC_REPLAN=1."
                )
                self.record_json_event("inspection_unplannable", {
                    "reason": "symbolic replan disabled",
                    "pending": [t.as_dict() for t in pending],
                })
            return False
        if self._inspection_replan_pending:
            return False        # a round is already in flight
        now = self.sim_time_now()
        if (self._last_inspection_round_sim_s is not None
                and now - self._last_inspection_round_sim_s < self.inspect_round_retry_s):
            return False        # cooling down after a failed round
        if self._inspection_round_attempts >= self.inspect_max_rounds:
            if self.inspect_phase != "not_planned":
                self.inspect_phase = "not_planned"
                self.get_logger().error(
                    f"[INSPECTION] gave up after {self._inspection_round_attempts} "
                    f"rounds with {len(pending)} object(s) still uninspected: "
                    + ", ".join(f"{t.name}@{t.region}" for t in pending)
                )
                self.record_json_event("inspection_rounds_exhausted", {
                    "attempts": self._inspection_round_attempts,
                    "pending": [t.as_dict() for t in pending],
                })
            return False
        self.inspect_phase = "inspect"
        self._inspection_replan_pending = True
        self._last_inspection_round_sim_s = now
        self._inspection_round_attempts += 1
        self.get_logger().warn(
            f"[INSPECTION ROUND {self._inspection_round_attempts}/"
            f"{self.inspect_max_rounds}] {len(pending)} object(s) to inspect: "
            + ", ".join(f"{t.name}@{t.region}" for t in pending)
        )
        self.record_json_event("inspection_round_started", {
            "pending": [t.as_dict() for t in pending],
            "registry": self.object_registry.summary(),
        })
        # Point the mission at the first object waiting. The planner is free to
        # take them in another order -- apply_symbolic_replan reads the real
        # endpoint back off the plan it returns -- but leaving target_region on
        # the survey's endpoint would leave the STL goal check, and anything
        # else that asks where the robot is headed, describing a leg of the
        # mission that is over.
        self.target_region = pending[0].region
        # force=True for the same reason the object approach uses it: the
        # inspections ARE the mission, so refusing to plan them because the
        # survey spent the replan budget on obstacles would fail the run for an
        # unrelated reason.
        self.escalate_symbolic_replan(
            self.active_plan_report, trigger="inspection_round", force=True)
        return self.symbolic.state != symbolic_replan.IDLE

    def inspection_retry_possible(self):
        """True when another inspection round is still owed to the mission.

        Distinguishes "waiting to try again" from "finished". The exhaustion
        check treats the two identically otherwise, and would end the run during
        a retry cooldown -- which is how a single timed-out replan came to
        finish a mission with every discovered object still uninspected.
        """
        if self.inspect_phase not in ("survey", "inspect"):
            return False
        if not self.enable_symbolic_replan or self.object_registry is None:
            return False
        if not self.object_registry.pending():
            return False
        return self._inspection_round_attempts < self.inspect_max_rounds

    def at_target_region(self):
        """True when the robot is parked at the mission target.

        Deliberately geometric rather than "Nav2 reported the goal finished":
        after a hot-swap the goal-finished history belongs to a plan that no
        longer exists, whereas position is always current.
        """
        if self.current_xy is None:
            return False
        target = self.regions.get(self.target_region)
        if target is None:
            return False
        return math.hypot(self.current_xy[0] - target[0],
                          self.current_xy[1] - target[1]) <= self.goal_arrival_radius_m

    def region_of_waypoint(self, waypoint_idx):
        """Graph region of the waypoint the robot was heading for."""
        if waypoint_idx is None or not self.active_waypoints:
            return None
        idx = max(0, min(int(waypoint_idx), len(self.active_waypoints) - 1))
        region, _ = self.nearest_region(self.active_waypoints[idx])
        return region

    def current_plan_leg(self):
        """``(from_region, to_region)`` of the step being executed, for prose.

        Indexed against the ROUTE, not against a filtered list of moves: an
        inspection is a step of the route too, and counting only moves would
        report the wrong leg for the rest of the plan once one is passed. An
        inspection has no from-region -- its second argument is the object --
        so that half comes back None.
        """
        route = self.route_actions or []
        idx = (self._last_feedback_waypoint or 0) + self._waypoints_offset
        if not route or idx >= len(route) or route[idx] is None:
            return None, None
        action = route[idx]
        args = action.args
        if not args:
            return None, None
        from_region = (args[-2] if len(args) >= 2 and action.name.startswith("move")
                       else None)
        return from_region, args[-1]

    def escalate_symbolic_replan(
        self,
        plan_report,
        trigger,
        monitor_result=None,
        failed_region=None,
        force=False,
    ):
        """Hold the robot and ask the host service for a new symbolic plan.

        Returns ``plan_report`` unchanged in every path: the new plan arrives
        asynchronously and is applied later by :meth:`symbolic_replan_tick`, so
        callers must not expect a swapped plan on return.
        """
        if not self.enable_symbolic_replan:
            return plan_report

        # Once the robot is parked at the mission target there is nothing left
        # to replan. Without this, a pedestrian wandering past the stationary
        # robot trips ped_approach_rate and the replanner dutifully produces a
        # route OUT of the goal region and back -- observed as
        # (move r8 r6) (move r6 r8), 198s after arrival, inflating both
        # path_length_m and duration_s for a mission that had already succeeded.
        #
        # `force` overrides it, and must: an inspection round is escalated
        # exactly when the survey has finished and the robot is parked at its
        # target, so this guard would refuse the one replan the mission depends
        # on. The reasoning above is about REACTIVE triggers -- a pedestrian
        # near a robot that has already arrived -- and those never force.
        if self.at_target_region() and not force:
            self.get_logger().info(
                f"Symbolic replan not escalated: already at target {self.target_region}"
            )
            return plan_report

        now_sim = self.sim_time_now()
        allowed, why_not = self.symbolic.can_escalate(
            now_sim,
            self.max_symbolic_replans,
            self.symbolic_replan_cooldown_s,
            self.mission_deliberation_budget_s,
        )
        # force bypasses the budget but never the state machine: a second
        # concurrent request would race the first one's plan swap.
        if force and not allowed and self.symbolic.state == symbolic_replan.IDLE:
            self.get_logger().warn(
                f"Overriding replan budget for {trigger} ({why_not})"
            )
            allowed, why_not = True, ""
        if not allowed:
            self.get_logger().info(f"Symbolic replan not escalated: {why_not}")
            # Log the give-up ONCE per reason. This runs on every monitor tick
            # (~7 Hz via the track and nav2-plan callbacks), and record_json_event
            # rewrites the whole JSON log on each call -- unguarded it produced
            # hundreds of identical events and a multi-megabyte log per run.
            if "budget exhausted" in why_not and why_not not in self._giveup_reasons_logged:
                self._giveup_reasons_logged.add(why_not)
                self.record_json_event("symbolic_replan_giveup", {"reason": why_not})
            return plan_report

        obstacle_region = None
        if monitor_result is not None and monitor_result.closest_obstacle_xy is not None:
            obstacle_region, _ = self.nearest_region(monitor_result.closest_obstacle_xy)

        blocked = derive_blocked_regions(
            trigger,
            current_region=self.current_region,
            target_region=self.target_region,
            obstacle_region=obstacle_region,
            failed_region=failed_region,
        )
        self.symbolic.blocked_regions.update(blocked)

        from_region, to_region = self.current_plan_leg()
        detail = {
            "from_region": from_region,
            "to_region": to_region,
            "blocked_regions": sorted(self.symbolic.blocked_regions),
            "reactive_attempts": self.nav2_replan_count,
            "stuck_window_s": self.stuck_window_s,
        }
        if monitor_result is not None:
            detail["closest_obstacle"] = monitor_result.closest_obstacle
        if trigger == "stl_shield" and self.last_shield_result is not None:
            detail["violated_conjunct"] = self.last_shield_result.worst_conjunct
            detail["robustness"] = self.last_shield_result.robustness
        if trigger in symbolic_replan.DISCOVERY_TRIGGERS and self.object_registry:
            detail["found_objects"] = len(self.object_registry)

        self.symbolic.count += 1
        self.symbolic.last_escalation_sim_s = now_sim
        self.record_json_event(
            "symbolic_replan_escalated",
            {
                "trigger": trigger,
                "attempt": self.symbolic.count,
                "max": self.max_symbolic_replans,
                "blocked_regions": sorted(self.symbolic.blocked_regions),
                "detail": {k: _finite_or_none(v) if isinstance(v, float) else v
                           for k, v in detail.items()},
            },
        )

        self.enter_hold(trigger)

        payload = self.build_replan_request(trigger, detail)
        self._replan_started_wall = time.monotonic()
        if not self.replan_client.request_async(payload, self.symbolic_replan_deadline_s):
            self.get_logger().error("Replan client was busy; aborting escalation")
            self.record_json_event(
                "symbolic_replan_service_error", {"error": "client busy"}
            )
            self.abandon_replan("client busy")
            return plan_report

        self.record_json_event(
            "symbolic_replan_requested",
            {
                "url": self.replan_service_url,
                "planner_mode": self.planner_mode,
                "deadline_s": self.symbolic_replan_deadline_s,
                "trigger": trigger,
                "reason_text": payload["reason"]["human_text"],
            },
        )
        self.get_logger().warn(
            f"Symbolic replan {self.symbolic.count}/{self.max_symbolic_replans} "
            f"requested ({trigger}); holding"
        )
        return plan_report

    def build_replan_request(self, trigger, detail):
        """Assemble the JSON request for the host service."""
        executed_idx = (self._last_feedback_waypoint or 0) + self._waypoints_offset
        # The route's own actions, in route order, so the split is between what
        # the robot HAS driven and what it has not. Indexing a filtered list of
        # moves was equivalent only while moves were the sole lowered action;
        # an interleaved inspect-object shifts every index past it.
        route = [a for a in (self.route_actions or []) if a is not None]
        if not route:
            route = [a for a in (self.active_plan_actions or [])
                     if a.name.startswith("move")]
        executed = [a.text() for a in route[:executed_idx]]
        remaining = [a.text() for a in route[executed_idx:]]
        if trigger == "inspection_round" and not remaining:
            # `remaining_plan` is the seed EvoPlan repairs, and an inspection
            # round is escalated precisely when the plan has been driven to the
            # end -- so the seed would be empty and the LLM would be
            # synthesising from scratch rather than repairing. In evoplan_only
            # there is not even an FD plan to fall back on.
            #
            # So seed it with the work the mission is asking for: one
            # inspect-object per object still pending. This is deliberately NOT
            # a valid plan -- it has no moves, so every action's (at ?r ?l) is
            # unmet -- and that is the mechanism, not a defect. VAL replies
            # "unsatisfied precondition (at jackal_1 r1)" for each one, EvoPlan
            # surfaces that as the artifact the next iteration reads, and
            # inserting the moves between them is exactly the repair this
            # pipeline is good at.
            remaining = [
                f"(inspect-object {self.robot_name} {target.name} {target.region})"
                for target in (self.object_registry.pending()
                               if self.object_registry else [])
            ]
        return {
            "mission_id": self.mission_id,
            # Which trial this replan belongs to; the service names its
            # archived problem/plan artifacts after it.
            "tag": self.trial_tag,
            "robot": self.robot_name,
            "current_region": self.current_region,
            "current_xy": list(self.current_xy) if self.current_xy else None,
            "goal": {"target_region": self.target_region},
            # The tour problem's goal is all-(visited), which the service keeps
            # verbatim because every conjunct is executable. For the approach
            # that is wrong: the tour is already done and the only thing left
            # is to reach the object's region, so narrow the goal explicitly.
            # None lets the service decide from the goal, as before.
            "preserve_goal": False if self.find_phase == "approach" else None,
            # What perception found, in the service's terms. When the mission
            # problem declares a `- target`, this is what turns the narrowed
            # goal into (reached <robot> <object>) with (object-at ...) as a
            # fact, so the PLANNER derives the region instead of being handed
            # it. `class` is the detector string ("traffic cone"); the service
            # maps it onto the declared PDDL symbol, because the two spellings
            # need not -- and cannot always -- match.
            "found_object": (
                {"class": self.find_object_class, "region": self.find_object_region}
                if self.find_phase == "approach" and self.find_object_region
                else None
            ),
            # Every object this mission knows about, inspected or not, with the
            # PDDL name the executor minted for it. The service declares each as
            # a `- target`, places it with (object-at ...) and adds
            # (inspected-object ...) to the goal -- which is how a problem that
            # declared no objects at all comes to state an open-world mission.
            # Already-inspected objects stay in the list: dropping one would
            # delete the fact that explains the goal conjunct it satisfies.
            "inspect_objects": (self.object_registry.instances()
                                if self.object_registry else None),
            "executed_plan": executed,
            "remaining_plan": remaining,
            "blocked_regions": sorted(self.symbolic.blocked_regions),
            "planner_mode": self.planner_mode,
            "deadline_s": self.symbolic_replan_deadline_s,
            "reason": {
                "trigger": trigger,
                "robustness": _finite_or_none(detail.get("robustness")),
                "violated_conjunct": detail.get("violated_conjunct"),
                "closest_obstacle": detail.get("closest_obstacle"),
                "failed_region": detail.get("to_region"),
                "reactive_attempts": detail.get("reactive_attempts"),
                # The prose the LLM actually reads, via OpenEvolve's artifacts
                # feedback channel. This is what makes the replan informed
                # rather than just re-run.
                "human_text": build_reason_text(trigger, detail),
            },
        }

    # ------------------------------------------------------------------
    # holding
    # ------------------------------------------------------------------
    def enter_hold(self, trigger):
        """Stop the robot in place while the planner deliberates.

        The Gazebo clock is deliberately left running: sim time keeps advancing,
        pedestrians keep walking, and the deliberation shows up honestly in
        ``duration_s``. Station-keeping is also the socially correct response to
        a shield violation -- the trigger there is that someone is too close.
        """
        self.symbolic.state = symbolic_replan.HOLDING
        # An inspection interrupted by deliberation does not count. Cancel it
        # before the hold takes over the base, so the two are not both
        # publishing cmd_vel.
        self.cancel_inspect_dwell(f"held for a symbolic replan ({trigger})")
        self.symbolic.resume_idx = self._last_feedback_waypoint or 0
        self.symbolic.hold_started_sim_s = self.sim_time_now()
        self.record_json_event(
            "robot_hold_started",
            {"trigger": trigger, "resume_idx": self.symbolic.resume_idx},
        )
        if not self.hold_on_replan:
            return
        self.cancel_active_nav2_goal()
        if self.hold_timer is None:
            self.hold_timer = self.create_timer(0.1, self.publish_hold_stop)

    def publish_hold_stop(self):
        """Zero-velocity heartbeat while held.

        Nav2's controller is cancelled so nothing else is commanding the base;
        this kills residual drift and makes "deliberately stopped" legible in a
        bag rather than looking like a hang.
        """
        if self.symbolic.state != symbolic_replan.HOLDING:
            return
        self.cmd_vel_pub.publish(Twist())

    def exit_hold(self):
        held_s = 0.0
        if self.symbolic.hold_started_sim_s is not None:
            held_s = self.sim_time_now() - self.symbolic.hold_started_sim_s
            self.symbolic.held_sim_s += held_s
        self.symbolic.hold_started_sim_s = None
        if self.hold_timer is not None:
            self.hold_timer.cancel()
            self.hold_timer = None
        self.record_json_event("robot_hold_ended", {"held_s": held_s})
        return held_s

    # ------------------------------------------------------------------
    # result handling
    # ------------------------------------------------------------------
    def symbolic_replan_tick(self):
        """Drain the replan worker's queue and apply results.

        Runs on the executor thread, so it is the only place plan bookkeeping
        is mutated.
        """
        if self.get_parameter("force_symbolic_replan").value:
            self.set_parameters(
                [rclpy.parameter.Parameter("force_symbolic_replan",
                                           rclpy.Parameter.Type.BOOL, False)]
            )
            self.get_logger().warn("force_symbolic_replan set; escalating on demand")
            self.escalate_symbolic_replan(self.active_plan_report, trigger="forced")

        result = self.replan_client.poll()
        if result is None:
            return

        if self._replan_started_wall is not None:
            self.symbolic.deliberation_wall_s += time.monotonic() - self._replan_started_wall
            self._replan_started_wall = None

        self.record_json_event(
            "symbolic_replan_received",
            {
                "status": result.get("status"),
                "planner": result.get("planner"),
                "valid": result.get("valid"),
                "elapsed_s": result.get("elapsed_s"),
                "plan": result.get("plan", []),
            },
        )

        if result.get("status") == "timeout" and not result.usable:
            self.record_json_event(
                "symbolic_replan_timeout",
                {"deadline_s": self.symbolic_replan_deadline_s},
            )
            self.abandon_replan("deadline expired with no valid plan")
            return
        if not result.usable:
            self.abandon_replan(
                f"service returned no usable plan ({result.get('status')}: "
                f"{result.get('error')})"
            )
            return

        self.symbolic.state = symbolic_replan.APPLYING
        if not self.apply_symbolic_replan(result["plan"]):
            self.abandon_replan("returned plan was rejected locally")

    def abandon_replan(self, reason):
        """Give up on this replan and resume the previous route.

        The robot must never be left planless: whatever went wrong upstream,
        the old waypoint suffix is still a route it was already following.
        """
        self.get_logger().warn(f"Symbolic replan abandoned: {reason}")
        self.exit_hold()
        self.symbolic.state = symbolic_replan.IDLE
        # Release the inspection round with it. This flag means "a round is in
        # flight", and after an abandon none is -- but it used to be cleared
        # ONLY on success, so a single failed round ended the inspections for
        # the rest of the mission. A live run discovered both objects, lost one
        # replan to a timeout, and finished having inspected neither, with its
        # replan budget still unspent and nothing left that would ask again.
        # The cooldown in begin_inspection_round is what now stops the retry
        # becoming a per-tick request.
        self._inspection_replan_pending = False
        if self.active_waypoints:
            self.resend_waypoint_suffix(self.symbolic.resume_idx)

    def apply_symbolic_replan(self, plan_lines):
        """Validate, lower and hot-swap a new symbolic plan. True if applied."""
        actions = self.parse_plan_text("\n".join(plan_lines))
        if not actions:
            self.record_json_event("symbolic_replan_rejected", {"errors": ["empty plan"]})
            return False

        executable, dropped = filter_executable_actions(actions)
        for action in dropped:
            self.record_json_event(
                "plan_action_unexecutable",
                {"action": action.text(), "reason": "no executor for this action type"},
            )
        if self.inspect_phase == "inspect":
            # An inspection plan has no target region agreed in advance: it ends
            # wherever the last object it inspects happens to be, and which
            # objects it takes in what order is the planner's decision. So the
            # endpoint is READ OFF the plan rather than checked against a
            # constant -- and read off it here, before anything downstream
            # (mission completion, the STL goal check) asks where the robot is
            # supposed to end up.
            end_region = symbolic_replan.plan_end_region(executable)
            if end_region is None or end_region not in self.regions:
                self.record_json_event(
                    "symbolic_replan_rejected",
                    {"errors": [f"inspection plan ends at unknown region {end_region!r}"]},
                )
                return False
            if end_region != self.target_region:
                self.get_logger().info(
                    f"[INSPECTION] plan ends at {end_region}; retargeting from "
                    f"{self.target_region}"
                )
                self.target_region = end_region
        elif not plan_reaches_target(executable, self.target_region):
            self.record_json_event(
                "symbolic_replan_rejected",
                {"errors": [f"move chain does not reach {self.target_region}"]},
            )
            return False

        # Align BEFORE validating. The service plans from the region the robot
        # was in when it escalated, but an EvoPlan replan takes 60-100s and the
        # robot's snapped region can differ by the time the plan lands -- that
        # is exactly what align_plan_start_with_current_region exists to
        # reconcile. Validating the raw plan first rejected VAL-valid plans on
        # "missing preconditions [('at', 'jackal_1', 'r5')]" before alignment
        # could fix them, which killed both replans of the first evoplan trial.
        aligned = self.align_plan_start_with_current_region(executable)

        # Second gate, in-container and dependency-free, on whatever the LLM
        # produced. require_goal is False because the service may legitimately
        # return a plan for a sub-goal.
        try:
            # parse_domain takes a PATH and reads it itself (pipeline.py:146).
            # Passing text raised OSError("File name too long"), which the
            # except below swallowed as "validation errored, accepting plan" --
            # so this gate silently passed every replanned plan.
            domain = parse_domain(self.domain_file)
            validation = validate_plan(domain, self.make_problem_from_graph(), aligned,
                                       require_goal=False)
            if not validation.ok:
                self.record_json_event(
                    "symbolic_replan_rejected",
                    {"errors": list(validation.errors),
                     "aligned_plan": [a.text() for a in aligned],
                     "current_region": self.current_region},
                )
                self.get_logger().warn(
                    f"Returned plan failed local validation after alignment "
                    f"(robot at {self.current_region}): {validation.errors}"
                )
                return False
        except Exception as exc:  # noqa: BLE001 - never crash the executor on a bad plan
            self.get_logger().warn(f"Local validation errored, accepting plan: {exc}")
        waypoints, tasks = self.plan_to_nav2_goals(aligned)
        if not waypoints:
            self.record_json_event(
                "symbolic_replan_rejected", {"errors": ["plan lowered to zero waypoints"]}
            )
            return False

        # --- bookkeeping reset ---------------------------------------
        # This is a WHOLE NEW plan, not a suffix of the old one, so every index
        # into the old plan must be dropped. feedback_callback computes
        # `global_idx = completed + self._waypoints_offset` and looks it up in
        # active_plan_actions; a stale offset here is bounds-guarded so it will
        # not crash, but it silently mislabels or skips the [PLAN STEP DONE]
        # lines, which is worse -- it looks like it is working.
        self.active_plan_actions = aligned
        self.active_plan_report = {
            "ok": True,
            "plan": [a.text() for a in aligned],
            "plan_actions": aligned,
            "source": "symbolic_replan",
        }
        self.set_route(waypoints, tasks)
        self._waypoints_offset = 0
        self._last_feedback_waypoint = None
        # A dwell belongs to the route that scheduled it. The object it was
        # aimed at stays uninspected and the new plan is free to include it
        # again -- which it will, since the goal conjunct is still open.
        self.cancel_inspect_dwell("a new symbolic plan replaced the route")
        self.last_monitor_signature = None
        self.last_nav2_path = None
        # A new symbolic plan earns a fresh reactive budget; max_symbolic_replans
        # is what bounds the overall loop.
        self.nav2_replan_count = 0
        # The inspection round asked for got planned. Clearing the latch here
        # rather than at request time is what lets the NEXT round be asked for
        # while stopping a failed one from being re-asked on every tick.
        self._inspection_replan_pending = False
        self.clear_costmap_edit_if_active("applying a new symbolic plan")
        self.shield.reset()
        self.shield_violation_active = False
        self.shield_violation_since = None
        self._stuck_anchor = None

        held_s = self.exit_hold()
        self.publish_waypoints_path(waypoints)
        self.send_waypoints(waypoints)
        self.symbolic.state = symbolic_replan.IDLE
        self.record_json_event(
            "symbolic_replan_applied",
            {
                "plan": [a.text() for a in aligned],
                "nav2_goals": [{"x": x, "y": y} for x, y in waypoints],
                "dropped_actions": [a.text() for a in dropped],
                "held_s": held_s,
                "waypoints_offset_reset": True,
            },
        )
        self.get_logger().warn(
            f"Symbolic replan applied after {held_s:.1f} s hold: "
            + " -> ".join(a.text() for a in aligned)
        )
        return True

    def resend_waypoint_suffix(self, start_idx):
        """Re-send the tail of the current route from ``start_idx``.

        Shared by the Tier-1 reactive replan and the Tier-2 abandon path so the
        two cannot drift on the ``_waypoints_offset`` arithmetic.
        """
        start = max(0, int(start_idx or 0))
        if start >= len(self.active_waypoints or []):
            start = 0                      # keep the whole route, as before
        remaining = self.slice_route(start)
        self._waypoints_offset += start
        self._last_feedback_waypoint = None
        self.publish_waypoints_path(remaining)
        self.send_waypoints(remaining)
        return remaining

    def advance_route(self):
        """Drive the next segment after one finished. Ends the plan if none.

        The dwell's continuation, and the only other place a route advances
        without a replan. Bookkeeping matches ``resend_waypoint_suffix`` exactly
        -- consume from the front, add to ``_waypoints_offset`` -- so the two
        cannot disagree about how far through the original route the robot is.
        """
        consumed = max(1, int(self._segment_len or 0))
        remaining = self.slice_route(consumed)
        self._waypoints_offset += consumed
        self._last_feedback_waypoint = None
        if not remaining:
            # Nothing left to drive. Whether that is SUCCESS is decided by
            # plan_exhaustion_tick, as for any completed route.
            self._segment_len = 0
            self._plan_finished_sim_s = self.sim_time_now()
            return None
        self.publish_waypoints_path(remaining)
        self.send_waypoints(remaining)
        return remaining

    # ------------------------------------------------------------------
    # inspection dwell
    # ------------------------------------------------------------------
    def begin_inspect_dwell(self, task):
        """Hold still, facing the object, for ``inspect_dwell_s``.

        This is the physical content of `inspect-object`: without it the action
        would be the same zero-cost bookkeeping `approach` is, and a mission
        that claims to have inspected six objects would have done nothing but
        drive past them.

        Time is measured in SIM seconds, like every other duration in this node.
        Five wall seconds and five sim seconds are routinely different amounts
        of Gazebo -- a crowded world with four YOLO instances runs well under
        1.0x -- and the dwell has to be five seconds of the world the robot and
        the camera are in.
        """
        self.cancel_inspect_dwell("superseded by a new inspection")
        self._dwell = {
            "task": task,
            "started_sim_s": self.sim_time_now(),
            "hold_started_sim_s": None,
            "face_warned": False,
        }
        self.get_logger().info(
            f"[INSPECTING] {task.get('object')} at {task.get('region')}: "
            f"holding {self.inspect_dwell_s:.1f} s facing "
            + (f"{task['object_xy']}" if task.get("object_xy") else "no known position")
        )
        self.record_json_event("inspect_started", {
            "object": task.get("object"),
            "class": task.get("class"),
            "region": task.get("region"),
            "object_xy": list(task["object_xy"]) if task.get("object_xy") else None,
            "dwell_s": self.inspect_dwell_s,
        })
        # Created once and reset thereafter, never destroyed: the tick that
        # finishes a dwell stops its own timer, and destroying a timer from
        # inside its own callback is not something to rely on.
        if self._dwell_timer is None:
            self._dwell_timer = self.create_timer(0.1, self.inspect_dwell_tick)
        else:
            self._dwell_timer.reset()

    def inspect_dwell_tick(self):
        """Turn to face the object, then hold still until the dwell is served."""
        if self._dwell is None:
            return
        task = self._dwell["task"]
        now = self.sim_time_now()
        if not self.face_object(task, now):
            return          # still rotating; the hold has not started
        if self._dwell["hold_started_sim_s"] is None:
            self._dwell["hold_started_sim_s"] = now
        # Keep commanding zero. Nav2 has finished with this goal and is not
        # driving, but a stale controller command or a nudge from the sim would
        # otherwise creep the robot through its own inspection. Published
        # directly rather than through publish_hold_stop, which is gated on the
        # replan hold state and would do nothing here.
        self.cmd_vel_pub.publish(Twist())
        if now - self._dwell["hold_started_sim_s"] < self.inspect_dwell_s:
            return
        self.finish_inspect_dwell()

    def face_object(self, task, now):
        """Rotate toward the object. True once aimed (or unable to aim).

        Recomputed from the LIVE pose rather than reusing the yaw baked into
        the goal pose: Nav2 stops within its goal tolerance, not on it, and a
        heading computed for the region centroid is wrong by however far short
        the robot came to rest. Half a metre of position error at three metres'
        range is nearly ten degrees of pointing error.

        Returns True when there is nothing to aim at -- no known object
        position, or no yaw estimate -- so a degraded inspection still dwells
        rather than blocking forever on a correction it cannot compute.
        """
        object_xy = task.get("object_xy")
        if object_xy is None or self.current_yaw is None or self.current_xy is None:
            return True
        desired = math.atan2(object_xy[1] - self.current_xy[1],
                             object_xy[0] - self.current_xy[0])
        error = yaw_error(desired, self.current_yaw)
        if abs(error) <= self.inspect_face_tolerance_rad:
            return True
        if now - self._dwell["started_sim_s"] > self.inspect_face_timeout_s:
            if not self._dwell["face_warned"]:
                self._dwell["face_warned"] = True
                self.get_logger().warn(
                    f"[INSPECT] gave up turning toward {task.get('object')} after "
                    f"{self.inspect_face_timeout_s:.1f} s ({math.degrees(error):.0f} deg "
                    f"off); dwelling anyway"
                )
                self.record_json_event("inspect_face_timeout", {
                    "object": task.get("object"),
                    "yaw_error_deg": round(math.degrees(error), 1),
                })
            return True
        twist = Twist()
        # Proportional, clamped, and slow: this is a final alignment of a
        # stationary robot, not a manoeuvre.
        twist.angular.z = max(-self.inspect_face_speed,
                              min(self.inspect_face_speed, 1.5 * error))
        self.cmd_vel_pub.publish(twist)
        return False

    def finish_inspect_dwell(self):
        """Record the inspection and drive on."""
        dwell = self._dwell
        if dwell is None:
            return
        task = dwell["task"]
        held_s = self.sim_time_now() - (dwell["hold_started_sim_s"] or dwell["started_sim_s"])
        self.cancel_inspect_dwell(None)
        name = task.get("object")
        if self.object_registry is not None and name:
            self.object_registry.mark_inspected(name, at=self.sim_time_now())
        self.get_logger().warn(
            f"[INSPECTED] {name} ({task.get('class') or 'unknown class'}) at "
            f"{task.get('region')} after {held_s:.1f} s; "
            f"{len(self.object_registry.pending()) if self.object_registry else 0} left"
        )
        self.record_json_event("inspect_completed", {
            "object": name,
            "class": task.get("class"),
            "region": task.get("region"),
            "held_s": round(held_s, 2),
            "registry": self.object_registry.summary() if self.object_registry else None,
        })
        self.advance_route()

    def cancel_inspect_dwell(self, reason):
        """Abandon a dwell in progress. Safe to call when there is none.

        Called whenever the route the dwell belongs to stops being the route:
        a symbolic replan, a hold, a Tier-1 reroute. The object stays
        UNINSPECTED, so the next plan will come back for it -- an interrupted
        stare is not an inspection.
        """
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
        if self._dwell is None:
            return
        if reason:
            task = self._dwell["task"]
            self.get_logger().warn(
                f"[INSPECT ABANDONED] {task.get('object')} at {task.get('region')}: {reason}"
            )
            self.record_json_event("inspect_abandoned", {
                "object": task.get("object"), "reason": reason})
        self._dwell = None

    # ------------------------------------------------------------------
    # stuck detection
    # ------------------------------------------------------------------
    def stuck_check_tick(self):
        """Escalate when Nav2 is grinding without aborting.

        Nav2 frequently does not abort a goal it cannot achieve -- it just keeps
        trying. Without this, the only failure signal would be the mission
        timing out.
        """
        if self.symbolic.state != symbolic_replan.IDLE:
            return
        if self._dwell is not None:
            # Standing still is the job right now. Without this the dwell and
            # the turn-to-face count as zero displacement toward the stuck
            # window and a long enough inspection would escalate a replan
            # against a robot doing exactly what it was told.
            self._stuck_anchor = None
            return
        if self.active_goal_handle is None or self.current_xy is None:
            self._stuck_anchor = None
            return

        now_sim = self.sim_time_now()
        if self._stuck_anchor is None:
            self._stuck_anchor = (self.current_xy[0], self.current_xy[1], now_sim,
                                  self._last_feedback_waypoint)
            return

        ax, ay, at, awp = self._stuck_anchor
        if self._last_feedback_waypoint != awp:
            self._stuck_anchor = None  # progress: a waypoint completed
            return
        if now_sim - at < self.stuck_window_s:
            return

        moved = math.hypot(self.current_xy[0] - ax, self.current_xy[1] - ay)
        if moved >= self.stuck_min_displacement_m:
            self._stuck_anchor = None
            return

        failed_region = self.region_of_waypoint(self._last_feedback_waypoint)
        self.get_logger().warn(
            f"Stuck: moved {moved:.2f} m in {now_sim - at:.0f} s heading for "
            f"{failed_region or 'an unknown region'}"
        )
        self.record_json_event(
            "nav2_stuck_detected",
            {
                "window_s": now_sim - at,
                "displacement_m": moved,
                "waypoint_idx": self._last_feedback_waypoint,
                "region": failed_region,
            },
        )
        self._stuck_anchor = None
        self.escalate_symbolic_replan(
            self.active_plan_report, trigger="nav2_stuck", failed_region=failed_region
        )

    def settle_if_nothing_running(self, reason):
        """Hand control to ``plan_exhaustion_tick`` when nothing else can act.

        An aborted Nav2 goal leaves the robot with no active goal. Normally a
        replan follows and supplies a new route; when one CANNOT be escalated --
        the deliberation budget is spent, the cooldown has not elapsed, Tier 2
        is off -- nothing follows. ``_plan_finished_sim_s`` is only ever set by
        a goal that SUCCEEDED, so the exhaustion path that decides whether the
        run is over never runs either, and the executor sits with no goal, no
        replan and no verdict until the mission timeout fires. Observed: a
        survey aborted on its final leg, then 40 minutes of STL monitor ticks
        against a stationary robot.

        Marking the plan finished is the honest description of the situation --
        there is no route left and nothing is going to produce one -- and it
        routes the decision to the code written to make it. For a
        find-and-inspect mission that is also a rescue: the inspection round is
        reached from there and escalates with ``force=True``, so the objects
        already discovered still get inspected even though the survey ended
        badly.

        Deliberately narrow. Every guard here names something that IS still
        going to act, and in each of those cases this must do nothing.
        """
        if self.active_goal_handle is not None:
            return                      # still driving
        if self.replan_in_progress or self.symbolic.state != symbolic_replan.IDLE:
            return                      # a replan is in flight or holding
        if self._dwell is not None:
            return                      # an inspection is being served
        if self._plan_finished_sim_s is not None:
            return                      # already settled
        self.get_logger().warn(
            f"[NO ROUTE] {reason}; nothing is left to drive. Letting the "
            f"exhaustion check decide whether the mission is over."
        )
        self.record_json_event("route_settled_without_replan", {
            "reason": reason,
            "symbolic_replans": self.symbolic.count,
            "deliberation_s": round(self.symbolic.deliberation_wall_s, 1),
            "regions_remaining": self.regions_remaining(),
            "inspect_phase": self.inspect_phase,
            "inspection_pending": [t.name for t in self.object_registry.pending()]
                                  if self.object_registry else [],
        })
        self._plan_finished_sim_s = self.sim_time_now()

    def plan_exhaustion_tick(self):
        """Declare the run over once there is nothing left to drive.

        The executor can finish its plan without finishing the MISSION -- a
        replan narrowed to ``(visited <target>)`` drops the rest of the tour, so
        mission_complete stays False forever while the robot sits at the goal.
        Nothing else resolves that: scand_metrics gates its early stop on
        mission_complete, so the trial idled to its timeout (observed: 11
        minutes parked 0.21 m from the target).

        Terminal means: plan finished, no replan in flight or possible, and the
        grace period elapsed. Whether it is a SUCCESS is left to
        mission_complete -- this only stops the hang.
        """
        if self.plan_exhausted or self._plan_finished_sim_s is None:
            return
        if self.symbolic.state != symbolic_replan.IDLE or self.replan_in_progress:
            return  # a replan may yet extend the mission
        if self.active_goal_handle is not None:
            return  # still driving
        # The tour is done -- now go find the object. This must run before the
        # grace check, or a run with end_on_replans_exhausted would terminate
        # the mission at exactly the moment the approach becomes possible.
        if self.find_phase == "search":
            if self.begin_object_approach():
                return          # approach plan in flight; not exhausted
            if self.find_phase == "done":
                self.update_mission_completion()

        # Same shape for the open-world mission, and the same reason for
        # running before the grace check: the survey ending is what makes the
        # inspections plannable, and terminating the run at that moment would
        # end it one step before the work.
        if self.inspect_phase in ("survey", "inspect"):
            if self.begin_inspection_round():
                return          # inspection plan in flight; not exhausted
            if self.inspection_retry_possible():
                # A round failed and another is due once the cooldown elapses.
                # Returning here is what keeps the run alive to make it: without
                # it the grace period below would expire during the cooldown and
                # mark the plan exhausted, ending the mission between two
                # attempts rather than after the last one.
                return

        grace = self.plan_exhaustion_grace_s
        if self.end_on_replans_exhausted and self.symbolic.count >= self.max_symbolic_replans:
            grace = 0.0   # no replan can rescue this run; do not wait for one
        if (self.sim_time_now() - self._plan_finished_sim_s) < grace:
            return

        self.plan_exhausted = True
        remaining = self.regions_remaining()
        self.record_json_event("plan_exhausted", {
            "mission_complete": self.mission_complete,
            "plan_exhausted": self.plan_exhausted,
            "regions_remaining": remaining,
            "at_target": self.at_target_region(),
        })
        if self.mission_complete:
            self.get_logger().info("[PLAN EXHAUSTED] mission complete; nothing left to drive")
        else:
            self.get_logger().warn(
                f"[PLAN EXHAUSTED] plan finished but the mission is NOT complete -- "
                f"{len(remaining)} region(s) never visited: {remaining}. "
                f"A replan narrowed to the target region drops the rest of the mission."
            )

    def mission_timeout_tick(self):
        """Hard cap on mission duration, in sim time."""
        if self.mission_timeout_s <= 0 or self.plan_exhausted:
            return
        elapsed = self.sim_time_now() - (self.started_at.nanoseconds / 1e9)
        if elapsed < self.mission_timeout_s:
            return
        self.plan_exhausted = True
        self.get_logger().error(
            f"[MISSION TIMEOUT] {elapsed:.0f}s exceeded the {self.mission_timeout_s:.0f}s cap; "
            f"regions never visited: {self.regions_remaining()}"
        )
        self.record_json_event("mission_timeout", {
            "elapsed_s": elapsed, "cap_s": self.mission_timeout_s,
            "regions_remaining": self.regions_remaining()})

    def publish_evoplan_status(self):
        """1 Hz status blob that scand_metrics folds into its results JSON."""
        self.status_pub.publish(String(data=json.dumps({
            "state": self.symbolic.state,
            "symbolic_replans": self.symbolic.count,
            "deliberation_s_wall": self.symbolic.deliberation_wall_s,
            "held_s": self.symbolic.held_sim_s,
            "shield_vetoes": self.shield_veto_count,
            "blocked_regions": sorted(self.symbolic.blocked_regions),
            # scand_metrics gates success on this instead of position alone.
            "mission_complete": self.mission_complete,
            # ...and gates TERMINATION on this one. Omitting it made
            # scand_metrics read None -- falsy -- so a plan that finished
            # short of the mission never stopped the run: the executor logged
            # [PLAN EXHAUSTED] and the trial then idled until its timeout
            # (observed once at 13.4 hours with the whole sim still up).
            "plan_exhausted": self.plan_exhausted,
            # Find-an-object missions: which phase, and what the search found.
            "find_phase": self.find_phase,
            "find_object_class": self.find_object_class,
            "find_object_region": self.find_object_region,
            "find_object_evidence": self.find_object_evidence,
            # Find-and-inspect missions: how many objects turned up and how many
            # have been looked at. The object count is a RESULT here, not a
            # setting -- it is what the mission was sent to determine -- so it
            # belongs in the status blob alongside the region counts.
            "inspect_phase": self.inspect_phase,
            "inspect_classes": self.inspect_object_classes,
            "inspect_dwell_s": self.inspect_dwell_s,
            "inspection": (self.object_registry.summary()
                           if self.object_registry else None),
            # Rounds attempted vs allowed. A run that ends with objects pending
            # is either "the planner kept failing" or "it never tried", and
            # only this tells them apart after the fact.
            "inspect_rounds": self._inspection_round_attempts,
            "inspect_max_rounds": self.inspect_max_rounds,
            "inspecting": ((self._dwell or {}).get("task") or {}).get("object"),
            "target_region": self.target_region,
            "regions_required": len(set(self.mission_required_regions)),
            "regions_visited": len(self.mission_visited_regions),
            "regions_remaining": self.regions_remaining(),
            # Closest the robot ever got to each region, so "0 visited" can be
            # told apart from "never drove there" without a rerun.
            "region_closest_m": {k: round(v, 2)
                                 for k, v in sorted(self._region_closest.items())},
        })))

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
            plan_report = self.load_pddl_plan_from_file()
            if not plan_report["ok"]:
                self.get_logger().error(f"No valid PDDL plan found: {plan_report.get('history', [])[-3:]}")
                self.sent_goal = True
                return

            waypoints, tasks = self.plan_to_nav2_goals(plan_report["plan_actions"])
            self.active_plan_report = plan_report
            self.active_plan_actions = plan_report["plan_actions"]
            self.set_route(waypoints, tasks)
            # Freeze what the MISSION requires, before any replan can shorten it.
            # Success is measured against this, never against the plan currently
            # loaded: a replan that narrows to "(visited <target>)" would
            # otherwise complete itself in one move. factory_mission_01 revisits
            # R8 at step 2 of 28, so a position-only success test scored a
            # two-action run as 100%.
            self.mission_required_regions = [
                a.args[-1].lower() for a in plan_report["plan_actions"]
                if a.name.startswith("move")
            ]
            self.get_logger().info(
                f"Mission requires visiting {len(set(self.mission_required_regions))} "
                f"distinct regions across {len(self.mission_required_regions)} moves"
            )

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
            self.archive_initial_plan(plan_report)
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
        objects.update({name: "object" for name in self.objects})

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
        if self.domain_name == "lens_lab":
            self.add_lens_lab_facts(objects, facts)

        # Discovered objects, so a returned plan containing inspect-object can
        # be validated at all. This problem is built from graph.json, which
        # knows nothing about what perception found -- without these the
        # in-container gate in apply_symbolic_replan would reject every
        # inspection plan for "missing preconditions (object-at ...)", i.e. for
        # naming the very objects the mission was sent to find.
        for target in (self.object_registry.all() if self.object_registry else []):
            if target.region in self.regions:
                objects[target.name] = "target"
                facts.add(("object-at", target.name, target.region))

        return ProblemState(
            objects=objects,
            facts=facts,
            goals={("visited", self.target_region)},
        )

    def add_lens_lab_facts(self, objects, facts):
        object_regions = {
            "desk_1": "top",
            "whiteboard": "bottom",
            "lambda_1": "lambdas",
            "lambda_2": "lambdas",
            "lambda_3": "lambdas",
        }
        for name, xy in self.objects.items():
            region = object_regions.get(name)
            if region not in self.regions:
                region, _ = self.nearest_region(xy)
            if region is None:
                continue
            facts.add(("object-at", name, region))
            if name.startswith("lambda_"):
                objects[name] = "lab_equipment"
                facts.add(("equipment", name))
            else:
                facts.add(("furniture-item", name))

        for region in self.regions:
            facts.add(("clear-access", region))
        for region in ("top", "bottom", "lambdas"):
            if region in self.regions:
                facts.add(("inspection-zone", region))
        if "lambdas" in self.regions:
            facts.add(("compute-zone", "lambdas"))
            facts.add(("fabrication-zone", "lambdas"))
        if "fire" in self.regions:
            facts.add(("fire-safety-zone", "fire"))

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

    def load_pddl_plan_from_file(self):
        self.get_logger().info(f"Loading PDDL plan from {self.plan_file}")
        candidate = self.parse_fast_downward_plan(self.plan_file)
        # Align first: the prefix is computed from the robot's live pose, and
        # an unaligned first move would be judged inapplicable at index 0 and
        # truncate the whole plan away.
        candidate = self.align_plan_start_with_current_region(candidate)
        candidate = self.truncate_to_executable_prefix(candidate)
        history = [{
            "planner": "plan_file",
            "plan_file": str(self.plan_file),
            "plan": [action.text() for action in candidate],
            "ok": True,
            "validation_skipped": True,
            "errors": [],
        }]

        self.get_logger().info(
            f"Selected PDDL plan from file without symbolic validation: {self.format_plan(candidate)}"
        )
        self.record_json_event(
            "pddl_plan_file_loaded",
            {
                "plan_file": str(self.plan_file),
                "pddl_plan": [action.text() for action in candidate],
                "validation_skipped": True,
            },
        )
        return {
            "ok": True,
            "plan": [action.text() for action in candidate],
            "plan_actions": candidate,
            "history": history,
        }

    def find_mission_problem(self):
        """Path to the authored problem for ``mission_id``, or None.

        Two directories because the mission corpus is split: the curated
        delivery/inspection problems live in the EvoPlan submodule, which is
        kept pristine, and the tours authored for this stack live in
        pipeline/missions. The replan service searches the same pair.
        """
        if self.mission_problem_file:
            path = Path(self.mission_problem_file)
            return path if path.is_file() else None
        for parent in ("pipeline/missions", "evolve_stl_pddl/jackal/in"):
            candidate = Path.cwd() / parent / f"{self.mission_id}.pddl"
            if candidate.is_file():
                return candidate
        return None

    def archive_initial_plan(self, plan_report):
        """Archive the starting plan as ``<tag>_replan_0``.

        The replan service archives every plan it produces, but it never sees
        this one: the initial plan is read straight off disk here, without a
        problem being built or a planner being run. That left index 0 missing
        from every trial, so reconstructing a run meant opening _replan_1.pddl
        and inferring backwards what it had replaced -- and the most common
        question about a failed run ("was it already going the wrong way, or
        did the replan send it there?") was the one the artifacts could not
        answer.

        What is archived is what genuinely exists at this point: the AUTHORED
        mission problem, unspliced, because no runtime problem is built for the
        initial plan. `authored: true` in the .json says so, so the .pddl is
        not mistaken for something a planner was handed.

        Never raises. An unwritable results directory must not stop the robot
        driving a plan it has already loaded.
        """
        try:
            out_dir = Path(self.replan_artifact_dir)
            if not out_dir.is_absolute():
                out_dir = Path.cwd() / out_dir
            tag = safe_tag(self.trial_tag, fallback=self.mission_id)
            problem = self.find_mission_problem()
            stem = write_artifacts(
                out_dir, tag, next_index(out_dir, tag),
                problem.read_text() if problem else None,
                plan_report["plan"], {
                    "source": "plan_file",
                    "authored": True,
                    "status": "ok",
                    "planner": "plan_file",
                    "valid": None,           # loaded without symbolic validation
                    "plan": plan_report["plan"],
                    "plan_file": str(self.plan_file),
                    "problem_file": str(problem) if problem else None,
                    "mission_id": self.mission_id,
                    "current_region": self.current_region,
                    "target_region": self.target_region,
                    "planner_mode": self.planner_mode,
                    "trigger": "initial_plan",
                })
            self.get_logger().info(
                f"Archived initial plan -> {stem.name}.{{pddl,plan,json}}")
        except Exception as exc:  # noqa: BLE001 - diagnostics never block driving
            self.get_logger().warn(f"Could not archive initial plan: {exc}")

    def align_plan_start_with_current_region(self, plan):
        """Thin ROS wrapper: the arithmetic lives in symbolic_replan."""
        plan, note = align_plan_start(plan, self.current_region, self.robot_name)
        if note:
            self.get_logger().info(note)
        return plan

    def plan_to_nav2_goals(self, plan):
        """Lower a PDDL plan to ``(waypoints, tasks)``.

        Two action types reach the robot. ``move`` becomes a waypoint at the
        destination region's centroid, as it always has. ``inspect-object``
        becomes a waypoint too -- at the region closest to the object, oriented
        to face it, and marked with a task the arrival handler turns into a
        stationary dwell. Everything else (pickup, dropoff, inspect-shelf,
        approach) still lowers to nothing, because nothing on this robot
        performs it.

        ``tasks`` is index-parallel to ``waypoints``, one dict each, tagged
        ``kind`` ``"move"`` or ``"inspect"`` and carrying the action it came
        from. That last field is what lets progress logging, the current-leg
        report and the replan request's executed/remaining split be stated in
        terms of the route the robot is actually driving; they used to index a
        filtered list of moves, which an interleaved inspection silently
        knocks out of step.
        """
        waypoints, tasks = [], []
        for action in plan:
            name = action.name.lower()
            if name == "inspect-object":
                task = self.inspection_waypoint(action)
                if task is None:
                    continue
                # Appended WITHOUT the co-location check `append_waypoint`
                # applies to moves. An inspection at the region the robot was
                # just sent to is the normal case, not a duplicate: same
                # position, different heading, and a dwell afterwards. Dropping
                # it as a repeat would silently delete the inspection and leave
                # a plan that drives the route and does none of the work.
                waypoints.append(task["xy"])
                tasks.append(dict(task, kind="inspect", action=action))
                continue
            if not name.startswith("move") or len(action.args) < 3:
                continue
            goal_region = action.args[-1].lower()
            if goal_region not in self.regions:
                self.get_logger().warn(
                    f"Skipping move action with unknown goal region '{goal_region}': {action.text()}"
                )
                continue
            if self.append_waypoint(waypoints, self.regions[goal_region]):
                tasks.append({"kind": "move", "action": action,
                              "region": goal_region})

        if not waypoints:
            raise RuntimeError(
                "PDDL plan lowered to zero waypoints: no move or inspect-object "
                "action named a region in graph.json")
        return waypoints, tasks

    def inspection_waypoint(self, action):
        """Where to stand and which way to look, for one ``inspect-object``.

        ``(inspect-object ?r ?t ?l)`` names the region, and the region is where
        the robot goes: centroids are waypoints Nav2 is known to be able to
        reach, whereas a pose computed near the object can land inside the
        obstacle being inspected, or behind it. ``inspect_standoff_m`` moves the
        pose off the centroid toward the object for missions where that is too
        far away to see anything; it is 0 by default.

        The heading is the part that has to come from perception: the PDDL says
        nothing about where in the region the object sits, so the yaw is
        computed from the registry's map position for it. With no registry entry
        -- a plan naming an object this executor never discovered, which an LLM
        replan can produce -- the robot still drives there and still dwells, but
        with no facing to aim for. That is a degraded inspection, and it is
        logged as one rather than skipped: standing in the right region for five
        seconds is closer to the mission than refusing to go.
        """
        name = (action.args[1].lower() if len(action.args) >= 2 else "")
        region = action.args[-1].lower() if action.args else ""
        if region not in self.regions:
            self.get_logger().warn(
                f"Skipping inspect-object with unknown region '{region}': {action.text()}"
            )
            return None
        target = self.object_registry.get(name) if self.object_registry else None
        if target is None:
            self.get_logger().warn(
                f"[INSPECT] {name} is not in the object registry; standing at "
                f"{region} with no facing"
            )
        object_xy = target.xy if target is not None else None
        stand, yaw = inspection_pose(self.regions[region], object_xy,
                                     self.inspect_standoff_m)
        return {"object": name,
                "class": target.class_name if target is not None else None,
                "region": region, "xy": stand,
                "object_xy": tuple(object_xy) if object_xy else None, "yaw": yaw}

    def set_route(self, waypoints, tasks):
        """Install a new route. The ONLY place the parallel lists are set.

        Pads or trims ``tasks`` to match rather than trusting the caller: a
        length mismatch would misattribute dwells to the wrong waypoints, which
        is the sort of bug that looks like "the robot stopped in a strange
        place" long after the cause.

        ``route_actions`` is the whole route and is never sliced, unlike the
        other two. It is indexed by ``waypoint index + _waypoints_offset``,
        which is the existing convention for "where in the original route am
        I" -- the offset accumulates precisely so that a suffix resend does not
        lose the count.
        """
        self.active_waypoints = list(waypoints)
        tasks = list(tasks or [])
        if len(tasks) != len(self.active_waypoints):
            tasks = (tasks + [None] * len(self.active_waypoints))[:len(self.active_waypoints)]
        self.waypoint_tasks = tasks
        self.route_actions = [(task or {}).get("action") for task in tasks]

    def slice_route(self, start):
        """Drop the first ``start`` waypoints, keeping the tasks aligned."""
        start = max(0, int(start or 0))
        self.active_waypoints = self.active_waypoints[start:]
        self.waypoint_tasks = self.waypoint_tasks[start:]
        return self.active_waypoints

    def route_segment(self, waypoints):
        """The leading run of ``waypoints`` that can be driven in one goal.

        Nav2's ``FollowWaypoints`` drives a list straight through; the only
        pause it offers is ``WaitAtWaypoint``, which is a global plugin
        parameter and would stop the robot at EVERY waypoint for the same
        duration. An inspection has to pause at one waypoint and not the
        others, so the route is cut into segments ending at each inspection and
        the dwell happens between goals, where this node is in control.

        A segment therefore runs up to and including the first inspection
        waypoint, or to the end of the route when there is none -- which is
        every waypoint of a mission with no inspections, i.e. exactly the old
        single-goal behaviour.
        """
        end = segment_end(self.waypoint_tasks[:len(waypoints)])
        return list(waypoints if end is None else waypoints[:end + 1])

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
        obstacles = self.build_tracked_obstacles()
        self.log_monitor_obstacles(obstacles)
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
            obstacle_type = self.object_type_from_name(tracked.label)
            if obstacle_type != "human":
                continue
            obstacles.append(
                ObstacleConstraint(
                    name=f"track_{name}",
                    xy=tracked.xy,
                    obstacle_type=obstacle_type,
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
            self.clear_costmap_edit_if_active("STL monitor is satisfied")
            return plan_report
        if monitor_result.min_obstacle_margin >= 0.0:
            self.clear_costmap_edit_if_active("STL obstacle clearance is restored")
            return plan_report
        return self.replan_nav2_from_stl_feedback(plan_report, monitor_result)

    def replan_nav2_from_stl_feedback(self, plan_report, monitor_result):
        if not self.enable_stl_replan:
            return plan_report
        if self.replan_in_progress:
            return plan_report
        if self.nav2_replan_count >= self.max_nav2_replans:
            # Tier 1 is out of options. Rather than give up and keep driving the
            # route that has already failed three times, hand up to the
            # symbolic layer, which can route around the region entirely.
            self.get_logger().warn(
                f"STL feedback Nav2 replan limit reached ({self.max_nav2_replans}); "
                "escalating to a symbolic replan"
            )
            return self.escalate_symbolic_replan(
                plan_report,
                trigger="reactive_budget_exhausted",
                monitor_result=monitor_result,
            )

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
            remaining_start = self._last_feedback_waypoint or 0
            # Shared with the Tier-2 abandon path so the offset arithmetic
            # cannot drift between the two.
            remaining_waypoints = self.resend_waypoint_suffix(remaining_start)
            self.record_json_event(
                "nav2_replan_sent",
                {
                    "pddl_plan": plan_report.get("plan", []),
                    "nav2_goals": [
                        {"x": x, "y": y}
                        for x, y in remaining_waypoints
                    ],
                    "skipped_waypoints": remaining_start,
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
        if monitor_result.closest_obstacle_type != "human":
            self.get_logger().info(
                "Skipping STL costmap inflation because closest violating obstacle "
                f"is type={monitor_result.closest_obstacle_type}; only human obstacles are inflated"
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
        radius = min(radius, self.costmap_edit_max_radius)
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

        self.active_costmap_edit = {
            "closest_obstacle": monitor_result.closest_obstacle,
            "center": monitor_result.closest_obstacle_xy,
            "radius": radius,
            "stamp": self.get_clock().now(),
        }
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

    def restore_costmap_if_obstacle_cleared(self):
        if self.active_costmap_edit is None:
            return
        closest_obstacle = self.active_costmap_edit.get("closest_obstacle", "")
        if not str(closest_obstacle).startswith("track_"):
            return

        track_name = str(closest_obstacle)[len("track_"):]
        tracked = self.tracked_objects.get(track_name)
        if tracked is None:
            self.clear_costmap_edit_if_active(f"{closest_obstacle} is no longer tracked")
            return

        age_s = (self.get_clock().now() - tracked.stamp).nanoseconds / 1e9
        if age_s > self.tracked_object_timeout_s:
            self.tracked_objects.pop(track_name, None)
            self.clear_costmap_edit_if_active(f"{closest_obstacle} timed out")

    def clear_costmap_edit_if_active(self, reason):
        if self.active_costmap_edit is None:
            return False
        if self.map_msg is None:
            return False

        restored_map = deepcopy(self.map_msg)
        restored_map.header.stamp = self.get_clock().now().to_msg()
        stamp_key = (restored_map.header.stamp.sec, restored_map.header.stamp.nanosec)
        self.published_costmap_edit_stamps.add(stamp_key)
        if len(self.published_costmap_edit_stamps) > 20:
            self.published_costmap_edit_stamps = set(list(self.published_costmap_edit_stamps)[-20:])

        for _ in range(self.costmap_edit_publish_repeats):
            self.costmap_edit_pub.publish(restored_map)

        edit = self.active_costmap_edit
        self.active_costmap_edit = None
        self.get_logger().warn(
            f"Removed STL costmap inflation on {self.costmap_edit_topic}: "
            f"reason={reason}, closest_obstacle={edit.get('closest_obstacle')}"
        )
        self.record_json_event(
            "stl_costmap_edit_removed",
            {
                "topic": self.costmap_edit_topic,
                "reason": reason,
                "closest_obstacle": edit.get("closest_obstacle"),
                "center": (
                    {"x": edit["center"][0], "y": edit["center"][1]}
                    if edit.get("center") is not None
                    else None
                ),
                "radius": edit.get("radius"),
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
        """Append unless it repeats the last point. True when it was appended.

        The return value is what keeps ``waypoint_tasks`` aligned: a caller
        must add a task entry if and only if a waypoint was actually added.
        """
        point = (float(xy[0]), float(xy[1]))
        if not waypoints or math.hypot(point[0] - waypoints[-1][0], point[1] - waypoints[-1][1]) > 1e-6:
            waypoints.append(point)
            return True
        return False

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

    def build_poses(self, waypoints, tasks=None):
        """Poses for ``waypoints``, headings pointing along the route.

        An inspection waypoint overrides that heading with one aimed at the
        object, which is what makes Nav2 leave the robot facing what it is
        about to look at instead of facing the next leg of the tour. The dwell
        corrects any residual error afterwards, but arriving already aligned is
        what keeps the correction to a nudge.

        ``tasks`` defaults to the head of ``waypoint_tasks`` because every
        caller passes either the whole route or a prefix of it -- the segment
        being driven, or the remaining route for RViz. A suffix is taken by
        ``slice_route``, which shifts both lists together.
        """
        if tasks is None:
            tasks = self.waypoint_tasks[:len(waypoints)]
        poses = []
        for idx, (x, y) in enumerate(waypoints):
            task = tasks[idx] if idx < len(tasks) else None
            if task is not None and task.get("yaw") is not None:
                yaw = float(task["yaw"])
            elif idx < len(waypoints) - 1:
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
        # New route: the robot has something to drive again.
        self._plan_finished_sim_s = None
        # Only as far as the next inspection, if there is one before the end.
        # Callers hand over the whole remaining route and do not need to know
        # this happened -- `result_callback` drives the rest.
        segment = self.route_segment(waypoints)
        self._segment_len = len(segment)
        # Kept so a rejected goal can be re-sent verbatim.
        self._pending_waypoints = list(segment)
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.build_poses(segment)
        future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def retry_rejected_goal(self):
        """Re-send the last waypoints after Nav2 refused them."""
        if self._goal_retry_timer is not None:
            self._goal_retry_timer.cancel()
            self._goal_retry_timer = None
        if self.active_goal_handle is not None or self._pending_waypoints is None:
            return          # something else already got a goal through
        self.get_logger().info(
            f"Re-sending waypoint goal "
            f"(attempt {self._goal_reject_retries}/{self.goal_reject_max_retries})"
        )
        self.send_waypoints(self._pending_waypoints)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Almost always Nav2 not finished activating. Retrying costs a few
            # seconds; not retrying costs the entire run.
            if self._goal_reject_retries < self.goal_reject_max_retries:
                self._goal_reject_retries += 1
                self.get_logger().warn(
                    f"Nav2 waypoint goal rejected; retrying in "
                    f"{self.goal_reject_retry_s:.1f}s "
                    f"({self._goal_reject_retries}/{self.goal_reject_max_retries}) "
                    f"-- waypoint_follower is most likely not active yet"
                )
                self._goal_retry_timer = self.create_timer(
                    self.goal_reject_retry_s, self.retry_rejected_goal
                )
                return
            self.get_logger().error(
                f"Nav2 waypoint goal rejected {self._goal_reject_retries} times; "
                f"giving up. The mission cannot start."
            )
            self.record_json_event("nav2_goal_rejected", {
                "retries": self._goal_reject_retries, "terminal": True})
            return
        self._goal_reject_retries = 0
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
        idx = feedback_msg.feedback.current_waypoint
        if self._last_feedback_waypoint is not None and idx > self._last_feedback_waypoint:
            completed = idx - 1
            # Against the route, so the count is over everything the robot was
            # asked to do rather than over its moves alone -- an inspection is a
            # step, and one that takes five seconds of standing still is
            # precisely the step worth seeing in a log.
            route = self.route_actions or []
            total = len(route)
            global_idx = completed + self._waypoints_offset
            if global_idx < total and route[global_idx] is not None:
                self.get_logger().info(
                    f"[PLAN STEP DONE] ({global_idx + 1}/{total}) "
                    f"{route[global_idx].text()}"
                )
        self._last_feedback_waypoint = idx

    def result_callback(self, future):
        outcome = future.result()
        status = getattr(outcome, "status", GoalStatus.STATUS_SUCCEEDED)
        result = outcome.result
        self.active_goal_handle = None

        if status == GoalStatus.STATUS_CANCELED:
            # Distinguish our own cancel from an external one. Tier 1 and the
            # hold both cancel deliberately; treating those as failures would
            # escalate on every replan and loop forever.
            ours = self.replan_in_progress or self.symbolic.state != symbolic_replan.IDLE
            self.record_json_event("nav2_goal_canceled", {"self_initiated": ours})
            if ours:
                self.get_logger().info("Nav2 goal canceled by us; no escalation")
                return
            self.get_logger().warn("Nav2 goal canceled externally; escalating")
            self.escalate_symbolic_replan(
                self.active_plan_report, trigger="nav2_goal_aborted"
            )
            return

        if status == GoalStatus.STATUS_ABORTED:
            failed_region = self.region_of_waypoint(self._last_feedback_waypoint)
            self.get_logger().warn(
                f"Nav2 aborted the waypoint goal near {failed_region or 'an unknown region'}"
            )
            self.record_json_event(
                "nav2_goal_aborted",
                {
                    "status": int(status),
                    "waypoint_idx": self._last_feedback_waypoint,
                    "region": failed_region,
                },
            )
            self.escalate_symbolic_replan(
                self.active_plan_report,
                trigger="nav2_goal_aborted",
                failed_region=failed_region,
            )
            self.settle_if_nothing_running(
                f"Nav2 aborted near {failed_region or 'an unknown region'} and no "
                f"replan could be escalated")
            return

        missed = list(getattr(result, "missed_waypoints", []) or [])
        if missed:
            self.get_logger().warn(f"Missed waypoints: {missed}")
            self.record_json_event("nav2_goal_finished", {"missed_waypoints": missed})
        else:
            self.get_logger().info("All PPDDL Nav2 STL-SAT waypoints completed successfully")
            self.record_json_event("nav2_goal_finished", {"missed_waypoints": []})

        # A segment that ends in an inspection has not finished the plan -- it
        # has arrived at the thing to look at. Dwell, then drive the rest.
        arrived = (self.waypoint_tasks[self._segment_len - 1]
                   if 0 < self._segment_len <= len(self.waypoint_tasks) else None)
        if (arrived or {}).get("kind") == "inspect":
            self.begin_inspect_dwell(arrived)
            return
        if self._segment_len and self._segment_len < len(self.active_waypoints or []):
            # Defensive: segments end at inspections, so a short one that is not
            # an inspection means the route was re-cut underneath this callback.
            # Carrying on beats stalling with waypoints left undriven.
            self.advance_route()
            return
        # The robot has nothing left to drive. Whether that means SUCCESS is a
        # separate question -- a replan narrowed to (visited <target>) finishes
        # its plan without visiting the regions the mission requires. Mark the
        # plan exhausted and let plan_exhaustion_tick decide, so the run cannot
        # simply hang: a completed-but-incomplete mission previously idled 11
        # minutes to the trial timeout with the robot parked at its goal.
        self._plan_finished_sim_s = self.sim_time_now()


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
