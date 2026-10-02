#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener

from waypoint_follower.redis_pose_reader import RedisPoseReader


AMCL_POSE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MPPIFollower(Node):
    def __init__(self):
        super().__init__("mppi_follower")

        # ------------------------------------------------------------------
        # Topics / path parameters
        # ------------------------------------------------------------------
        self.localization_source = str(
            self.declare_parameter(
                "localization_source",
                "odom"
            ).value
        ).lower()

        self.cmd_vel_topic = self.declare_parameter(
            "cmd_vel_topic",
            "/a200_0000/cmd_vel"
        ).value

        self.odom_topic = self.declare_parameter(
            "odom_topic",
            "/a200_0000/odometry/filtered"
        ).value

        self.pose_topic = self.declare_parameter(
            "pose_topic",
            "/a200_0000/amcl_pose"
        ).value

        self.tf_fixed_frame = self.declare_parameter(
            "tf_fixed_frame",
            "map"
        ).value

        self.tf_robot_frame = self.declare_parameter(
            "tf_robot_frame",
            "a200_0000/base_link"
        ).value

        self.tf_timeout_s = float(
            self.declare_parameter(
                "tf_timeout_s",
                0.05
            ).value
        )

        self.trajectory_csv = self.declare_parameter(
            "trajectory_csv",
            "trajectories/warthog_trajectory.csv"
        ).value

        self.controller_path_csv = self.declare_parameter(
            "controller_path_csv",
            "trajectories/mppi_controller_path.csv"
        ).value

        self.controller_log_every_n = int(
            self.declare_parameter(
                "controller_log_every_n",
                1
            ).value
        )

        self.min_waypoint_spacing_m = float(
            self.declare_parameter(
                "min_waypoint_spacing_m",
                0.40
            ).value
        )

        self.recompute_path_yaw = bool(
            self.declare_parameter(
                "recompute_path_yaw",
                True
            ).value
        )

        self.control_rate_hz = float(
            self.declare_parameter(
                "control_rate_hz",
                20.0
            ).value
        )

        self.waypoint_tolerance_m = float(
            self.declare_parameter(
                "waypoint_tolerance_m",
                0.75
            ).value
        )

        self.goal_tolerance_m = float(
            self.declare_parameter(
                "goal_tolerance_m",
                0.50
            ).value
        )

        self.require_goal_orientation = bool(
            self.declare_parameter(
                "require_goal_orientation",
                False
            ).value
        )

        self.goal_yaw_tolerance_rad = float(
            self.declare_parameter(
                "goal_yaw_tolerance_rad",
                0.25
            ).value
        )

        self.align_trajectory_to_start = bool(
            self.declare_parameter(
                "align_trajectory_to_start",
                False
            ).value
        )

        self.closed_loop = bool(
            self.declare_parameter(
                "closed_loop",
                False
            ).value
        )

        self.loop_count = int(
            self.declare_parameter(
                "loop_count",
                1
            ).value
        )

        self.duplicate_endpoint_tolerance_m = float(
            self.declare_parameter(
                "duplicate_endpoint_tolerance_m",
                0.25
            ).value
        )

        self.reverse_allowed = bool(
            self.declare_parameter(
                "reverse_allowed",
                False
            ).value
        )

        self.stop_on_completion = bool(
            self.declare_parameter(
                "stop_on_completion",
                True
            ).value
        )

        self.status_log_period_s = float(
            self.declare_parameter(
                "status_log_period_s",
                1.0
            ).value
        )

        # ------------------------------------------------------------------
        # MPPI parameters
        # ------------------------------------------------------------------
        self.num_samples = int(
            self.declare_parameter(
                "num_samples",
                1500
            ).value
        )

        self.horizon_steps = int(
            self.declare_parameter(
                "horizon_steps",
                35
            ).value
        )

        self.model_dt = float(
            self.declare_parameter(
                "model_dt",
                0.10
            ).value
        )

        self.temperature = float(
            self.declare_parameter(
                "temperature",
                1.0
            ).value
        )

        self.noise_linear_std = float(
            self.declare_parameter(
                "noise_linear_std",
                0.25
            ).value
        )

        self.noise_angular_std = float(
            self.declare_parameter(
                "noise_angular_std",
                0.35
            ).value
        )

        # Robot limits
        self.max_linear_speed_mps = float(
            self.declare_parameter(
                "max_linear_speed_mps",
                0.8
            ).value
        )

        self.min_linear_speed_mps = float(
            self.declare_parameter(
                "min_linear_speed_mps",
                0.0
            ).value
        )

        self.max_angular_speed_rps = float(
            self.declare_parameter(
                "max_angular_speed_rps",
                0.7
            ).value
        )

        self.max_linear_accel_mps2 = float(
            self.declare_parameter(
                "max_linear_accel_mps2",
                0.6
            ).value
        )

        self.max_angular_accel_rps2 = float(
            self.declare_parameter(
                "max_angular_accel_rps2",
                1.0
            ).value
        )

        self.angular_direction_sign = float(
            self.declare_parameter(
                "angular_direction_sign",
                1.0
            ).value
        )

        self.path_window_points = int(
            self.declare_parameter(
                "path_window_points",
                35
            ).value
        )

        # Cost weights
        self.path_cost_weight = float(
            self.declare_parameter(
                "path_cost_weight",
                8.0
            ).value
        )

        self.heading_cost_weight = float(
            self.declare_parameter(
                "heading_cost_weight",
                1.5
            ).value
        )

        self.progress_cost_weight = float(
            self.declare_parameter(
                "progress_cost_weight",
                3.0
            ).value
        )

        self.terminal_cost_weight = float(
            self.declare_parameter(
                "terminal_cost_weight",
                10.0
            ).value
        )

        self.control_cost_weight = float(
            self.declare_parameter(
                "control_cost_weight",
                0.08
            ).value
        )

        self.angular_control_cost_weight = float(
            self.declare_parameter(
                "angular_control_cost_weight",
                0.10
            ).value
        )

        self.reverse_cost_weight = float(
            self.declare_parameter(
                "reverse_cost_weight",
                8.0
            ).value
        )

        # ------------------------------------------------------------------
        # Localization
        # ------------------------------------------------------------------
        self.current_pose = None
        self.last_odom_time = None

        if self.localization_source not in ("redis", "odom", "pose", "tf"):
            raise ValueError(
                "localization_source must be 'redis', 'odom', 'pose', or 'tf'"
            )

        self.redis_pose_reader = None
        self.odom_sub = None
        self.pose_sub = None
        self.tf_buffer = None
        self.tf_listener = None

        if self.localization_source == "redis":
            self.redis_pose_reader = RedisPoseReader(self)
        elif self.localization_source == "odom":
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                10
            )
        elif self.localization_source == "pose":
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_callback,
                AMCL_POSE_QOS
            )
        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(
                self.tf_buffer,
                self
            )

        # ------------------------------------------------------------------
        # Trajectory
        # ------------------------------------------------------------------
        self.raw_waypoint_count = 0
        self.waypoints = self.load_waypoints(self.trajectory_csv)

        self.trajectory_aligned = not self.align_trajectory_to_start
        self.progress_index = 0
        self.done = False

        # Nominal MPPI control sequence: [linear_x, angular_z]
        self.u_nominal = np.zeros(
            (self.horizon_steps, 2),
            dtype=np.float64
        )

        self.last_cmd = Twist()
        self.last_cost = float("nan")
        self.last_effective_samples = 0.0

        # Logging
        self.controller_log_count = 0
        self.controller_log_closed = False
        self.controller_log_file = None
        self.controller_log_writer = None
        self.open_controller_log()

        # ROS publisher/timers
        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop
        )

        self.status_timer = self.create_timer(
            max(self.status_log_period_s, 0.1),
            self.log_status
        )

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {self.trajectory_csv} "
            f"(raw={self.raw_waypoint_count}, spacing={self.min_waypoint_spacing_m:.2f} m); "
            f"MPPI samples={self.num_samples}, horizon={self.horizon_steps}, "
            f"model_dt={self.model_dt:.3f}s; "
            f"{self.pose_source_text()}, publishing to {self.cmd_vel_topic}"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def normalize_angle(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    @staticmethod
    def normalize_angle_array(angle):
        return np.arctan2(
            np.sin(angle),
            np.cos(angle)
        )

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(
            min_value,
            min(value, max_value)
        )

    # ------------------------------------------------------------------
    # Odometry subscriber
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.current_pose = (
            x,
            y,
            yaw
        )

        self.last_odom_time = (
            self.get_clock().now().nanoseconds * 1e-9
        )

    def pose_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y

        q = pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.current_pose = (
            x,
            y,
            yaw
        )

    # ------------------------------------------------------------------
    # Localization polling
    # ------------------------------------------------------------------
    def update_current_pose(self):
        if self.localization_source == "redis":
            return self.update_current_pose_from_redis()

        if self.localization_source == "tf":
            return self.update_current_pose_from_tf()

        return self.current_pose is not None

    def update_current_pose_from_redis(self):
        pose = self.redis_pose_reader.get_pose()

        if pose is None:
            return False

        self.current_pose = (
            pose.x,
            pose.y,
            pose.yaw
        )

        return True

    def update_current_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tf_fixed_frame,
                self.tf_robot_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s)
            )
        except TransformException:
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        siny_cosp = 2.0 * (
            rotation.w * rotation.z +
            rotation.x * rotation.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            rotation.y * rotation.y +
            rotation.z * rotation.z
        )

        self.current_pose = (
            translation.x,
            translation.y,
            math.atan2(
                siny_cosp,
                cosy_cosp
            )
        )

        return True

    def pose_source_text(self):
        if self.localization_source == "redis":
            return (
                f"reading Redis stream {self.redis_pose_reader.stream_name} "
                f"node {self.redis_pose_reader.target_node}"
            )

        if self.localization_source == "tf":
            return (
                f"reading TF {self.tf_fixed_frame} -> "
                f"{self.tf_robot_frame}"
            )

        if self.localization_source == "pose":
            return f"subscribing to pose on {self.pose_topic}"

        return f"subscribing to odometry on {self.odom_topic}"

    # ------------------------------------------------------------------
    # Trajectory handling
    # ------------------------------------------------------------------
    def load_waypoints(self, csv_file):
        path = self.resolve_package_path(csv_file)

        if not path.exists():
            raise FileNotFoundError(
                f"Trajectory CSV does not exist: {path}"
            )

        waypoints = []

        with path.open("r", newline="") as handle:
            reader = csv.DictReader(handle)

            required_columns = {"x", "y"}

            if not required_columns.issubset(
                reader.fieldnames or []
            ):
                raise ValueError(
                    f"Trajectory CSV must include columns: "
                    f"{sorted(required_columns)}"
                )

            for row in reader:
                try:
                    x = float(row["x"])
                    y = float(row["y"])

                    yaw = (
                        float(row["yaw"])
                        if row.get("yaw") not in (None, "")
                        else 0.0
                    )

                except ValueError:
                    continue

                waypoints.append(
                    (x, y, yaw)
                )

        if len(waypoints) < 2:
            raise ValueError(
                "Trajectory CSV must contain at least two valid waypoints"
            )

        self.raw_waypoint_count = len(waypoints)

        if self.closed_loop:
            first_x, first_y, _ = waypoints[0]
            last_x, last_y, _ = waypoints[-1]

            if (
                math.hypot(
                    last_x - first_x,
                    last_y - first_y
                )
                <= self.duplicate_endpoint_tolerance_m
            ):
                waypoints.pop()

        waypoints = self.downsample_waypoints(
            waypoints
        )

        if self.recompute_path_yaw:
            waypoints = self.with_path_yaws(
                waypoints
            )

        return waypoints

    def downsample_waypoints(self, waypoints):
        if (
            self.min_waypoint_spacing_m <= 0.0
            or len(waypoints) <= 2
        ):
            return waypoints

        downsampled = [
            waypoints[0]
        ]

        last_x, last_y, _ = waypoints[0]

        end_index = (
            len(waypoints)
            if self.closed_loop
            else len(waypoints) - 1
        )

        for x, y, yaw in waypoints[1:end_index]:
            if (
                math.hypot(
                    x - last_x,
                    y - last_y
                )
                >= self.min_waypoint_spacing_m
            ):
                downsampled.append(
                    (x, y, yaw)
                )

                last_x = x
                last_y = y

        if not self.closed_loop:
            downsampled.append(
                waypoints[-1]
            )

        if len(downsampled) < 2:
            return waypoints

        return downsampled

    def with_path_yaws(self, waypoints):
        path_yaws = []

        for i, (x, y, yaw) in enumerate(waypoints):
            if i < len(waypoints) - 1:
                next_x, next_y, _ = waypoints[i + 1]

                yaw = math.atan2(
                    next_y - y,
                    next_x - x
                )

            elif self.closed_loop and len(waypoints) > 1:
                next_x, next_y, _ = waypoints[0]

                yaw = math.atan2(
                    next_y - y,
                    next_x - x
                )

            elif path_yaws:
                yaw = path_yaws[-1][2]

            path_yaws.append(
                (x, y, yaw)
            )

        return path_yaws

    @staticmethod
    def resolve_package_path(path_value):
        path = Path(
            path_value
        ).expanduser()

        if path.is_absolute():
            return path

        package_share = Path(
            get_package_share_directory(
                "waypoint_follower"
            )
        )

        return package_share / path

    def align_waypoints_to_current_pose(self):
        if (
            self.current_pose is None
            or self.trajectory_aligned
        ):
            return

        current_x, current_y, current_yaw = self.current_pose

        if current_yaw is None:
            current_yaw = self.waypoints[0][2]

        first_x, first_y, first_yaw = self.waypoints[0]

        yaw_delta = self.normalize_angle(
            current_yaw - first_yaw
        )

        cos_yaw = math.cos(
            yaw_delta
        )

        sin_yaw = math.sin(
            yaw_delta
        )

        aligned = []

        for wx, wy, wyaw in self.waypoints:
            rel_x = wx - first_x
            rel_y = wy - first_y

            aligned_x = (
                current_x
                + rel_x * cos_yaw
                - rel_y * sin_yaw
            )

            aligned_y = (
                current_y
                + rel_x * sin_yaw
                + rel_y * cos_yaw
            )

            aligned_yaw = self.normalize_angle(
                wyaw + yaw_delta
            )

            aligned.append(
                (
                    aligned_x,
                    aligned_y,
                    aligned_yaw
                )
            )

        self.waypoints = aligned
        self.trajectory_aligned = True
        self.progress_index = 0

        self.get_logger().info(
            "Aligned recorded trajectory start to current robot pose "
            f"({current_x:.2f}, {current_y:.2f}, yaw={current_yaw:.2f})"
        )

    def waypoint_at(self, absolute_index):
        if self.closed_loop:
            return self.waypoints[
                absolute_index % len(self.waypoints)
            ]

        return self.waypoints[
            min(
                absolute_index,
                len(self.waypoints) - 1
            )
        ]

    def target_index(self):
        if self.closed_loop:
            return self.progress_index

        return min(
            self.progress_index,
            len(self.waypoints) - 1
        )

    def advance_sequential_waypoint(self, x, y):
        if self.is_final_waypoint():
            return

        idx = self.progress_index

        target_x, target_y, _ = self.waypoint_at(
            idx
        )

        dist_to_target = math.hypot(
            target_x - x,
            target_y - y
        )

        # Case 1: waypoint reached
        if (
            dist_to_target
            <= self.waypoint_tolerance_m
        ):
            self.progress_index += 1
            return

        # Case 2: robot passed waypoint
        next_x, next_y, _ = self.waypoint_at(
            idx + 1
        )

        path_dx = next_x - target_x
        path_dy = next_y - target_y

        robot_dx = x - target_x
        robot_dy = y - target_y

        dot = (
            robot_dx * path_dx
            + robot_dy * path_dy
        )

        if dot > 0.0:
            self.progress_index += 1

    def is_final_waypoint(self):
        if self.closed_loop:
            return (
                self.loop_count > 0
                and self.progress_index
                >= self.loop_count * len(self.waypoints)
            )

        return (
            self.progress_index
            >= len(self.waypoints) - 1
        )

    def reached_goal(self, x, y, yaw):
        if self.closed_loop:
            if self.loop_count <= 0:
                return False

            start_x, start_y, _ = self.waypoints[0]

            return (
                self.progress_index
                >= self.loop_count * len(self.waypoints)
                and math.hypot(
                    start_x - x,
                    start_y - y
                )
                <= self.goal_tolerance_m
            )

        goal_x, goal_y, goal_yaw = self.waypoints[-1]

        position_reached = (
            self.progress_index
            >= len(self.waypoints) - 1
            and math.hypot(
                goal_x - x,
                goal_y - y
            )
            <= self.goal_tolerance_m
        )

        if not position_reached:
            return False

        if not self.require_goal_orientation:
            return True

        if yaw is None:
            return False

        return (
            abs(
                self.normalize_angle(
                    goal_yaw - yaw
                )
            )
            <= self.goal_yaw_tolerance_rad
        )

    # ------------------------------------------------------------------
    # MPPI
    # ------------------------------------------------------------------
    def build_reference_window(self):
        count = max(
            self.path_window_points,
            2
        )

        refs = []

        for offset in range(count):
            absolute_index = (
                self.progress_index
                + offset
            )

            if (
                not self.closed_loop
                and absolute_index
                >= len(self.waypoints)
            ):
                break

            refs.append(
                self.waypoint_at(
                    absolute_index
                )
            )

        if not refs:
            refs.append(
                self.waypoint_at(
                    self.progress_index
                )
            )

        return np.asarray(
            refs,
            dtype=np.float64
        )

    def enforce_rollout_acceleration_limits(
        self,
        controls
    ):
        last_v = float(
            self.last_cmd.linear.x
        )

        # The published command may have steering sign applied.
        # Convert it back to the internal MPPI sign before constraining rollouts.
        last_w = float(
            self.last_cmd.angular.z
        )

        if abs(self.angular_direction_sign) > 1e-9:
            last_w /= self.angular_direction_sign

        max_dv = (
            self.max_linear_accel_mps2
            * self.model_dt
        )

        max_dw = (
            self.max_angular_accel_rps2
            * self.model_dt
        )

        previous_v = np.full(
            self.num_samples,
            last_v,
            dtype=np.float64
        )

        previous_w = np.full(
            self.num_samples,
            last_w,
            dtype=np.float64
        )

        for t in range(
            self.horizon_steps
        ):
            v = controls[:, t, 0]
            w = controls[:, t, 1]

            v = np.clip(
                v,
                previous_v - max_dv,
                previous_v + max_dv
            )

            w = np.clip(
                w,
                previous_w - max_dw,
                previous_w + max_dw
            )

            controls[:, t, 0] = v
            controls[:, t, 1] = w

            previous_v = v
            previous_w = w

    def mppi_control(
        self,
        x0,
        y0,
        yaw0
    ):
        reference = self.build_reference_window()

        ref_x = reference[:, 0]
        ref_y = reference[:, 1]
        ref_yaw = reference[:, 2]

        # Sample perturbations
        noise = np.empty(
            (
                self.num_samples,
                self.horizon_steps,
                2
            ),
            dtype=np.float64
        )

        noise[:, :, 0] = np.random.normal(
            0.0,
            self.noise_linear_std,
            (
                self.num_samples,
                self.horizon_steps
            )
        )

        noise[:, :, 1] = np.random.normal(
            0.0,
            self.noise_angular_std,
            (
                self.num_samples,
                self.horizon_steps
            )
        )

        controls = (
            self.u_nominal[None, :, :]
            + noise
        )

        if self.reverse_allowed:
            controls[:, :, 0] = np.clip(
                controls[:, :, 0],
                -self.max_linear_speed_mps,
                self.max_linear_speed_mps
            )
        else:
            controls[:, :, 0] = np.clip(
                controls[:, :, 0],
                self.min_linear_speed_mps,
                self.max_linear_speed_mps
            )

        controls[:, :, 1] = np.clip(
            controls[:, :, 1],
            -self.max_angular_speed_rps,
            self.max_angular_speed_rps
        )

        self.enforce_rollout_acceleration_limits(
            controls
        )

        x = np.full(
            self.num_samples,
            x0,
            dtype=np.float64
        )

        y = np.full(
            self.num_samples,
            y0,
            dtype=np.float64
        )

        yaw = np.full(
            self.num_samples,
            yaw0,
            dtype=np.float64
        )

        total_cost = np.zeros(
            self.num_samples,
            dtype=np.float64
        )

        for t in range(
            self.horizon_steps
        ):
            v = controls[:, t, 0]
            w = controls[:, t, 1]

            # Differential/skid-steer approximation
            x += (
                v
                * np.cos(yaw)
                * self.model_dt
            )

            y += (
                v
                * np.sin(yaw)
                * self.model_dt
            )

            yaw = self.normalize_angle_array(
                yaw
                + w * self.model_dt
            )

            # Distance to forward reference window
            dx = (
                x[:, None]
                - ref_x[None, :]
            )

            dy = (
                y[:, None]
                - ref_y[None, :]
            )

            dist_sq = (
                dx * dx
                + dy * dy
            )

            nearest = np.argmin(
                dist_sq,
                axis=1
            )

            nearest_dist_sq = dist_sq[
                np.arange(self.num_samples),
                nearest
            ]

            # Path tracking cost
            total_cost += (
                self.path_cost_weight
                * nearest_dist_sq
                * self.model_dt
            )

            # Path heading cost
            desired_yaw = ref_yaw[
                nearest
            ]

            heading_error = (
                self.normalize_angle_array(
                    desired_yaw - yaw
                )
            )

            total_cost += (
                self.heading_cost_weight
                * heading_error
                * heading_error
                * self.model_dt
            )

            # Progress reward
            progress_fraction = (
                nearest.astype(
                    np.float64
                )
                / max(
                    len(reference) - 1,
                    1
                )
            )

            total_cost -= (
                self.progress_cost_weight
                * progress_fraction
                * self.model_dt
            )

            # Control effort
            total_cost += (
                self.control_cost_weight
                * v * v
                +
                self.angular_control_cost_weight
                * w * w
            ) * self.model_dt

            if self.reverse_allowed:
                total_cost += (
                    self.reverse_cost_weight
                    * np.maximum(
                        -v,
                        0.0
                    )
                    * self.model_dt
                )

        # Terminal path cost
        dx = (
            x[:, None]
            - ref_x[None, :]
        )

        dy = (
            y[:, None]
            - ref_y[None, :]
        )

        terminal_dist_sq = (
            dx * dx
            + dy * dy
        )

        terminal_nearest = np.argmin(
            terminal_dist_sq,
            axis=1
        )

        terminal_min_dist_sq = (
            terminal_dist_sq[
                np.arange(
                    self.num_samples
                ),
                terminal_nearest
            ]
        )

        total_cost += (
            self.terminal_cost_weight
            * terminal_min_dist_sq
        )

        # MPPI importance weights
        min_cost = np.min(
            total_cost
        )

        scaled_cost = (
            -(total_cost - min_cost)
            / max(
                self.temperature,
                1e-6
            )
        )

        scaled_cost = np.clip(
            scaled_cost,
            -80.0,
            0.0
        )

        weights = np.exp(
            scaled_cost
        )

        weight_sum = np.sum(
            weights
        )

        if (
            not np.isfinite(weight_sum)
            or weight_sum <= 1e-12
        ):
            self.get_logger().warn(
                "MPPI weights collapsed; "
                "keeping previous nominal control"
            )
        else:
            weights /= weight_sum

            effective_noise = (
                controls
                - self.u_nominal[
                    None,
                    :,
                    :
                ]
            )

            correction = np.sum(
                weights[
                    :,
                    None,
                    None
                ]
                * effective_noise,
                axis=0
            )

            self.u_nominal += correction

            self.last_effective_samples = float(
                1.0
                / max(
                    np.sum(
                        weights * weights
                    ),
                    1e-12
                )
            )

        # Clamp nominal sequence
        if self.reverse_allowed:
            self.u_nominal[:, 0] = np.clip(
                self.u_nominal[:, 0],
                -self.max_linear_speed_mps,
                self.max_linear_speed_mps
            )
        else:
            self.u_nominal[:, 0] = np.clip(
                self.u_nominal[:, 0],
                self.min_linear_speed_mps,
                self.max_linear_speed_mps
            )

        self.u_nominal[:, 1] = np.clip(
            self.u_nominal[:, 1],
            -self.max_angular_speed_rps,
            self.max_angular_speed_rps
        )

        self.last_cost = float(
            min_cost
        )

        v_cmd = float(
            self.u_nominal[0, 0]
        )

        w_cmd_internal = float(
            self.u_nominal[0, 1]
        )

        w_cmd = (
            self.angular_direction_sign
            * w_cmd_internal
        )

        return (
            v_cmd,
            w_cmd
        )

    def shift_nominal_sequence(self):
        if self.horizon_steps <= 1:
            self.u_nominal[0, :] = 0.0
            return

        self.u_nominal[:-1, :] = (
            self.u_nominal[1:, :]
        )

        self.u_nominal[-1, :] = (
            self.u_nominal[-2, :]
        )

    # ------------------------------------------------------------------
    # Controller loop
    # ------------------------------------------------------------------
    def control_loop(self):
        if self.done:
            return

        self.update_current_pose()

        if self.current_pose is None:
            return

        self.align_waypoints_to_current_pose()

        x, y, yaw = self.current_pose

        self.advance_sequential_waypoint(
            x,
            y
        )

        if self.reached_goal(
            x,
            y,
            yaw
        ):
            self.done = True

            if self.stop_on_completion:
                self.publish_stop()

            self.get_logger().info(
                "Reached trajectory goal"
            )

            return

        linear_x, angular_z = (
            self.mppi_control(
                x,
                y,
                yaw
            )
        )

        cmd = Twist()

        cmd.linear.x = (
            linear_x
        )

        cmd.angular.z = (
            angular_z
        )

        self.last_cmd = cmd
        self.cmd_pub.publish(
            cmd
        )

        target_index = (
            self.target_index()
        )

        target_x, target_y, target_yaw = (
            self.waypoint_at(
                target_index
            )
        )

        self.log_controller_path(
            x,
            y,
            yaw,
            target_index,
            target_x,
            target_y,
            target_yaw,
            cmd
        )

        self.shift_nominal_sequence()

    # ------------------------------------------------------------------
    # Stop / logging
    # ------------------------------------------------------------------
    def publish_stop(self):
        self.last_cmd = Twist()

        self.cmd_pub.publish(
            Twist()
        )

    def open_controller_log(self):
        if not self.controller_path_csv:
            return

        path = self.resolve_package_path(
            self.controller_path_csv
        )

        path.parent.mkdir(
            parents=True,
            exist_ok=True
        )

        self.controller_log_file = path.open(
            "w",
            newline=""
        )

        self.controller_log_writer = csv.writer(
            self.controller_log_file
        )

        self.controller_log_writer.writerow([
            "time_sec",
            "x",
            "y",
            "yaw",
            "progress_index",
            "target_x",
            "target_y",
            "target_yaw",
            "linear_x",
            "angular_z",
            "mppi_min_cost",
            "effective_samples"
        ])

        self.get_logger().info(
            f"Logging MPPI controller path to {path}"
        )

    def log_controller_path(
        self,
        x,
        y,
        yaw,
        target_index,
        target_x,
        target_y,
        target_yaw,
        cmd
    ):
        if (
            self.controller_log_writer
            is None
        ):
            return

        self.controller_log_count += 1

        if (
            self.controller_log_count
            % max(
                self.controller_log_every_n,
                1
            )
            != 0
        ):
            return

        now = (
            self.get_clock()
            .now()
            .nanoseconds
            * 1e-9
        )

        yaw_value = (
            ""
            if yaw is None
            else f"{yaw:.6f}"
        )

        self.controller_log_writer.writerow([
            f"{now:.9f}",
            f"{x:.6f}",
            f"{y:.6f}",
            yaw_value,
            target_index,
            f"{target_x:.6f}",
            f"{target_y:.6f}",
            f"{target_yaw:.6f}",
            f"{cmd.linear.x:.6f}",
            f"{cmd.angular.z:.6f}",
            f"{self.last_cost:.6f}",
            f"{self.last_effective_samples:.2f}"
        ])

        self.controller_log_file.flush()

    def log_status(self):
        if self.done:
            return

        if self.current_pose is None:
            self.get_logger().warn(
                f"Waiting for localization from "
                f"{self.pose_source_text()}; "
                "no /cmd_vel will be published yet"
            )

            return

        x, y, yaw = self.current_pose

        target_index = self.target_index()

        target_x, target_y, _ = (
            self.waypoint_at(
                target_index
            )
        )

        distance = math.hypot(
            target_x - x,
            target_y - y
        )

        cmd_subscribers = (
            self.cmd_pub
            .get_subscription_count()
        )

        yaw_text = (
            "unknown"
            if yaw is None
            else f"{yaw:.2f}"
        )

        self.get_logger().info(
            "MPPI status: "
            f"pose=({x:.2f}, {y:.2f}, yaw={yaw_text}), "
            f"progress_index={target_index}, "
            f"target=({target_x:.2f}, {target_y:.2f}), "
            f"distance={distance:.2f}, "
            f"cmd=(linear.x={self.last_cmd.linear.x:.2f}, "
            f"angular.z={self.last_cmd.angular.z:.2f}), "
            f"cost={self.last_cost:.2f}, "
            f"ESS={self.last_effective_samples:.1f}/{self.num_samples}, "
            f"cmd_topic={self.cmd_vel_topic}, "
            f"cmd_subscribers={cmd_subscribers}"
        )

    def close_controller_log(self):
        if self.controller_log_closed:
            return

        if self.controller_log_file is not None:
            self.controller_log_file.flush()
            self.controller_log_file.close()

        self.controller_log_closed = True

    def destroy_node(self):
        if self.stop_on_completion:
            self.publish_stop()

        self.close_controller_log()

        super().destroy_node()


def main(args=None):
    rclpy.init(
        args=args
    )

    try:
        node = MPPIFollower()

    except (
        FileNotFoundError,
        ValueError
    ) as exc:
        rclpy.logging.get_logger(
            "mppi_follower"
        ).error(
            str(exc)
        )

        rclpy.shutdown()
        return

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
