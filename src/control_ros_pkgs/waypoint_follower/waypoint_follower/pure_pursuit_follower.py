#!/usr/bin/env python3

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from waypoint_follower.redis_pose_reader import RedisPoseReader


class PurePursuitFollower(Node):
    def __init__(self):
        super().__init__("pure_pursuit_follower")

        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/w200_0105/cmd_vel").value
        self.trajectory_csv = self.declare_parameter("trajectory_csv", "trajectories/warthog_trajectory.csv").value
        self.controller_path_csv = self.declare_parameter(
            "controller_path_csv", "trajectories/controller_path.csv"
        ).value
        self.controller_log_every_n = int(self.declare_parameter("controller_log_every_n", 1).value)
        self.min_waypoint_spacing_m = float(self.declare_parameter("min_waypoint_spacing_m", 0.75).value)
        self.recompute_path_yaw = bool(self.declare_parameter("recompute_path_yaw", True).value)
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 20.0).value)
        self.lookahead_distance_m = float(self.declare_parameter("lookahead_distance_m", 1.5).value)
        self.curvature_lookahead_floor_m = float(
            self.declare_parameter("curvature_lookahead_floor_m", 0.9).value
        )
        self.waypoint_tolerance_m = float(self.declare_parameter("waypoint_tolerance_m", 0.75).value)
        self.goal_tolerance_m = float(self.declare_parameter("goal_tolerance_m", 0.5).value)
        self.linear_speed_mps = float(self.declare_parameter("linear_speed_mps", 0.8).value)
        self.max_linear_speed_mps = float(self.declare_parameter("max_linear_speed_mps", 1.2).value)
        self.min_linear_speed_mps = float(self.declare_parameter("min_linear_speed_mps", 0.2).value)
        self.max_angular_speed_rps = float(self.declare_parameter("max_angular_speed_rps", 0.9).value)
        self.angular_direction_sign = float(self.declare_parameter("angular_direction_sign", 1.0).value)
        self.regulated_linear_scaling_enabled = bool(
            self.declare_parameter("regulated_linear_scaling_enabled", True).value
        )
        self.regulated_min_radius_m = float(self.declare_parameter("regulated_min_radius_m", 1.0).value)
        self.heading_slowdown_angle_rad = float(self.declare_parameter("heading_slowdown_angle_rad", 0.8).value)
        self.approach_slowdown_distance_m = float(
            self.declare_parameter("approach_slowdown_distance_m", 1.0).value
        )
        self.align_trajectory_to_start = bool(
            self.declare_parameter("align_trajectory_to_start", False).value
        )
        self.status_log_period_s = float(self.declare_parameter("status_log_period_s", 1.0).value)
        self.orientation_tracking_enabled = bool(
            self.declare_parameter("orientation_tracking_enabled", False).value
        )
        self.yaw_error_gain = float(self.declare_parameter("yaw_error_gain", 0.8).value)
        self.max_heading_error_for_full_speed_rad = float(
            self.declare_parameter("max_heading_error_for_full_speed_rad", 0.0).value
        )
        self.require_goal_orientation = bool(
            self.declare_parameter("require_goal_orientation", False).value
        )
        self.goal_yaw_tolerance_rad = float(
            self.declare_parameter("goal_yaw_tolerance_rad", 0.25).value
        )
        self.closed_loop = bool(self.declare_parameter("closed_loop", False).value)
        self.loop_count = int(self.declare_parameter("loop_count", 1).value)
        self.duplicate_endpoint_tolerance_m = float(
            self.declare_parameter("duplicate_endpoint_tolerance_m", 0.25).value
        )
        self.reverse_allowed = bool(self.declare_parameter("reverse_allowed", False).value)
        self.stop_on_completion = bool(self.declare_parameter("stop_on_completion", True).value)
        self.redis_pose_reader = RedisPoseReader(self)

        self.raw_waypoint_count = 0
        self.waypoints = self.load_waypoints(self.trajectory_csv)
        self.current_pose = None
        self.trajectory_aligned = not self.align_trajectory_to_start
        self.progress_index = 0
        self.done = False
        self.last_status_time = None
        self.last_cmd = Twist()
        self.last_alpha = 0.0
        self.last_curvature = 0.0
        self.last_speed_scale = 1.0
        self.controller_log_count = 0
        self.controller_log_closed = False
        self.controller_log_file = None
        self.controller_log_writer = None
        self.open_controller_log()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)
        self.status_timer = self.create_timer(max(self.status_log_period_s, 0.1), self.log_status)

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {self.trajectory_csv} "
            f"(raw={self.raw_waypoint_count}, spacing={self.min_waypoint_spacing_m:.2f} m); "
            f"reading Redis stream {self.redis_pose_reader.stream_name} "
            f"node {self.redis_pose_reader.target_node}, publishing to {self.cmd_vel_topic}"
        )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def load_waypoints(self, csv_file):
        path = self.resolve_package_path(csv_file)
        if not path.exists():
            raise FileNotFoundError(f"Trajectory CSV does not exist: {path}")

        waypoints = []
        with path.open("r", newline="") as handle:
            reader = csv.DictReader(handle)
            required_columns = {"x", "y"}
            if not required_columns.issubset(reader.fieldnames or []):
                raise ValueError(f"Trajectory CSV must include columns: {sorted(required_columns)}")

            for row in reader:
                try:
                    x = float(row["x"])
                    y = float(row["y"])
                    yaw = float(row["yaw"]) if row.get("yaw") not in (None, "") else 0.0
                except ValueError:
                    continue
                waypoints.append((x, y, yaw))

        if len(waypoints) < 2:
            raise ValueError("Trajectory CSV must contain at least two valid waypoints")
        self.raw_waypoint_count = len(waypoints)

        if self.closed_loop:
            first_x, first_y, _ = waypoints[0]
            last_x, last_y, _ = waypoints[-1]
            if math.hypot(last_x - first_x, last_y - first_y) <= self.duplicate_endpoint_tolerance_m:
                waypoints.pop()

        waypoints = self.downsample_waypoints(waypoints)
        if self.recompute_path_yaw:
            waypoints = self.with_path_yaws(waypoints)
        return waypoints

    def downsample_waypoints(self, waypoints):
        if self.min_waypoint_spacing_m <= 0.0 or len(waypoints) <= 2:
            return waypoints

        downsampled = [waypoints[0]]
        last_x, last_y, _ = waypoints[0]

        end_index = len(waypoints) if self.closed_loop else len(waypoints) - 1
        for x, y, yaw in waypoints[1:end_index]:
            if math.hypot(x - last_x, y - last_y) >= self.min_waypoint_spacing_m:
                downsampled.append((x, y, yaw))
                last_x, last_y = x, y

        if not self.closed_loop:
            downsampled.append(waypoints[-1])

        if len(downsampled) < 2:
            return waypoints
        return downsampled

    def with_path_yaws(self, waypoints):
        path_yaws = []
        for i, (x, y, yaw) in enumerate(waypoints):
            if i < len(waypoints) - 1:
                next_x, next_y, _ = waypoints[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            elif self.closed_loop and len(waypoints) > 1:
                next_x, next_y, _ = waypoints[0]
                yaw = math.atan2(next_y - y, next_x - x)
            elif path_yaws:
                yaw = path_yaws[-1][2]
            path_yaws.append((x, y, yaw))
        return path_yaws

    @staticmethod
    def resolve_package_path(path_value):
        path = Path(path_value).expanduser()
        if path.is_absolute():
            return path

        package_share = Path(get_package_share_directory("waypoint_follower"))
        return package_share / path

    def open_controller_log(self):
        if not self.controller_path_csv:
            return

        path = self.resolve_package_path(self.controller_path_csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.controller_log_file = path.open("w", newline="")
        self.controller_log_writer = csv.writer(self.controller_log_file)
        self.controller_log_writer.writerow([
            "time_sec",
            "x",
            "y",
            "yaw",
            "target_index",
            "target_x",
            "target_y",
            "target_yaw",
            "alpha",
            "curvature",
            "speed_scale",
            "linear_x",
            "angular_z",
        ])
        self.get_logger().info(f"Logging controller path to {path}")

    def update_current_pose_from_redis(self):
        pose = self.redis_pose_reader.get_pose()
        if pose is None:
            return False

        self.current_pose = (pose.x, pose.y, pose.yaw)
        return True

    def align_waypoints_to_current_pose(self):
        if self.current_pose is None or self.trajectory_aligned:
            return

        current_x, current_y, current_yaw = self.current_pose
        if current_yaw is None:
            current_yaw = self.waypoints[0][2]
        first_x, first_y, first_yaw = self.waypoints[0]
        yaw_delta = self.normalize_angle(current_yaw - first_yaw)
        cos_yaw = math.cos(yaw_delta)
        sin_yaw = math.sin(yaw_delta)

        aligned = []
        for wx, wy, wyaw in self.waypoints:
            rel_x = wx - first_x
            rel_y = wy - first_y
            aligned_x = current_x + rel_x * cos_yaw - rel_y * sin_yaw
            aligned_y = current_y + rel_x * sin_yaw + rel_y * cos_yaw
            aligned_yaw = self.normalize_angle(wyaw + yaw_delta)
            aligned.append((aligned_x, aligned_y, aligned_yaw))

        self.waypoints = aligned
        self.trajectory_aligned = True
        self.progress_index = 0
        self.get_logger().info(
            "Aligned recorded trajectory start to current robot pose "
            f"({current_x:.2f}, {current_y:.2f}, yaw {current_yaw:.2f})"
        )

    def waypoint_at(self, absolute_index):
        if self.closed_loop:
            return self.waypoints[absolute_index % len(self.waypoints)]
        return self.waypoints[min(absolute_index, len(self.waypoints) - 1)]

    def target_index(self):
        if self.closed_loop:
            return self.progress_index
        return min(self.progress_index, len(self.waypoints) - 1)


    def lookahead_target_index(self, x, y):
        last_index = len(self.waypoints) - 1

        for idx in range(self.progress_index, last_index + 1):
            wx, wy, _ = self.waypoint_at(idx)
            dist = math.hypot(wx - x, wy - y)

            if dist >= self.lookahead_distance_m:
                return idx

        return last_index
    def advance_sequential_waypoint(self, x, y):
        if self.is_final_waypoint():
            return

        idx = self.progress_index
        target_x, target_y, _ = self.waypoint_at(idx)

        dist_to_target = math.hypot(target_x - x, target_y - y)

        # Case 1: robot reached waypoint neighborhood
        if dist_to_target <= self.waypoint_tolerance_m:
            self.progress_index += 1
            return

        # Case 2: robot passed the waypoint but did not get close enough
        next_x, next_y, _ = self.waypoint_at(idx + 1)

        path_dx = next_x - target_x
        path_dy = next_y - target_y

        robot_dx = x - target_x
        robot_dy = y - target_y

        dot = robot_dx * path_dx + robot_dy * path_dy

        if dot > 0.0:
            self.progress_index += 1
    def is_final_waypoint(self):
        if self.closed_loop:
            return self.loop_count > 0 and self.progress_index >= self.loop_count * len(self.waypoints)
        return self.progress_index >= len(self.waypoints) - 1

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def reached_goal(self, x, y, yaw):
        if self.closed_loop:
            if self.loop_count <= 0:
                return False

            start_x, start_y, _ = self.waypoints[0]
            return (
                self.progress_index >= self.loop_count * len(self.waypoints)
                and math.hypot(start_x - x, start_y - y) <= self.goal_tolerance_m
            )

        goal_x, goal_y, goal_yaw = self.waypoints[-1]
        position_reached = (
            self.progress_index >= len(self.waypoints) - 1
            and math.hypot(goal_x - x, goal_y - y) <= self.goal_tolerance_m
        )
        if not position_reached:
            return False

        if not self.require_goal_orientation:
            return True

        if yaw is None:
            return False

        return abs(self.normalize_angle(goal_yaw - yaw)) <= self.goal_yaw_tolerance_rad

    def regulated_speed(self, target_distance, alpha, curvature):
        speed = self.clamp(self.linear_speed_mps, 0.0, self.max_linear_speed_mps)
        if not self.regulated_linear_scaling_enabled:
            self.last_speed_scale = 1.0
            return speed

        speed_scale = 1.0

        abs_curvature = abs(curvature)
        if abs_curvature > 1e-6 and self.regulated_min_radius_m > 0.0:
            turn_radius = 1.0 / abs_curvature
            if turn_radius < self.regulated_min_radius_m:
                speed_scale = min(speed_scale, turn_radius / self.regulated_min_radius_m)

        if self.heading_slowdown_angle_rad > 0.0:
            heading_scale = self.clamp(
                1.0 - abs(alpha) / self.heading_slowdown_angle_rad,
                0.0,
                1.0,
            )
            speed_scale = min(speed_scale, heading_scale)

        if self.approach_slowdown_distance_m > 0.0:
            approach_scale = self.clamp(
                target_distance / self.approach_slowdown_distance_m,
                0.0,
                1.0,
            )
            speed_scale = min(speed_scale, approach_scale)

        self.last_speed_scale = speed_scale
        regulated = speed * speed_scale
        return self.clamp(regulated, self.min_linear_speed_mps, self.max_linear_speed_mps)

    def control_loop(self):
        if self.done:
            return
        self.update_current_pose_from_redis()
        if self.current_pose is None:
            return

        self.align_waypoints_to_current_pose()

        x, y, yaw = self.current_pose
        self.advance_sequential_waypoint(x, y)

        if self.reached_goal(x, y, yaw):
            self.done = True
            if self.stop_on_completion:
                self.publish_stop()
            self.get_logger().info("Reached trajectory goal")
            return

        # target_index = self.target_index()
        # target_x, target_y, target_yaw = self.waypoint_at(target_index)

        target_index = self.lookahead_target_index(x, y)
        target_x, target_y, target_yaw = self.waypoint_at(target_index)

        dx = target_x - x
        dy = target_y - y
        target_heading = math.atan2(dy, dx)
        if yaw is None:
            _, _, yaw = self.waypoint_at(self.progress_index)
        alpha = self.normalize_angle(target_heading - yaw)
        yaw_error = self.normalize_angle(target_yaw - yaw)

        # lookahead = max(math.hypot(dx, dy), self.curvature_lookahead_floor_m, 0.01)
        lookahead = max(math.hypot(dx, dy), self.curvature_lookahead_floor_m, 0.01)
        curvature = 2.0 * math.sin(alpha) / lookahead
        speed = self.regulated_speed(math.hypot(dx, dy), alpha, curvature)
        if self.reverse_allowed and abs(alpha) > math.pi / 2.0:
            speed = -speed
            alpha = self.normalize_angle(alpha + math.pi)
            yaw_error = self.normalize_angle(yaw_error + math.pi)

        if self.orientation_tracking_enabled and self.max_heading_error_for_full_speed_rad > 0.0:
            heading_scale = self.clamp(
                1.0 - abs(alpha) / self.max_heading_error_for_full_speed_rad,
                0.0,
                1.0,
            )
            min_speed = min(abs(speed), self.min_linear_speed_mps)
            speed = math.copysign(max(abs(speed) * heading_scale, min_speed), speed)

        yaw_feedback = self.yaw_error_gain * yaw_error if self.orientation_tracking_enabled else 0.0
        self.last_alpha = alpha
        self.last_curvature = curvature
        angular_z = self.clamp(
            self.angular_direction_sign * (speed * curvature + yaw_feedback),
            -self.max_angular_speed_rps,
            self.max_angular_speed_rps,
        )

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angular_z
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)
        self.log_controller_path(x, y, yaw, target_index, target_x, target_y, target_yaw, cmd)

    def log_controller_path(self, x, y, yaw, target_index, target_x, target_y, target_yaw, cmd):
        if self.controller_log_writer is None:
            return

        self.controller_log_count += 1
        if self.controller_log_count % max(self.controller_log_every_n, 1) != 0:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        yaw_value = "" if yaw is None else f"{yaw:.6f}"
        self.controller_log_writer.writerow([
            f"{now:.9f}",
            f"{x:.6f}",
            f"{y:.6f}",
            yaw_value,
            target_index,
            f"{target_x:.6f}",
            f"{target_y:.6f}",
            f"{target_yaw:.6f}",
            f"{self.last_alpha:.6f}",
            f"{self.last_curvature:.6f}",
            f"{self.last_speed_scale:.6f}",
            f"{cmd.linear.x:.6f}",
            f"{cmd.angular.z:.6f}",
        ])
        self.controller_log_file.flush()

    def log_status(self):
        if self.done:
            return

        if self.current_pose is None:
            self.get_logger().warn(
                f"Waiting for Redis stream {self.redis_pose_reader.stream_name} "
                f"node {self.redis_pose_reader.target_node}; no /cmd_vel will be published yet"
            )
            return

        x, y, yaw = self.current_pose
        target_index = self.target_index()
        target_x, target_y, _ = self.waypoint_at(target_index)
        distance = math.hypot(target_x - x, target_y - y)
        cmd_subscribers = self.cmd_pub.get_subscription_count()
        yaw_text = "unknown" if yaw is None else f"{yaw:.2f}"
        self.get_logger().info(
            "Pure pursuit status: "
            f"pose=({x:.2f}, {y:.2f}, yaw={yaw_text}), "
            f"target_index={target_index}, target=({target_x:.2f}, {target_y:.2f}), "
            f"distance={distance:.2f}, "
            f"alpha={self.last_alpha:.2f}, curvature={self.last_curvature:.2f}, "
            f"speed_scale={self.last_speed_scale:.2f}, "
            f"cmd=(linear.x={self.last_cmd.linear.x:.2f}, angular.z={self.last_cmd.angular.z:.2f}), "
            f"cmd_topic={self.cmd_vel_topic}, cmd_subscribers={cmd_subscribers}"
        )

    def destroy_node(self):
        if self.stop_on_completion:
            self.publish_stop()
        self.close_controller_log()
        super().destroy_node()

    def close_controller_log(self):
        if self.controller_log_closed:
            return

        if self.controller_log_file is not None:
            self.controller_log_file.flush()
            self.controller_log_file.close()
        self.controller_log_closed = True


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PurePursuitFollower()
    except (FileNotFoundError, ValueError) as exc:
        rclpy.logging.get_logger("pure_pursuit_follower").error(str(exc))
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
