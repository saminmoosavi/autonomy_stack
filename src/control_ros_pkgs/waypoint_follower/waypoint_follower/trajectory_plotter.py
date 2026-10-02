#!/usr/bin/env python3

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from waypoint_follower.pose_utils import quaternion_to_yaw
from waypoint_follower.redis_pose_reader import RedisPoseReader


AMCL_POSE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_plotter")

        self.localization_source = str(
            self.declare_parameter("localization_source", "redis").value
        ).lower()
        self.odom_topic = self.declare_parameter("odom_topic", "/a200_0000/odometry/filtered").value
        self.pose_topic = self.declare_parameter("pose_topic", "/a200_0000/amcl_pose").value
        self.tf_fixed_frame = self.declare_parameter("tf_fixed_frame", "map").value
        self.tf_robot_frame = self.declare_parameter("tf_robot_frame", "a200_0000/base_link").value
        self.tf_timeout_s = float(self.declare_parameter("tf_timeout_s", 0.05).value)
        self.trajectory_csv = self.declare_parameter("trajectory_csv", "trajectories/warthog_trajectory.csv").value
        self.lookahead_distance_m = float(self.declare_parameter("lookahead_distance_m", 1.5).value)
        self.closed_loop = bool(self.declare_parameter("closed_loop", False).value)
        self.duplicate_endpoint_tolerance_m = float(
            self.declare_parameter("duplicate_endpoint_tolerance_m", 0.25).value
        )
        self.search_window_points = int(self.declare_parameter("search_window_points", 80).value)
        self.plot_rate_hz = float(self.declare_parameter("plot_rate_hz", 5.0).value)
        self.trail_max_points = int(self.declare_parameter("trail_max_points", 2000).value)
        self.figure_title = self.declare_parameter("figure_title", "Warthog Pure Pursuit").value
        if self.localization_source not in ("redis", "odom", "pose", "tf"):
            raise ValueError("localization_source must be 'redis', 'odom', 'pose', or 'tf'")

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
                20,
            )
        elif self.localization_source == "pose":
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_callback,
                AMCL_POSE_QOS,
            )
        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = self.load_waypoints(self.trajectory_csv)
        self.current_pose = None
        self.actual_path = []
        self.progress_index = 0
        self.last_pose_wait_log_time = None

        self.timer = self.create_timer(1.0 / max(self.plot_rate_hz, 0.1), self.update_plot)

        self.setup_plot()
        self.get_logger().info(
            f"Plotting {len(self.waypoints)} waypoints from {self.trajectory_csv}; "
            f"{self.pose_source_text()}"
        )

    def load_waypoints(self, csv_file):
        path = self.resolve_package_path(csv_file)
        if not path.exists():
            raise FileNotFoundError(f"Trajectory CSV does not exist: {path}")

        waypoints = []
        with path.open("r", newline="") as handle:
            reader = csv.DictReader(handle)
            if not {"x", "y"}.issubset(reader.fieldnames or []):
                raise ValueError("Trajectory CSV must include x and y columns")

            for row in reader:
                try:
                    waypoints.append((float(row["x"]), float(row["y"])))
                except ValueError:
                    continue

        if len(waypoints) < 2:
            raise ValueError("Trajectory CSV must contain at least two valid waypoints")

        if self.closed_loop:
            first_x, first_y = waypoints[0]
            last_x, last_y = waypoints[-1]
            if math.hypot(last_x - first_x, last_y - first_y) <= self.duplicate_endpoint_tolerance_m:
                waypoints.pop()
        return waypoints

    @staticmethod
    def resolve_package_path(path_value):
        path = Path(path_value).expanduser()
        if path.is_absolute():
            return path

        package_share = Path(get_package_share_directory("waypoint_follower"))
        return package_share / path

    def setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title(self.figure_title)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True)
        self.ax.axis("equal")

        ref_x = [point[0] for point in self.waypoints]
        ref_y = [point[1] for point in self.waypoints]
        (self.reference_line,) = self.ax.plot(ref_x, ref_y, "k--", linewidth=1.5, label="saved trajectory")
        (self.actual_line,) = self.ax.plot([], [], "b-", linewidth=2.0, label="current path")
        (self.robot_marker,) = self.ax.plot([], [], "go", markersize=8, label="robot")
        (self.goal_marker,) = self.ax.plot([], [], "ro", markersize=8, label="next goal")
        (self.heading_line,) = self.ax.plot([], [], "g-", linewidth=2.0)
        self.ax.legend(loc="best")
        self.fig.tight_layout()
        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update_pose_from_redis(self):
        pose = self.redis_pose_reader.get_pose()
        if pose is None:
            return False

        x = pose.x
        y = pose.y
        yaw = pose.yaw
        self.current_pose = (x, y, yaw)
        self.actual_path.append((x, y))

        if len(self.actual_path) > self.trail_max_points:
            self.actual_path = self.actual_path[-self.trail_max_points:]
        return True

    def odom_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(pose.orientation)
        self.current_pose = (x, y, yaw)
        self.actual_path.append((x, y))

        if len(self.actual_path) > self.trail_max_points:
            self.actual_path = self.actual_path[-self.trail_max_points:]

    def pose_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(pose.orientation)
        self.current_pose = (x, y, yaw)
        self.actual_path.append((x, y))

        if len(self.actual_path) > self.trail_max_points:
            self.actual_path = self.actual_path[-self.trail_max_points:]

    def update_pose(self):
        if self.localization_source == "redis":
            return self.update_pose_from_redis()
        if self.localization_source == "tf":
            return self.update_pose_from_tf()
        return self.current_pose is not None

    def update_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tf_fixed_frame,
                self.tf_robot_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException:
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x = translation.x
        y = translation.y
        yaw = quaternion_to_yaw(rotation)
        self.current_pose = (x, y, yaw)
        self.actual_path.append((x, y))

        if len(self.actual_path) > self.trail_max_points:
            self.actual_path = self.actual_path[-self.trail_max_points:]
        return True

    def pose_source_text(self):
        if self.localization_source == "redis":
            return (
                f"reading Redis stream {self.redis_pose_reader.stream_name} "
                f"node {self.redis_pose_reader.target_node}"
            )
        if self.localization_source == "tf":
            return f"reading TF {self.tf_fixed_frame} -> {self.tf_robot_frame}"
        if self.localization_source == "pose":
            return f"subscribing to pose on {self.pose_topic}"
        return f"subscribing to odometry on {self.odom_topic}"

    def waypoint_at(self, absolute_index):
        if self.closed_loop:
            return self.waypoints[absolute_index % len(self.waypoints)]
        return self.waypoints[min(absolute_index, len(self.waypoints) - 1)]

    def nearest_progress_index(self, x, y):
        if self.closed_loop:
            search_start = max(0, self.progress_index - 5)
            search_stop = self.progress_index + max(self.search_window_points, 1)
        else:
            search_start = max(0, self.progress_index - 5)
            search_stop = len(self.waypoints) - 1

        return min(
            range(search_start, search_stop + 1),
            key=lambda i: math.hypot(self.waypoint_at(i)[0] - x, self.waypoint_at(i)[1] - y),
        )

    def next_goal(self, x, y):
        self.progress_index = self.nearest_progress_index(x, y)
        max_steps = len(self.waypoints) if self.closed_loop else len(self.waypoints) - self.progress_index

        for i in range(self.progress_index, self.progress_index + max_steps):
            wx, wy = self.waypoint_at(i)
            if math.hypot(wx - x, wy - y) >= self.lookahead_distance_m:
                return wx, wy
        return self.waypoint_at(self.progress_index + max_steps - 1)

    def update_plot(self):
        self.update_pose()
        if self.current_pose is None or not plt.fignum_exists(self.fig.number):
            self.log_pose_wait()
            return

        x, y, yaw = self.current_pose
        path_x = [point[0] for point in self.actual_path]
        path_y = [point[1] for point in self.actual_path]
        goal_x, goal_y = self.next_goal(x, y)

        heading_length = max(0.5, self.lookahead_distance_m * 0.5)
        heading_x = [x, x + heading_length * math.cos(yaw)]
        heading_y = [y, y + heading_length * math.sin(yaw)]

        self.actual_line.set_data(path_x, path_y)
        self.robot_marker.set_data([x], [y])
        self.goal_marker.set_data([goal_x], [goal_y])
        self.heading_line.set_data(heading_x, heading_y)

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.axis("equal")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def log_pose_wait(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pose_wait_log_time is not None and now - self.last_pose_wait_log_time < 1.0:
            return

        self.last_pose_wait_log_time = now
        self.get_logger().warn(
            f"Waiting for localization from {self.pose_source_text()}; plot will update after the first pose"
        )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrajectoryPlotter()
    except (FileNotFoundError, ValueError) as exc:
        rclpy.logging.get_logger("trajectory_plotter").error(str(exc))
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close("all")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
