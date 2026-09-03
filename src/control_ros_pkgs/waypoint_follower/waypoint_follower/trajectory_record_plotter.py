#!/usr/bin/env python3

import math

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


class TrajectoryRecordPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_record_plotter")

        self.localization_source = str(
            self.declare_parameter("localization_source", "redis").value
        ).lower()
        self.odom_topic = self.declare_parameter("odom_topic", "/a200_0000/odometry/filtered").value
        self.pose_topic = self.declare_parameter("pose_topic", "/a200_0000/amcl_pose").value
        self.tf_fixed_frame = self.declare_parameter("tf_fixed_frame", "map").value
        self.tf_robot_frame = self.declare_parameter("tf_robot_frame", "a200_0000/base_link").value
        self.tf_timeout_s = float(self.declare_parameter("tf_timeout_s", 0.05).value)
        self.min_distance_m = float(self.declare_parameter("min_distance_m", 0.25).value)
        self.plot_rate_hz = float(self.declare_parameter("record_plot_rate_hz", 5.0).value)
        self.trail_max_points = int(self.declare_parameter("record_trail_max_points", 5000).value)
        self.figure_title = self.declare_parameter("record_figure_title", "Warthog Trajectory Recording").value
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

        self.current_pose = None
        self.raw_path = []
        self.saved_path = []
        self.last_saved_pose = None
        self.last_pose_wait_log_time = None

        self.timer = self.create_timer(1.0 / max(self.plot_rate_hz, 0.1), self.update_plot)

        self.setup_plot()
        self.get_logger().info(
            f"Live plotting {self.pose_source_text()}"
        )

    def setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title(self.figure_title)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True)
        self.ax.axis("equal")

        (self.raw_line,) = self.ax.plot([], [], color="0.75", linewidth=1.0, label="pose trail")
        (self.saved_line,) = self.ax.plot([], [], "b-", linewidth=2.0, label="sampled trajectory")
        (self.saved_points,) = self.ax.plot([], [], "bo", markersize=3, label="saved points")
        (self.robot_marker,) = self.ax.plot([], [], "go", markersize=8, label="robot")
        (self.heading_line,) = self.ax.plot([], [], "g-", linewidth=2.0)
        self.ax.legend(loc="best")
        self.fig.tight_layout()
        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def should_save(self, x, y):
        if self.last_saved_pose is None:
            return True

        last_x, last_y = self.last_saved_pose
        return math.hypot(x - last_x, y - last_y) >= self.min_distance_m

    def update_pose_from_redis(self):
        pose = self.redis_pose_reader.get_pose()
        if pose is None:
            return False

        x = pose.x
        y = pose.y
        yaw = pose.yaw
        self.update_pose(x, y, yaw)
        return True

    def odom_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(pose.orientation)
        self.update_pose(x, y, yaw)

    def pose_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(pose.orientation)
        self.update_pose(x, y, yaw)

    def update_pose(self, x, y, yaw):
        self.current_pose = (x, y, yaw)

        self.raw_path.append((x, y))
        if len(self.raw_path) > self.trail_max_points:
            self.raw_path = self.raw_path[-self.trail_max_points:]

        if self.should_save(x, y):
            self.saved_path.append((x, y))
            self.last_saved_pose = (x, y)

    def refresh_pose(self):
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
        self.update_pose(
            translation.x,
            translation.y,
            quaternion_to_yaw(rotation),
        )
        return True

    def pose_source_text(self):
        if self.localization_source == "redis":
            return (
                f"Redis stream {self.redis_pose_reader.stream_name} "
                f"node {self.redis_pose_reader.target_node}"
            )
        if self.localization_source == "tf":
            return f"TF {self.tf_fixed_frame} -> {self.tf_robot_frame}"
        if self.localization_source == "pose":
            return f"pose topic {self.pose_topic}"
        return f"odometry topic {self.odom_topic}"

    def update_plot(self):
        self.refresh_pose()
        if self.current_pose is None or not plt.fignum_exists(self.fig.number):
            self.log_pose_wait()
            return

        x, y, yaw = self.current_pose
        raw_x = [point[0] for point in self.raw_path]
        raw_y = [point[1] for point in self.raw_path]
        saved_x = [point[0] for point in self.saved_path]
        saved_y = [point[1] for point in self.saved_path]

        heading_length = max(0.5, self.min_distance_m * 3.0)
        heading_x = [x, x + heading_length * math.cos(yaw)]
        heading_y = [y, y + heading_length * math.sin(yaw)]

        self.raw_line.set_data(raw_x, raw_y)
        self.saved_line.set_data(saved_x, saved_y)
        self.saved_points.set_data(saved_x, saved_y)
        self.robot_marker.set_data([x], [y])
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
    node = TrajectoryRecordPlotter()

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
