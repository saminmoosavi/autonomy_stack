#!/usr/bin/env python3

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from waypoint_follower.pose_utils import quaternion_to_yaw, stamp_to_sec
from waypoint_follower.redis_pose_reader import RedisPoseReader


AMCL_POSE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        self.localization_source = str(
            self.declare_parameter("localization_source", "redis").value
        ).lower()
        self.odom_topic = self.declare_parameter("odom_topic", "/a200_0000/odometry/filtered").value
        self.pose_topic = self.declare_parameter("pose_topic", "/a200_0000/amcl_pose").value
        self.tf_fixed_frame = self.declare_parameter("tf_fixed_frame", "map").value
        self.tf_robot_frame = self.declare_parameter("tf_robot_frame", "a200_0000/base_link").value
        self.tf_timeout_s = float(self.declare_parameter("tf_timeout_s", 0.05).value)
        self.output_csv = self.declare_parameter("output_csv", "trajectories/warthog_trajectory.csv").value
        self.frame_id = self.declare_parameter("frame_id", "odom").value
        self.min_distance_m = float(self.declare_parameter("min_distance_m", 0.25).value)
        self.min_interval_s = float(self.declare_parameter("min_interval_s", 0.1).value)
        self.flush_every_n = int(self.declare_parameter("flush_every_n", 3).value)
        self.redis_poll_rate_hz = float(self.declare_parameter("redis_poll_rate_hz", 20.0).value)
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

        self.output_path = self.resolve_package_path(self.output_csv)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.csv_file = self.output_path.open("w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time_sec", "frame_id", "x", "y", "z", "yaw"])

        self.last_saved_pose = None
        self.last_saved_time = None
        self.samples_written = 0
        self.closed = False

        self.timer = None
        if self.localization_source == "redis":
            self.timer = self.create_timer(
                1.0 / max(self.redis_poll_rate_hz, 0.1),
                self.poll_redis_pose,
            )
        elif self.localization_source == "tf":
            self.timer = self.create_timer(
                1.0 / max(self.redis_poll_rate_hz, 0.1),
                self.poll_tf_pose,
            )

        self.get_logger().info(
            f"Recording {self.pose_source_text()} to {self.output_path}"
        )

    @staticmethod
    def resolve_package_path(path_value):
        path = Path(path_value).expanduser()
        if path.is_absolute():
            return path

        package_share = Path(get_package_share_directory("waypoint_follower"))
        return package_share / path

    def should_save(self, t, x, y):
        if self.last_saved_pose is None:
            return True

        last_x, last_y = self.last_saved_pose
        distance = math.hypot(x - last_x, y - last_y)
        elapsed = t - self.last_saved_time if self.last_saved_time is not None else math.inf
        return distance >= self.min_distance_m and elapsed >= self.min_interval_s

    def poll_redis_pose(self):
        pose = self.redis_pose_reader.get_pose()
        if pose is None:
            return

        t = pose.time_sec
        x = pose.x
        y = pose.y
        z = pose.z
        yaw = pose.yaw
        frame_id = pose.frame_id or self.frame_id

        self.write_pose(t, frame_id, x, y, z, yaw)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        yaw = quaternion_to_yaw(pose.orientation)
        frame_id = msg.header.frame_id or self.frame_id

        self.write_pose(t, frame_id, x, y, z, yaw)

    def pose_callback(self, msg):
        pose = msg.pose.pose
        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        yaw = quaternion_to_yaw(pose.orientation)
        frame_id = msg.header.frame_id or self.frame_id

        self.write_pose(t, frame_id, x, y, z, yaw)

    def poll_tf_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tf_fixed_frame,
                self.tf_robot_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        t = stamp_to_sec(transform.header.stamp)
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        self.write_pose(
            t,
            transform.header.frame_id or self.frame_id,
            translation.x,
            translation.y,
            translation.z,
            quaternion_to_yaw(rotation),
        )

    def write_pose(self, t, frame_id, x, y, z, yaw):
        if not self.should_save(t, x, y):
            return

        if yaw is None:
            yaw = 0.0
        self.writer.writerow([f"{t:.9f}", frame_id, f"{x:.6f}", f"{y:.6f}", f"{z:.6f}", f"{yaw:.6f}"])
        self.last_saved_pose = (x, y)
        self.last_saved_time = t
        self.samples_written += 1

        if self.samples_written % self.flush_every_n == 0:
            self.csv_file.flush()

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

    def close(self):
        if self.closed:
            return

        self.csv_file.flush()
        self.csv_file.close()
        self.closed = True
        self.get_logger().info(f"Saved {self.samples_written} trajectory samples to {self.output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
