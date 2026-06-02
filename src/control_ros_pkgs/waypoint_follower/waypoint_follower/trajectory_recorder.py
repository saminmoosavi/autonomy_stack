#!/usr/bin/env python3

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from waypoint_follower.pose_utils import (
    OdomVectornavHeadingLocalizer,
    VectornavEcefLocalizer,
    quaternion_to_yaw,
    stamp_to_sec,
)


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        self.pose_source_type = self.declare_parameter("pose_source_type", "odom").value
        self.odom_topic = self.declare_parameter("odom_topic", "/warthog/localization/odom").value
        self.vectornav_topic = self.declare_parameter("vectornav_topic", "/vectornav/pose").value
        self.output_csv = self.declare_parameter("output_csv", "trajectories/warthog_trajectory.csv").value
        self.frame_id = self.declare_parameter("frame_id", "odom").value
        self.min_distance_m = float(self.declare_parameter("min_distance_m", 0.25).value)
        self.min_interval_s = float(self.declare_parameter("min_interval_s", 0.1).value)
        self.flush_every_n = int(self.declare_parameter("flush_every_n", 3).value)

        self.output_path = self.resolve_package_path(self.output_csv)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.csv_file = self.output_path.open("w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time_sec", "frame_id", "x", "y", "z", "yaw"])

        self.last_saved_pose = None
        self.last_saved_time = None
        self.samples_written = 0
        self.closed = False
        self.vectornav_localizer = VectornavEcefLocalizer()
        self.hybrid_localizer = OdomVectornavHeadingLocalizer()
        self.vectornav_yaw = None

        msg_type = PoseWithCovarianceStamped if self.pose_source_type == "vectornav_ecef" else Odometry
        self.sub = self.create_subscription(msg_type, self.odom_topic, self.odom_callback, 20)
        self.vectornav_sub = None
        if self.pose_source_type == "odom_vectornav_heading":
            self.vectornav_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.vectornav_topic,
                self.vectornav_callback,
                20,
            )

        self.get_logger().info(
            f"Recording odometry from {self.odom_topic} to {self.output_path}"
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

    def odom_callback(self, msg):
        stamp = msg.header.stamp
        t = stamp_to_sec(stamp)
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        if self.pose_source_type == "vectornav_ecef":
            x, y, z, yaw = self.vectornav_localizer.local_pose(msg)
        elif self.pose_source_type == "odom_vectornav_heading":
            x, y, z, yaw = self.hybrid_localizer.local_pose(msg, self.vectornav_yaw)
        else:
            pose = msg.pose.pose
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            yaw = quaternion_to_yaw(pose.orientation)
        frame_id = msg.header.frame_id or self.frame_id

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

    def vectornav_callback(self, msg):
        _, _, _, yaw = self.vectornav_localizer.local_pose(msg)
        if yaw is not None:
            self.vectornav_yaw = yaw

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
