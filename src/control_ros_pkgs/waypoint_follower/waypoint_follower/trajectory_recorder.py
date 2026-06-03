#!/usr/bin/env python3

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from waypoint_follower.redis_pose_reader import RedisPoseReader


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        self.output_csv = self.declare_parameter("output_csv", "trajectories/warthog_trajectory.csv").value
        self.frame_id = self.declare_parameter("frame_id", "odom").value
        self.min_distance_m = float(self.declare_parameter("min_distance_m", 0.25).value)
        self.min_interval_s = float(self.declare_parameter("min_interval_s", 0.1).value)
        self.flush_every_n = int(self.declare_parameter("flush_every_n", 3).value)
        self.redis_poll_rate_hz = float(self.declare_parameter("redis_poll_rate_hz", 20.0).value)
        self.redis_pose_reader = RedisPoseReader(self)

        self.output_path = self.resolve_package_path(self.output_csv)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        self.csv_file = self.output_path.open("w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time_sec", "frame_id", "x", "y", "z", "yaw"])

        self.last_saved_pose = None
        self.last_saved_time = None
        self.samples_written = 0
        self.closed = False

        self.timer = self.create_timer(
            1.0 / max(self.redis_poll_rate_hz, 0.1),
            self.poll_redis_pose,
        )

        self.get_logger().info(
            f"Recording Redis stream {self.redis_pose_reader.stream_name} "
            f"node {self.redis_pose_reader.target_node} to {self.output_path}"
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
