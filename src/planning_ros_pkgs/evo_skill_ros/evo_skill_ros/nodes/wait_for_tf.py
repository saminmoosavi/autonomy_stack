#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2

import tf2_ros


class WaitForPointCloudTf(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_pointcloud_tf")

        self.declare_parameter("namespace", "/a200_0000")
        self.declare_parameter("points_topic", "/sensors/camera_0/points")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("poll_period_s", 0.25)
        self.declare_parameter("warn_after_s", 10.0)

        ns = str(self.get_parameter("namespace").value).rstrip("/")
        points_topic = str(self.get_parameter("points_topic").value)
        self.points_topic = ns + points_topic if points_topic.startswith("/") else f"{ns}/{points_topic}"
        self.target_frame = str(self.get_parameter("target_frame").value).strip() or "map"
        poll_period_s = max(0.05, float(self.get_parameter("poll_period_s").value))
        self.warn_after_s = max(0.0, float(self.get_parameter("warn_after_s").value))

        self.source_frame = ""
        self.done = False
        self.started_at = self.get_clock().now()
        self.logged_source_frame = False
        self.logged_wait_warning = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(PointCloud2, self.points_topic, self.pointcloud_callback, 10)
        self.create_timer(poll_period_s, self.check_tf)

        self.get_logger().info(
            f"Waiting for PointCloud2 on {self.points_topic} and TF to {self.target_frame}"
        )

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        if not msg.header.frame_id:
            return
        self.source_frame = msg.header.frame_id
        if not self.logged_source_frame:
            self.logged_source_frame = True
            self.get_logger().info(f"Point cloud source frame is {self.source_frame}")

    def check_tf(self) -> None:
        if self.done:
            return

        if self.warn_after_s > 0.0 and not self.logged_wait_warning:
            elapsed_s = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
            if elapsed_s > self.warn_after_s:
                self.logged_wait_warning = True
                self.get_logger().warn(
                    f"Still waiting for TF {self.source_frame or '<unknown>'} -> {self.target_frame}"
                )

        if not self.source_frame:
            return

        if self.source_frame == self.target_frame:
            self.done = True
            self.get_logger().info(
                f"Point cloud is already in {self.target_frame}; continuing launch"
            )
            return

        if self.tf_buffer.can_transform(self.target_frame, self.source_frame, Time()):
            self.done = True
            self.get_logger().info(
                f"TF available: {self.source_frame} -> {self.target_frame}; continuing launch"
            )


def main() -> None:
    rclpy.init()
    node = WaitForPointCloudTf()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
