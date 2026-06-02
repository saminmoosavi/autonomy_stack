#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class LioSamSensorRelay(Node):
    def __init__(self):
        super().__init__("lio_sam_sensor_relay")

        self.input_points_topic = self.declare_parameter(
            "input_points_topic", "/ouster/points"
        ).value
        self.output_points_topic = self.declare_parameter(
            "output_points_topic", "/points"
        ).value
        self.input_imu_topic = self.declare_parameter(
            "input_imu_topic", "/vectornav/imu"
        ).value
        self.output_imu_topic = self.declare_parameter(
            "output_imu_topic", "/imu/data"
        ).value

        self.lidar_frame_id = self.declare_parameter(
            "lidar_frame_id", "lidar_link"
        ).value
        self.imu_frame_id = self.declare_parameter(
            "imu_frame_id", "base_link"
        ).value
        self.stamp_with_now = bool(self.declare_parameter("stamp_with_now", True).value)
        self.stamp_points_with_now = bool(
            self.declare_parameter("stamp_points_with_now", self.stamp_with_now).value
        )
        self.stamp_imu_with_now = bool(
            self.declare_parameter("stamp_imu_with_now", self.stamp_with_now).value
        )
        self.log_stamp_offsets = bool(
            self.declare_parameter("log_stamp_offsets", False).value
        )
        self.last_points_stamp = None
        self.last_imu_stamp = None

        self.points_pub = self.create_publisher(
            PointCloud2,
            self.output_points_topic,
            qos_profile_sensor_data,
        )
        self.imu_pub = self.create_publisher(
            Imu,
            self.output_imu_topic,
            qos_profile_sensor_data,
        )

        self.points_sub = self.create_subscription(
            PointCloud2,
            self.input_points_topic,
            self.points_callback,
            qos_profile_sensor_data,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.input_imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Relaying Ouster PointCloud2 "
            f"{self.input_points_topic} -> {self.output_points_topic} "
            f"with frame_id '{self.lidar_frame_id}', "
            f"stamp_with_now={self.stamp_points_with_now}"
        )
        self.get_logger().info(
            "Relaying VectorNav Imu "
            f"{self.input_imu_topic} -> {self.output_imu_topic} "
            f"with frame_id '{self.imu_frame_id}', "
            f"stamp_with_now={self.stamp_imu_with_now}"
        )

    def points_callback(self, msg):
        if self.stamp_points_with_now:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.last_points_stamp = self.stamp_to_sec(msg.header.stamp)
        self.log_stamp_offset()
        msg.header.frame_id = self.lidar_frame_id
        self.points_pub.publish(msg)

    def imu_callback(self, msg):
        if self.stamp_imu_with_now:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.last_imu_stamp = self.stamp_to_sec(msg.header.stamp)
        self.log_stamp_offset()
        msg.header.frame_id = self.imu_frame_id
        self.imu_pub.publish(msg)

    @staticmethod
    def stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def log_stamp_offset(self):
        if (
            not self.log_stamp_offsets
            or self.last_points_stamp is None
            or self.last_imu_stamp is None
        ):
            return

        offset = self.last_imu_stamp - self.last_points_stamp
        self.get_logger().info(
            f"Latest IMU - point cloud stamp offset: {offset:.6f} s",
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init()
    node = LioSamSensorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
