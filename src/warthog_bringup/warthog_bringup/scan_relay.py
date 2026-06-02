#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__("scan_relay")

        self.input_topic = self.declare_parameter("input_topic", "/ouster/scan").value
        self.output_topic = self.declare_parameter("output_topic", "/w200_0105/sensors/lidar2d_0/scan").value

        self.pub = self.create_publisher(LaserScan, self.output_topic, 10)
        self.sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Relaying LaserScan from {self.input_topic} to {self.output_topic}"
        )

    def scan_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar2d_0_laser"
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
