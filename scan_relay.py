#!/usr/bin/env python3
"""Minimal relay: republish a LaserScan from one topic onto another.

Used in sim to feed the Jackal's 3D-lidar flattened scan (lidar3d_0/scan)
into the lidar2d_0/scan topic that clearpath SLAM / Nav2 subscribe to.
Usage: scan_relay.py <in_topic> <out_topic>
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    in_topic, out_topic = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = Node("scan_relay")
    pub = node.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
    node.create_subscription(LaserScan, in_topic, lambda m: pub.publish(m),
                             qos_profile_sensor_data)
    node.get_logger().info(f"Relaying {in_topic} -> {out_topic}")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
