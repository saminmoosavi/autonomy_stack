#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
from datetime import datetime
import math


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')

        self.declare_parameter('topic_name', '/a200_0000/platform/odom')
        self.declare_parameter('output_file', 'trajectory.csv')

        topic_name = self.get_parameter('topic_name').value
        output_file = self.get_parameter('output_file').value

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.file_path = f"{timestamp_str}_{output_file}"

        self.csv_file = open(self.file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            'time_sec',
            'x',
            'y',
            'z',
            'yaw'
        ])

        self.get_logger().info(f"Logging trajectory to: {self.file_path}")

        self.subscription = self.create_subscription(
            Odometry,
            topic_name,
            self.odom_callback,
            10
        )

        self._file_closed = False

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        self.csv_writer.writerow([t, x, y, z, yaw])
        self.csv_file.flush()

    def close_file(self):
        if not self._file_closed:
            self.csv_file.close()
            self._file_closed = True


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_file()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()