#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from yolo_msgs.msg import DetectionArray


class Nav2SemanticSafetyFilter(Node):

    def __init__(self):
        super().__init__("nav2_semantic_safety_filter")

        self.ns = self.declare_parameter("ns", "/a200_0000").value

        self.nav2_cmd_topic = self.declare_parameter(
            "nav2_cmd_topic",
            f"{self.ns}/nav2_cmd_vel"
        ).value

        self.safe_cmd_topic = self.declare_parameter(
            "safe_cmd_topic",
            f"{self.ns}/cmd_vel"
        ).value

        self.yolo_topic = self.declare_parameter(
            "yolo_topic",
            "/yolo/tracking"
        ).value

        self.closest_objects = {
            "person": 999.0,
            "box": 999.0,
            "pallet": 999.0,
            "forklift": 999.0,
        }

        self.cmd_sub = self.create_subscription(
            Twist,
            self.nav2_cmd_topic,
            self.cmd_callback,
            10
        )

        self.yolo_sub = self.create_subscription(
            DetectionArray,
            self.yolo_topic,
            self.yolo_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.safe_cmd_topic,
            10
        )

        self.get_logger().info("Semantic safety filter started")
        self.get_logger().info(f"Listening to Nav2 cmd: {self.nav2_cmd_topic}")
        self.get_logger().info(f"Publishing safe cmd: {self.safe_cmd_topic}")

    def yolo_callback(self, msg):
        self.closest_objects = {
            "person": 999.0,
            "box": 999.0,
            "pallet": 999.0,
            "forklift": 999.0,
        }

        for det in msg.detections:
            class_name = self.get_class_name(det)
            distance = self.get_distance(det)

            if class_name in self.closest_objects:
                self.closest_objects[class_name] = min(
                    self.closest_objects[class_name],
                    distance
                )

    def cmd_callback(self, nav2_cmd):
        safe_cmd = Twist()
        safe_cmd.linear.x = nav2_cmd.linear.x
        safe_cmd.angular.z = nav2_cmd.angular.z

        safe_cmd = self.apply_object_safety_rules(safe_cmd)

        self.cmd_pub.publish(safe_cmd)

    def apply_object_safety_rules(self, cmd):
        d_person = self.closest_objects["person"]
        d_box = self.closest_objects["box"]
        d_pallet = self.closest_objects["pallet"]
        d_forklift = self.closest_objects["forklift"]

        # -----------------------------
        # Person safety
        # -----------------------------
        if d_person < 1.0:
            return self.stop_cmd()

        if d_person < 2.0:
            cmd.linear.x = min(cmd.linear.x, 0.15)

        elif d_person < 3.0:
            cmd.linear.x = min(cmd.linear.x, 0.35)

        # -----------------------------
        # Forklift safety
        # -----------------------------
        if d_forklift < 2.0:
            return self.stop_cmd()

        if d_forklift < 4.0:
            cmd.linear.x = min(cmd.linear.x, 0.2)

        # -----------------------------
        # Box / pallet safety
        # -----------------------------
        if d_box < 0.4:
            return self.stop_cmd()

        if d_box < 0.8:
            cmd.linear.x = min(cmd.linear.x, 0.2)

        if d_pallet < 0.6:
            return self.stop_cmd()

        if d_pallet < 1.2:
            cmd.linear.x = min(cmd.linear.x, 0.2)

        return cmd

    def stop_cmd(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd

    def get_class_name(self, det):
        if hasattr(det, "class_name"):
            return det.class_name

        if hasattr(det, "name"):
            return det.name

        if hasattr(det, "results") and len(det.results) > 0:
            return det.results[0].hypothesis.class_id

        return "unknown"

    def get_distance(self, det):
        """
        Adapt this to your YOLO message.

        Preferred:
          det.position.x/y/z
          det.pose.pose.position.x/y/z
          det.estimated_depth

        YOLO alone only gives bounding boxes, so you need depth,
        point cloud association, or a tracker that estimates range.
        """

        if hasattr(det, "estimated_depth"):
            return float(det.estimated_depth)

        if hasattr(det, "position"):
            x = det.position.x
            y = det.position.y
            z = det.position.z
            return math.sqrt(x*x + y*y + z*z)

        if hasattr(det, "pose"):
            x = det.pose.pose.position.x
            y = det.pose.pose.position.y
            z = det.pose.pose.position.z
            return math.sqrt(x*x + y*y + z*z)

        return 999.0


def main(args=None):
    rclpy.init(args=args)
    node = Nav2SemanticSafetyFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()