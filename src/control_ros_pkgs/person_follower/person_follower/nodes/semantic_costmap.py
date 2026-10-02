#!/usr/bin/env python3

'''
python3 semantic_costmap_layer.py --ros-args \
  -p map_topic:=/map \
  -p yolo_topic:=/yolo/tracking \
  -p output_topic:=/semantic_costmap
'''

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

# Change this import to match your YOLO message type
# Example:
# from yolo_msgs.msg import DetectionArray
from yolo_msgs.msg import DetectionArray


class SemanticCostmapLayer(Node):

    def __init__(self):
        super().__init__("semantic_costmap_layer")

        self.map_topic = self.declare_parameter(
            "map_topic", "/map"
        ).value

        self.yolo_topic = self.declare_parameter(
            "yolo_topic", "/yolo/tracking"
        ).value

        self.output_topic = self.declare_parameter(
            "output_topic", "/semantic_costmap"
        ).value

        self.semantic_decay_time = self.declare_parameter(
            "semantic_decay_time", 3.0
        ).value

        self.person_cost = self.declare_parameter(
            "person_cost", 100
        ).value

        self.box_cost = self.declare_parameter(
            "box_cost", 80
        ).value

        self.inflation_radius = self.declare_parameter(
            "inflation_radius", 0.8
        ).value

        self.map_msg = None
        self.semantic_grid = None
        self.semantic_timestamps = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.yolo_sub = self.create_subscription(
            DetectionArray,
            self.yolo_topic,
            self.yolo_callback,
            10
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            self.output_topic,
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_semantic_layer
        )

        self.get_logger().info("Semantic costmap layer node started")

    # ---------------------------------------------------------
    # Map callback
    # ---------------------------------------------------------

    def map_callback(self, msg):
        self.map_msg = msg

        width = msg.info.width
        height = msg.info.height

        if self.semantic_grid is None:
            self.semantic_grid = np.zeros((height, width), dtype=np.int8)
            self.semantic_timestamps = np.zeros((height, width), dtype=np.float64)

            self.get_logger().info(
                f"Initialized semantic grid: {width} x {height}"
            )

    # ---------------------------------------------------------
    # YOLO callback
    # ---------------------------------------------------------

    def yolo_callback(self, msg):
        if self.map_msg is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        for det in msg.detections:

            class_name = self.get_detection_class(det)

            if class_name not in ["person", "box", "pallet"]:
                continue

            point_map = self.extract_detection_position(det, msg.header.frame_id)

            if point_map is None:
                continue

            mx, my = self.world_to_map(
                point_map.point.x,
                point_map.point.y
            )

            if mx is None:
                continue

            if class_name == "person":
                cost = self.person_cost
                radius = self.inflation_radius

            elif class_name in ["box", "pallet"]:
                cost = self.box_cost
                radius = 0.5

            else:
                cost = 50
                radius = 0.3

            self.mark_inflated_region(mx, my, radius, cost, now)

    # ---------------------------------------------------------
    # Extract class name
    # ---------------------------------------------------------

    def get_detection_class(self, det):
        """
        Adapt this to your YOLO message.

        Common possibilities:
            det.class_name
            det.results[0].hypothesis.class_id
            det.id
        """

        if hasattr(det, "class_name"):
            return det.class_name

        if hasattr(det, "name"):
            return det.name

        if hasattr(det, "results"):
            if len(det.results) > 0:
                return det.results[0].hypothesis.class_id

        return "unknown"

    # ---------------------------------------------------------
    # Extract 3D detection position
    # ---------------------------------------------------------

    def extract_detection_position(self, det, source_frame):
        """
        This function must return the detection position in map frame.

        YOLO alone only gives image-space bounding boxes.
        To place objects on a costmap, you need one of these:

          1. YOLO + depth camera
          2. YOLO + point cloud
          3. YOLO message already includes 3D position
          4. Another tracker publishes object pose

        This example assumes your detection has:
            det.pose.pose.position
        or
            det.position
        """

        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = rclpy.time.Time().to_msg()

        if hasattr(det, "pose"):
            point.point.x = det.pose.pose.position.x
            point.point.y = det.pose.pose.position.y
            point.point.z = det.pose.pose.position.z

        elif hasattr(det, "position"):
            point.point.x = det.position.x
            point.point.y = det.position.y
            point.point.z = det.position.z

        else:
            self.get_logger().warn(
                "YOLO detection has no 3D position. "
                "Add depth projection or object tracking."
            )
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                point.header.frame_id,
                rclpy.time.Time()
            )

            point_map = do_transform_point(point, transform)
            return point_map

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

    # ---------------------------------------------------------
    # Convert world coordinate to map cell
    # ---------------------------------------------------------

    def world_to_map(self, x, y):
        origin = self.map_msg.info.origin.position
        resolution = self.map_msg.info.resolution
        width = self.map_msg.info.width
        height = self.map_msg.info.height

        mx = int((x - origin.x) / resolution)
        my = int((y - origin.y) / resolution)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None, None

        return mx, my

    # ---------------------------------------------------------
    # Mark inflated semantic region
    # ---------------------------------------------------------

    def mark_inflated_region(self, mx, my, radius_m, cost, timestamp):
        resolution = self.map_msg.info.resolution
        radius_cells = int(radius_m / resolution)

        height, width = self.semantic_grid.shape

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):

                x = mx + dx
                y = my + dy

                if x < 0 or y < 0 or x >= width or y >= height:
                    continue

                dist = math.sqrt(dx * dx + dy * dy) * resolution

                if dist > radius_m:
                    continue

                inflated_cost = int(cost * (1.0 - dist / radius_m))
                inflated_cost = max(inflated_cost, 1)

                self.semantic_grid[y, x] = max(
                    self.semantic_grid[y, x],
                    inflated_cost
                )

                self.semantic_timestamps[y, x] = timestamp

    # ---------------------------------------------------------
    # Publish semantic layer
    # ---------------------------------------------------------

    def publish_semantic_layer(self):
        if self.map_msg is None or self.semantic_grid is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        expired = (
            now - self.semantic_timestamps
        ) > self.semantic_decay_time

        self.semantic_grid[expired] = 0

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        out.info = self.map_msg.info

        out.data = self.semantic_grid.flatten().astype(np.int8).tolist()

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = SemanticCostmapLayer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()