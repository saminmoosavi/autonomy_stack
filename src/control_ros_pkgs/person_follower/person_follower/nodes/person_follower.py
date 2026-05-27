#!/usr/bin/env python3

"""
person_follower_nav2.py

ROS 2 node that follows a detected person using:
- YOLO bounding boxes
- Depth image
- Nav2 NavigateToPose action

Assumptions:
1. YOLO publishes detections containing:
   - class label/name
   - bounding box center (u, v) in image pixels
2. Depth image is aligned with the RGB image used by YOLO.
3. TF is available for:
   map <- base_link
   map <- camera frame
4. Nav2 is running and accepting goals on /navigate_to_pose.

You may need to adapt the detection parsing section to your exact YOLO message type.
Tested as a template for ROS 2 Humble-style APIs.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped, Twist

from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs
# from tf_transformations import quaternion_from_euler
# Change this import if your YOLO package uses a different message type.
# Common examples:
#   from yolo_msgs.msg import DetectionArray
#   from vision_msgs.msg import Detection2DArray
#
# This script assumes a message similar to yolo_msgs/DetectionArray.
from yolo_msgs.msg import DetectionArray


@dataclass
class PersonDetection:
    """Simple container for one person detection."""
    u: float
    v: float
    score: float


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


class PersonFollowerNav2(Node):
    def __init__(self) -> None:
        super().__init__("person_follower")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("namespace", "/a200_0000")
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("points_topic", "/sensors/camera_0/points")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("follow_distance", 1.5)      # meters
        self.declare_parameter("max_person_distance", 20.0)  # meters
        self.declare_parameter("min_person_distance", 0.6)  # meters
        self.declare_parameter("goal_update_period", 1.0)   # seconds
        self.declare_parameter("goal_pos_tolerance", 0.35)  # meters
        self.declare_parameter("goal_yaw_tolerance", 0.35)  # rad
        self.declare_parameter("depth_window", 5)           # odd window size around bbox center
        self.declare_parameter("person_label", "person")
        self.declare_parameter("search_angular_speed", 0.4)  # rad/s
        self.declare_parameter("search_duration", 5.0)       # seconds
        
        self.namespace = str(self.get_parameter("namespace").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.points_topic = str(self.get_parameter("points_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.cmd_vel_topic = (f"/{self.namespace.strip('/')}/{str(self.get_parameter('cmd_vel_topic').value).strip('/')}")

        self.follow_distance = float(self.get_parameter("follow_distance").value)
        self.max_person_distance = float(self.get_parameter("max_person_distance").value)
        self.min_person_distance = float(self.get_parameter("min_person_distance").value)
        self.goal_update_period = float(self.get_parameter("goal_update_period").value)
        self.goal_pos_tolerance = float(self.get_parameter("goal_pos_tolerance").value)
        self.goal_yaw_tolerance = float(self.get_parameter("goal_yaw_tolerance").value)
        self.depth_window = int(self.get_parameter("depth_window").value)
        self.person_label = str(self.get_parameter("person_label").value)
        self.search_angular_speed = float(self.get_parameter("search_angular_speed").value)
        self.search_duration = float(self.get_parameter("search_duration").value)

        if self.depth_window % 2 == 0:
            self.depth_window += 1

        # -----------------------------
        # State
        # -----------------------------
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f"/{self.namespace.strip('/')}/navigate_to_pose",
        )

        self.latest_cloud: Optional[PointCloud2] = None

        self.latest_persons: List[PersonDetection] = []

        self.last_goal_time = self.get_clock().now()
        self.last_goal_pose: Optional[Tuple[float, float, float]] = None

        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.search_start_time = None
        self.is_searching = False

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(
            DetectionArray,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        self.create_subscription(
            PointCloud2,
            self.points_topic,
            self.points_callback,
            10,
        )

        # Main timer
        self.create_timer(0.2, self.control_loop)

        self.get_logger().info("PersonFollowerNav2 started.")
        self.get_logger().info(f"detections_topic: {self.detections_topic}")
        self.get_logger().info(f"points_topic: {self.points_topic}")
        self.get_logger().info(f"cmd_vel_topic: {self.cmd_vel_topic}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def points_callback(self, msg: PointCloud2) -> None:
        self.latest_cloud = msg

    def detections_callback(self, msg: DetectionArray) -> None:
        """
        Parse YOLO detections and keep only person detections.

        IMPORTANT:
        This parser assumes a yolo_msgs-style message where each detection has:
          - class_name
          - score
          - bbox.center.position.x
          - bbox.center.position.y

        If your message differs, adapt this function.
        """

        persons: List[PersonDetection] = []

        for det in msg.detections:
            
            # Adjust these fields if your message structure differs.
            label = det.class_name
            score = float(det.score)
            u = float(det.bbox.center.position.x)
            v = float(det.bbox.center.position.y)

            if label == self.person_label:
                persons.append(PersonDetection(u=u, v=v, score=score))
        self.get_logger().info(f"latest person is at {persons}")
        self.latest_persons = persons
        

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self) -> None:
        if self.latest_cloud is None:
            return
 
        person = self.select_best_person(self.latest_persons)
        if person is None:
            self.search_for_person()
            return

        self.stop_search_rotation()
        # self.get_logger().info(f"person is at {person}")
        person_3d_cam = self.pointcloud_to_3d(person.u, person.v)
        if person_3d_cam is None:
            self.get_logger().debug("Could not compute 3D point from point cloud.")
            return
        # self.get_logger().info(f"person 3d is at {person_3d_cam}")
        point_map = self.transform_point_to_map(person_3d_cam)
        if point_map is None:
            return
        # self.get_logger().info(f"point map is at {point_map}")

        robot_pose = self.get_robot_pose_in_map()
        if robot_pose is None:
            return
        self.get_logger().info(f"robot pose is at {robot_pose}")

        goal = self.compute_follow_goal(robot_pose, point_map)
        if goal is None:
            return
        self.get_logger().info(f"goal is at {goal}")

        gx, gy, gyaw = goal

        now = self.get_clock().now()
        if (now - self.last_goal_time).nanoseconds * 1e-9 < self.goal_update_period:
            self.get_logger().info("checking time")
            return

        if not self.should_send_new_goal(gx, gy, gyaw):
            self.get_logger().info("sending new goal")
            return
        self.get_logger().info("I am here!")
        self.send_nav2_goal(gx, gy, gyaw)
        self.last_goal_time = now
        self.last_goal_pose = (gx, gy, gyaw)

    # ------------------------------------------------------------------
    # Person selection / geometry
    # ------------------------------------------------------------------
    def select_best_person(self, persons: List[PersonDetection]) -> Optional[PersonDetection]:
        """
        Pick the highest-confidence person whose point cloud sample is valid.
        You could replace this with tracking logic later.
        """
        best = None
        best_score = -1.0

        for p in persons:
            point = self.pointcloud_to_3d(p.u, p.v)
            if point is None:
                continue
            depth = math.sqrt(sum(coord * coord for coord in point))
            if not (self.min_person_distance <= depth <= self.max_person_distance):
                continue
            if p.score > best_score:
                best_score = p.score
                best = p
        return best

    def pointcloud_to_3d(self, u: float, v: float) -> Optional[Tuple[float, float, float]]:
        """
        Sample a 3D point from an organized point cloud at the detection center.
        """
        if self.latest_cloud is None:
            return None

        cloud = self.latest_cloud
        width = int(cloud.width)
        height = int(cloud.height)
        if height <= 1:
            return None

        ui = clamp(int(round(u)), 0, width - 1)
        vi = clamp(int(round(v)), 0, height - 1)
        half = self.depth_window // 2
        u0 = max(0, ui - half)
        u1 = min(width, ui + half + 1)
        v0 = max(0, vi - half)
        v1 = min(height, vi + half + 1)

        try:
            dtype = pc2.dtype_from_fields(cloud.fields, point_step=cloud.point_step)
            arr = np.frombuffer(cloud.data, dtype=dtype)
            if cloud.is_bigendian:
                arr = arr.byteswap().newbyteorder()
            arr = arr.reshape((height, width))
        except Exception as exc:
            self.get_logger().warn(f"Failed to read PointCloud2: {exc}")
            return None

        if "x" not in arr.dtype.names or "y" not in arr.dtype.names or "z" not in arr.dtype.names:
            return None

        patch_x = arr["x"][v0:v1, u0:u1].astype(np.float32)
        patch_y = arr["y"][v0:v1, u0:u1].astype(np.float32)
        patch_z = arr["z"][v0:v1, u0:u1].astype(np.float32)

        valid = (
            np.isfinite(patch_x)
            & np.isfinite(patch_y)
            & np.isfinite(patch_z)
        )
        if not np.any(valid):
            return None

        x = float(np.median(patch_x[valid]))
        y = float(np.median(patch_y[valid]))
        z = float(np.median(patch_z[valid]))

        point_range = math.sqrt(x * x + y * y + z * z)
        if not (self.min_person_distance <= point_range <= self.max_person_distance):
            return None

        return (x, y, z)

    def transform_point_to_map(
        self,
        point_cam: Tuple[float, float, float]
    ) -> Optional[Tuple[float, float]]:
        """
        Transform 3D point from camera frame to map frame.
        """

        # self.get_logger().info(f"inside transfrom point to map")
        ps = PointStamped()
        ps.header.frame_id = self.camera_frame

        ps.header.stamp = Time().to_msg()
        ps.point.x = float(point_cam[0])
        ps.point.y = float(point_cam[1])
        ps.point.z = float(point_cam[2])
        try:

            out = self.tf_buffer.transform(
                ps,
                self.map_frame,
                timeout=Duration(seconds=1),
            )
            # self.get_logger().info(f"tried tf transform {out}")
        except TransformException as exc:
            self.get_logger().debug(f"TF transform to map failed: {exc}")
            return None
        self.get_logger().info(f"transform to  map {(float(out.point.x), float(out.point.y))}")
        return (float(out.point.x), float(out.point.y))

    def get_robot_pose_in_map(self) -> Optional[Tuple[float, float, float]]:
        """
        Returns robot pose in map frame as (x, y, yaw).
        """
        self.get_logger().info("get_robot_pose")
        timeout = Duration(seconds=3)
        try:

            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=timeout
            )

    
        except TransformException as exc:
            self.get_logger().debug(f"TF transform to map failed: {exc}")
            # self._warn_once(f"TF lookup for robot pose failed: {exc}")
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        q = tf.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        # self.get_logger().info(f"get_robot_pose {(float(tx), float(ty), float(yaw))}")
        return (float(tx), float(ty), float(yaw))



    def compute_follow_goal(
        self,
        robot_pose: Tuple[float, float, float],
        person_map: Tuple[float, float]
    ) -> Optional[Tuple[float, float, float]]:
        """
        Compute a standoff goal behind the person along the robot->person line.
        Robot will face the person.
        """
        rx, ry, _ = robot_pose
        px, py = person_map

        dx = px - rx
        dy = py - ry
        dist = math.hypot(dx, dy)

        if dist < 1e-3:
            return None

        # Keep a standoff distance from the person
        goal_dist = max(0.0, dist - self.follow_distance)
        if goal_dist < 0.1:
            return None

        ux = dx / dist
        uy = dy / dist

        gx = rx + goal_dist * ux
        gy = ry + goal_dist * uy

        # Face toward the person
        gyaw = math.atan2(py - gy, px - gx)
        self.get_logger().info(f"person is at {(gx, gy, gyaw)}")

        return (gx, gy, gyaw)

    def should_send_new_goal(self, gx: float, gy: float, gyaw: float) -> bool:
        self.get_logger().info(f"last gaol is {self.last_goal_pose}")
        if self.last_goal_pose is None:
            return True

        lx, ly, lyaw = self.last_goal_pose
        pos_err = math.hypot(gx - lx, gy - ly)
        yaw_err = abs(self.normalize_angle(gyaw - lyaw))

        return (pos_err > self.goal_pos_tolerance) or (yaw_err > self.goal_yaw_tolerance)

    def search_for_person(self) -> None:
        now = self.get_clock().now()

        if not self.is_searching:
            self.is_searching = True
            self.search_start_time = now
            self.cancel_active_goal()
            self.get_logger().info("No valid person found. Rotating to search.")

        elapsed = (now - self.search_start_time).nanoseconds * 1e-9
        if elapsed >= self.search_duration:
            self.stop_search_rotation()
            self.get_logger().info("Search timeout reached without finding a person.")
            return

        twist = Twist()
        twist.angular.z = self.search_angular_speed
        self.cmd_vel_pub.publish(twist)

    def stop_search_rotation(self) -> None:
        if not self.is_searching:
            return

        self.is_searching = False
        self.search_start_time = None
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Stopping search rotation.")

    def cancel_active_goal(self) -> None:
        with self.goal_lock:
            if self.goal_handle is None:
                return

            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_cb)
            self.goal_handle = None

    # ------------------------------------------------------------------
    # Nav2 action
    # ------------------------------------------------------------------
    def send_nav2_goal(self, x: float, y: float, yaw: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn("NavigateToPose action server is not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(x, y, yaw, self.map_frame)

        with self.goal_lock:
            # Cancel previous goal before sending a new one
            if self.goal_handle is not None:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_cb)

            send_future = self.nav_client.send_goal_async(goal_msg)
            send_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(
            f"Sent follow goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    def goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send Nav2 goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            return

        with self.goal_lock:
            self.goal_handle = goal_handle

        self.get_logger().debug("Nav2 goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future) -> None:
        try:
            result = future.result()
            status = result.status
            self.get_logger().debug(f"Nav2 result received with status: {status}")
        except Exception as exc:
            self.get_logger().debug(f"Error reading Nav2 result: {exc}")

    def _cancel_done_cb(self, future) -> None:
        try:
            _ = future.result()
            self.get_logger().debug("Previous Nav2 goal canceled.")
        except Exception as exc:
            self.get_logger().debug(f"Failed to cancel previous goal: {exc}")


    # ------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------
    def make_pose_stamped(self, x: float, y: float, yaw: float, frame_id: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose
    
    @staticmethod
    def quaternion_from_euler(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw
    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonFollowerNav2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
