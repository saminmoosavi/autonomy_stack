#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from sensor_msgs.msg import NavSatStatus
from tf2_ros import TransformBroadcaster


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = 0.5 * yaw
    z = math.sin(half_yaw)
    w = math.cos(half_yaw)
    return (0.0, 0.0, z, w)


class GpsImuEkfLocalizer(Node):
    def __init__(self):
        super().__init__("gps_imu_ekf_localizer")

        self.gps_topic = self.declare_parameter("gps_topic", "/vectornav/gnss").value
        self.imu_topic = self.declare_parameter("imu_topic", "/vectornav/imu").value
        self.odom_topic = self.declare_parameter("odom_topic", "/warthog/localization/odom").value
        self.frame_id = self.declare_parameter("frame_id", "odom").value
        self.child_frame_id = self.declare_parameter("child_frame_id", "base_link").value
        self.publish_tf = bool(self.declare_parameter("publish_tf", False).value)

        self.gps_xy_std = float(self.declare_parameter("gps_xy_std", 1.5).value)
        self.imu_yaw_std = float(self.declare_parameter("imu_yaw_std", 0.05).value)
        self.accel_std = float(self.declare_parameter("accel_std", 0.8).value)
        self.velocity_process_std = float(
            self.declare_parameter("velocity_process_std", 0.5).value
        )
        self.max_dt = float(self.declare_parameter("max_dt", 0.25).value)
        fixed_origin_lat = float(self.declare_parameter("origin_lat", math.nan).value)
        fixed_origin_lon = float(self.declare_parameter("origin_lon", math.nan).value)
        fixed_origin_alt = float(self.declare_parameter("origin_alt", 0.0).value)

        self.origin_lat = None if math.isnan(fixed_origin_lat) else fixed_origin_lat
        self.origin_lon = None if math.isnan(fixed_origin_lon) else fixed_origin_lon
        self.origin_alt = fixed_origin_alt
        self.fixed_origin = self.origin_lat is not None and self.origin_lon is not None
        self.last_imu_time = None
        self.initialized = False

        self.x = np.zeros((5, 1), dtype=float)  # x, y, vx, vy, yaw
        self.p = np.diag([25.0, 25.0, 4.0, 4.0, 1.0])

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 20)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 100)

        self.get_logger().info(
            f"GPS+IMU EKF publishing {self.odom_topic}; "
            f"GPS={self.gps_topic}, IMU={self.imu_topic}"
        )
        if self.fixed_origin:
            self.get_logger().info(
                f"Using fixed GPS origin lat={self.origin_lat:.8f}, "
                f"lon={self.origin_lon:.8f}, alt={self.origin_alt:.3f}"
            )

    @staticmethod
    def stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def gps_to_local_xy(self, latitude, longitude):
        earth_radius_m = 6378137.0
        lat0 = math.radians(self.origin_lat)
        d_lat = math.radians(latitude - self.origin_lat)
        d_lon = math.radians(longitude - self.origin_lon)
        x = earth_radius_m * d_lon * math.cos(lat0)
        y = earth_radius_m * d_lat
        return x, y

    def gps_callback(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude if not math.isnan(msg.altitude) else 0.0
            self.get_logger().info(
                f"Set GPS origin lat={self.origin_lat:.8f}, lon={self.origin_lon:.8f}"
            )

        gps_x, gps_y = self.gps_to_local_xy(msg.latitude, msg.longitude)
        self.update_gps(gps_x, gps_y, msg)

        if self.initialized:
            self.publish_odom(msg.header.stamp)

    def imu_callback(self, msg):
        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        imu_yaw = yaw_from_quaternion(msg.orientation)

        if not self.initialized:
            self.x[4, 0] = imu_yaw
            self.initialized = self.origin_lat is not None
            self.last_imu_time = stamp_sec
            if self.initialized:
                self.publish_odom(msg.header.stamp)
            return

        dt = stamp_sec - self.last_imu_time if self.last_imu_time is not None else 0.0
        self.last_imu_time = stamp_sec
        if dt <= 0.0:
            dt = 0.0
        dt = min(dt, self.max_dt)

        self.predict(msg, dt)
        self.update_yaw(imu_yaw)
        self.publish_odom(msg.header.stamp)

    def predict(self, imu_msg, dt):
        yaw = self.x[4, 0]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        ax_body = imu_msg.linear_acceleration.x
        ay_body = imu_msg.linear_acceleration.y
        ax_world = cos_yaw * ax_body - sin_yaw * ay_body
        ay_world = sin_yaw * ax_body + cos_yaw * ay_body

        self.x[0, 0] += self.x[2, 0] * dt + 0.5 * ax_world * dt * dt
        self.x[1, 0] += self.x[3, 0] * dt + 0.5 * ay_world * dt * dt
        self.x[2, 0] += ax_world * dt
        self.x[3, 0] += ay_world * dt
        self.x[4, 0] = normalize_angle(self.x[4, 0] + imu_msg.angular_velocity.z * dt)

        f = np.eye(5)
        f[0, 2] = dt
        f[1, 3] = dt

        accel_var = self.accel_std * self.accel_std
        vel_var = self.velocity_process_std * self.velocity_process_std
        q = np.diag([
            0.25 * dt**4 * accel_var,
            0.25 * dt**4 * accel_var,
            dt * dt * (accel_var + vel_var),
            dt * dt * (accel_var + vel_var),
            dt * dt * self.imu_yaw_std * self.imu_yaw_std,
        ])
        self.p = f @ self.p @ f.T + q

    def update_gps(self, gps_x, gps_y, msg):
        z = np.array([[gps_x], [gps_y]], dtype=float)
        h = np.array([[1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0]])

        std_x = self.gps_xy_std
        std_y = self.gps_xy_std
        if msg.position_covariance[0] > 0.0:
            std_x = max(math.sqrt(msg.position_covariance[0]), 0.1)
        if msg.position_covariance[4] > 0.0:
            std_y = max(math.sqrt(msg.position_covariance[4]), 0.1)

        r = np.diag([std_x * std_x, std_y * std_y])
        self.kalman_update(z, h, r)

    def update_yaw(self, imu_yaw):
        h = np.array([[0.0, 0.0, 0.0, 0.0, 1.0]])
        z = np.array([[imu_yaw]])
        r = np.array([[self.imu_yaw_std * self.imu_yaw_std]])
        innovation = z - h @ self.x
        innovation[0, 0] = normalize_angle(innovation[0, 0])
        self.kalman_update(z, h, r, innovation=innovation)
        self.x[4, 0] = normalize_angle(self.x[4, 0])

    def kalman_update(self, z, h, r, innovation=None):
        y = innovation if innovation is not None else z - h @ self.x
        s = h @ self.p @ h.T + r
        k = self.p @ h.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        i = np.eye(self.p.shape[0])
        self.p = (i - k @ h) @ self.p @ (i - k @ h).T + k @ r @ k.T

    def publish_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = float(self.x[0, 0])
        msg.pose.pose.position.y = float(self.x[1, 0])
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(float(self.x[4, 0]))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = float(self.x[2, 0])
        msg.twist.twist.linear.y = float(self.x[3, 0])

        msg.pose.covariance[0] = float(self.p[0, 0])
        msg.pose.covariance[7] = float(self.p[1, 1])
        msg.pose.covariance[35] = float(self.p[4, 4])
        msg.twist.covariance[0] = float(self.p[2, 2])
        msg.twist.covariance[7] = float(self.p[3, 3])

        self.odom_pub.publish(msg)
        if self.tf_broadcaster is not None:
            self.publish_tf_msg(msg)

    def publish_tf_msg(self, odom_msg):
        tf_msg = TransformStamped()
        tf_msg.header = odom_msg.header
        tf_msg.child_frame_id = odom_msg.child_frame_id
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
        tf_msg.transform.translation.z = odom_msg.pose.pose.position.z
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsImuEkfLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
