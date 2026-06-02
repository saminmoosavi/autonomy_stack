#!/usr/bin/env python3

import math

import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from waypoint_follower.pose_utils import quaternion_to_yaw


class EkfComparisonPlotter(Node):
    def __init__(self):
        super().__init__("ekf_comparison_plotter")

        self.gps_topic = self.declare_parameter("gps_topic", "/vectornav/gnss").value
        self.imu_topic = self.declare_parameter("imu_topic", "/vectornav/imu").value
        self.odom_topic = self.declare_parameter("odom_topic", "/warthog/localization/odom").value
        self.origin_lat = float(self.declare_parameter("origin_lat", math.nan).value)
        self.origin_lon = float(self.declare_parameter("origin_lon", math.nan).value)
        self.trail_max_points = int(self.declare_parameter("ekf_plot_trail_max_points", 5000).value)
        self.plot_rate_hz = float(self.declare_parameter("ekf_plot_rate_hz", 5.0).value)
        self.figure_title = self.declare_parameter(
            "ekf_plot_title", "GPS / IMU / EKF Comparison"
        ).value

        self.raw_gps_path = []
        self.ekf_path = []
        self.current_gps = None
        self.current_ekf = None
        self.current_yaw = None

        self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, 20)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 100)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.timer = self.create_timer(1.0 / max(self.plot_rate_hz, 0.1), self.update_plot)

        self.setup_plot()
        self.get_logger().info(
            f"Plotting raw GPS {self.gps_topic}, IMU heading {self.imu_topic}, "
            f"and EKF odom {self.odom_topic}"
        )

    def gps_callback(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if math.isnan(self.origin_lat) or math.isnan(self.origin_lon):
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(
                f"Set plot GPS origin lat={self.origin_lat:.8f}, lon={self.origin_lon:.8f}"
            )

        x, y = self.gps_to_local_xy(msg.latitude, msg.longitude)
        self.current_gps = (x, y)
        self.raw_gps_path.append((x, y))
        self.trim_paths()

    def imu_callback(self, msg):
        self.current_yaw = quaternion_to_yaw(msg.orientation)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        point = (pose.position.x, pose.position.y)
        self.current_ekf = point
        self.ekf_path.append(point)
        self.trim_paths()

    def gps_to_local_xy(self, latitude, longitude):
        earth_radius_m = 6378137.0
        lat0 = math.radians(self.origin_lat)
        d_lat = math.radians(latitude - self.origin_lat)
        d_lon = math.radians(longitude - self.origin_lon)
        x = earth_radius_m * d_lon * math.cos(lat0)
        y = earth_radius_m * d_lat
        return x, y

    def trim_paths(self):
        if len(self.raw_gps_path) > self.trail_max_points:
            self.raw_gps_path = self.raw_gps_path[-self.trail_max_points:]
        if len(self.ekf_path) > self.trail_max_points:
            self.ekf_path = self.ekf_path[-self.trail_max_points:]

    def setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title(self.figure_title)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True)
        self.ax.axis("equal")

        (self.gps_line,) = self.ax.plot([], [], ".", color="0.55", markersize=4, label="raw GPS")
        (self.ekf_line,) = self.ax.plot([], [], "b-", linewidth=2.0, label="EKF odom")
        (self.gps_marker,) = self.ax.plot([], [], "ko", markersize=6, label="latest GPS")
        (self.ekf_marker,) = self.ax.plot([], [], "go", markersize=7, label="latest EKF")
        (self.imu_heading_line,) = self.ax.plot([], [], "r-", linewidth=2.0, label="IMU yaw")
        self.ax.legend(loc="best")
        self.fig.tight_layout()
        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update_plot(self):
        if not plt.fignum_exists(self.fig.number):
            return

        gps_x = [point[0] for point in self.raw_gps_path]
        gps_y = [point[1] for point in self.raw_gps_path]
        ekf_x = [point[0] for point in self.ekf_path]
        ekf_y = [point[1] for point in self.ekf_path]

        self.gps_line.set_data(gps_x, gps_y)
        self.ekf_line.set_data(ekf_x, ekf_y)

        if self.current_gps is not None:
            self.gps_marker.set_data([self.current_gps[0]], [self.current_gps[1]])

        if self.current_ekf is not None:
            x, y = self.current_ekf
            self.ekf_marker.set_data([x], [y])
            if self.current_yaw is not None:
                heading_length = 1.0
                self.imu_heading_line.set_data(
                    [x, x + heading_length * math.cos(self.current_yaw)],
                    [y, y + heading_length * math.sin(self.current_yaw)],
                )

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.axis("equal")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = EkfComparisonPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close("all")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
