#!/usr/bin/env python3

import math

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from waypoint_follower.redis_pose_reader import RedisPoseReader


class TrajectoryRecordPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_record_plotter")

        self.min_distance_m = float(self.declare_parameter("min_distance_m", 0.25).value)
        self.plot_rate_hz = float(self.declare_parameter("record_plot_rate_hz", 5.0).value)
        self.trail_max_points = int(self.declare_parameter("record_trail_max_points", 5000).value)
        self.figure_title = self.declare_parameter("record_figure_title", "Warthog Trajectory Recording").value
        self.redis_pose_reader = RedisPoseReader(self)

        self.current_pose = None
        self.raw_path = []
        self.saved_path = []
        self.last_saved_pose = None

        self.timer = self.create_timer(1.0 / max(self.plot_rate_hz, 0.1), self.update_plot)

        self.setup_plot()
        self.get_logger().info(
            f"Live plotting Redis stream {self.redis_pose_reader.stream_name} "
            f"node {self.redis_pose_reader.target_node}"
        )

    def setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title(self.figure_title)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True)
        self.ax.axis("equal")

        (self.raw_line,) = self.ax.plot([], [], color="0.75", linewidth=1.0, label="redis pose trail")
        (self.saved_line,) = self.ax.plot([], [], "b-", linewidth=2.0, label="sampled trajectory")
        (self.saved_points,) = self.ax.plot([], [], "bo", markersize=3, label="saved points")
        (self.robot_marker,) = self.ax.plot([], [], "go", markersize=8, label="robot")
        (self.heading_line,) = self.ax.plot([], [], "g-", linewidth=2.0)
        self.ax.legend(loc="best")
        self.fig.tight_layout()
        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def should_save(self, x, y):
        if self.last_saved_pose is None:
            return True

        last_x, last_y = self.last_saved_pose
        return math.hypot(x - last_x, y - last_y) >= self.min_distance_m

    def update_pose_from_redis(self):
        pose = self.redis_pose_reader.get_pose()
        if pose is None:
            return False

        x = pose.x
        y = pose.y
        yaw = pose.yaw
        self.current_pose = (x, y, yaw)

        self.raw_path.append((x, y))
        if len(self.raw_path) > self.trail_max_points:
            self.raw_path = self.raw_path[-self.trail_max_points:]

        if self.should_save(x, y):
            self.saved_path.append((x, y))
            self.last_saved_pose = (x, y)
        return True

    def update_plot(self):
        self.update_pose_from_redis()
        if self.current_pose is None or not plt.fignum_exists(self.fig.number):
            return

        x, y, yaw = self.current_pose
        raw_x = [point[0] for point in self.raw_path]
        raw_y = [point[1] for point in self.raw_path]
        saved_x = [point[0] for point in self.saved_path]
        saved_y = [point[1] for point in self.saved_path]

        heading_length = max(0.5, self.min_distance_m * 3.0)
        heading_x = [x, x + heading_length * math.cos(yaw)]
        heading_y = [y, y + heading_length * math.sin(yaw)]

        self.raw_line.set_data(raw_x, raw_y)
        self.saved_line.set_data(saved_x, saved_y)
        self.saved_points.set_data(saved_x, saved_y)
        self.robot_marker.set_data([x], [y])
        self.heading_line.set_data(heading_x, heading_y)

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.axis("equal")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecordPlotter()

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
