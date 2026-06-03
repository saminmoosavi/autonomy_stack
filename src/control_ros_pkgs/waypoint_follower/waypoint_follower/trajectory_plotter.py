#!/usr/bin/env python3

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from waypoint_follower.redis_pose_reader import RedisPoseReader


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_plotter")

        self.trajectory_csv = self.declare_parameter("trajectory_csv", "trajectories/warthog_trajectory.csv").value
        self.lookahead_distance_m = float(self.declare_parameter("lookahead_distance_m", 1.5).value)
        self.closed_loop = bool(self.declare_parameter("closed_loop", False).value)
        self.duplicate_endpoint_tolerance_m = float(
            self.declare_parameter("duplicate_endpoint_tolerance_m", 0.25).value
        )
        self.search_window_points = int(self.declare_parameter("search_window_points", 80).value)
        self.plot_rate_hz = float(self.declare_parameter("plot_rate_hz", 5.0).value)
        self.trail_max_points = int(self.declare_parameter("trail_max_points", 2000).value)
        self.figure_title = self.declare_parameter("figure_title", "Warthog Pure Pursuit").value
        self.redis_pose_reader = RedisPoseReader(self)

        self.waypoints = self.load_waypoints(self.trajectory_csv)
        self.current_pose = None
        self.actual_path = []
        self.progress_index = 0

        self.timer = self.create_timer(1.0 / max(self.plot_rate_hz, 0.1), self.update_plot)

        self.setup_plot()
        self.get_logger().info(
            f"Plotting {len(self.waypoints)} waypoints from {self.trajectory_csv}; "
            f"reading Redis stream {self.redis_pose_reader.stream_name} "
            f"node {self.redis_pose_reader.target_node}"
        )

    def load_waypoints(self, csv_file):
        path = self.resolve_package_path(csv_file)
        if not path.exists():
            raise FileNotFoundError(f"Trajectory CSV does not exist: {path}")

        waypoints = []
        with path.open("r", newline="") as handle:
            reader = csv.DictReader(handle)
            if not {"x", "y"}.issubset(reader.fieldnames or []):
                raise ValueError("Trajectory CSV must include x and y columns")

            for row in reader:
                try:
                    waypoints.append((float(row["x"]), float(row["y"])))
                except ValueError:
                    continue

        if len(waypoints) < 2:
            raise ValueError("Trajectory CSV must contain at least two valid waypoints")

        if self.closed_loop:
            first_x, first_y = waypoints[0]
            last_x, last_y = waypoints[-1]
            if math.hypot(last_x - first_x, last_y - first_y) <= self.duplicate_endpoint_tolerance_m:
                waypoints.pop()
        return waypoints

    @staticmethod
    def resolve_package_path(path_value):
        path = Path(path_value).expanduser()
        if path.is_absolute():
            return path

        package_share = Path(get_package_share_directory("waypoint_follower"))
        return package_share / path

    def setup_plot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title(self.figure_title)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True)
        self.ax.axis("equal")

        ref_x = [point[0] for point in self.waypoints]
        ref_y = [point[1] for point in self.waypoints]
        (self.reference_line,) = self.ax.plot(ref_x, ref_y, "k--", linewidth=1.5, label="saved trajectory")
        (self.actual_line,) = self.ax.plot([], [], "b-", linewidth=2.0, label="current path")
        (self.robot_marker,) = self.ax.plot([], [], "go", markersize=8, label="robot")
        (self.goal_marker,) = self.ax.plot([], [], "ro", markersize=8, label="next goal")
        (self.heading_line,) = self.ax.plot([], [], "g-", linewidth=2.0)
        self.ax.legend(loc="best")
        self.fig.tight_layout()
        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update_pose_from_redis(self):
        pose = self.redis_pose_reader.get_pose()
        if pose is None:
            return False

        x = pose.x
        y = pose.y
        yaw = pose.yaw
        self.current_pose = (x, y, yaw)
        self.actual_path.append((x, y))

        if len(self.actual_path) > self.trail_max_points:
            self.actual_path = self.actual_path[-self.trail_max_points:]
        return True

    def waypoint_at(self, absolute_index):
        if self.closed_loop:
            return self.waypoints[absolute_index % len(self.waypoints)]
        return self.waypoints[min(absolute_index, len(self.waypoints) - 1)]

    def nearest_progress_index(self, x, y):
        if self.closed_loop:
            search_start = max(0, self.progress_index - 5)
            search_stop = self.progress_index + max(self.search_window_points, 1)
        else:
            search_start = max(0, self.progress_index - 5)
            search_stop = len(self.waypoints) - 1

        return min(
            range(search_start, search_stop + 1),
            key=lambda i: math.hypot(self.waypoint_at(i)[0] - x, self.waypoint_at(i)[1] - y),
        )

    def next_goal(self, x, y):
        self.progress_index = self.nearest_progress_index(x, y)
        max_steps = len(self.waypoints) if self.closed_loop else len(self.waypoints) - self.progress_index

        for i in range(self.progress_index, self.progress_index + max_steps):
            wx, wy = self.waypoint_at(i)
            if math.hypot(wx - x, wy - y) >= self.lookahead_distance_m:
                return wx, wy
        return self.waypoint_at(self.progress_index + max_steps - 1)

    def update_plot(self):
        self.update_pose_from_redis()
        if self.current_pose is None or not plt.fignum_exists(self.fig.number):
            return

        x, y, yaw = self.current_pose
        path_x = [point[0] for point in self.actual_path]
        path_y = [point[1] for point in self.actual_path]
        goal_x, goal_y = self.next_goal(x, y)

        heading_length = max(0.5, self.lookahead_distance_m * 0.5)
        heading_x = [x, x + heading_length * math.cos(yaw)]
        heading_y = [y, y + heading_length * math.sin(yaw)]

        self.actual_line.set_data(path_x, path_y)
        self.robot_marker.set_data([x], [y])
        self.goal_marker.set_data([goal_x], [goal_y])
        self.heading_line.set_data(heading_x, heading_y)

        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.axis("equal")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrajectoryPlotter()
    except (FileNotFoundError, ValueError) as exc:
        rclpy.logging.get_logger("trajectory_plotter").error(str(exc))
        rclpy.shutdown()
        return

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
