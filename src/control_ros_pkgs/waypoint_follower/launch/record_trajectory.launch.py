#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    trajectory_csv = LaunchConfiguration("trajectory_csv")
    redis_host = LaunchConfiguration("redis_host")
    redis_port = LaunchConfiguration("redis_port")
    redis_db = LaunchConfiguration("redis_db")
    redis_pose_key = LaunchConfiguration("redis_pose_key")
    enable_plot = LaunchConfiguration("enable_plot")

    recorder = Node(
        package="waypoint_follower",
        executable="trajectory_recorder",
        name="trajectory_recorder",
        output="screen",
        parameters=[
            params_file,
            {
                "output_csv": trajectory_csv,
                "redis_host": redis_host,
                "redis_port": redis_port,
                "redis_db": redis_db,
                "redis_pose_key": redis_pose_key,
            },
        ],
    )

    plotter = Node(
        package="waypoint_follower",
        executable="trajectory_record_plotter",
        name="trajectory_record_plotter",
        output="screen",
        condition=IfCondition(enable_plot),
        parameters=[
            params_file,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("waypoint_follower"),
                "config",
                "warthog_waypoint_follower.yaml",
            ]),
            description="YAML parameters for the Warthog trajectory recorder.",
        ),
        DeclareLaunchArgument(
            "trajectory_csv",
            default_value=PathJoinSubstitution([
                FindPackageShare("waypoint_follower"),
                "trajectories",
                "warthog_trajectory.csv",
            ]),
            description="CSV file where sampled odometry waypoints are saved.",
        ),
        DeclareLaunchArgument(
            "redis_host",
            default_value="localhost",
            description="Redis host containing the latest robot pose.",
        ),
        DeclareLaunchArgument(
            "redis_port",
            default_value="6379",
            description="Redis port containing the latest robot pose.",
        ),
        DeclareLaunchArgument(
            "redis_db",
            default_value="0",
            description="Redis database index containing the latest robot pose.",
        ),
        DeclareLaunchArgument(
            "redis_pose_key",
            default_value="warthog:odom",
            description="Redis key containing the latest robot pose.",
        ),
        DeclareLaunchArgument(
            "enable_plot",
            default_value="true",
            description="Start the live Matplotlib plot while recording trajectory.",
        ),
        recorder,
        plotter,
    ])
