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
    odom_topic = LaunchConfiguration("odom_topic")
    pose_source_type = LaunchConfiguration("pose_source_type")
    enable_plot = LaunchConfiguration("enable_plot")

    follower = Node(
        package="waypoint_follower",
        executable="pure_pursuit_follower",
        name="pure_pursuit_follower",
        output="screen",
        parameters=[
            params_file,
            {
                "trajectory_csv": trajectory_csv,
                "odom_topic": odom_topic,
                "pose_source_type": pose_source_type,
            },
        ],
    )

    plotter = Node(
        package="waypoint_follower",
        executable="trajectory_plotter",
        name="trajectory_plotter",
        output="screen",
        condition=IfCondition(enable_plot),
        parameters=[
            params_file,
            {
                "trajectory_csv": trajectory_csv,
                "odom_topic": odom_topic,
                "pose_source_type": pose_source_type,
            },
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
            description="YAML parameters for the Warthog pure pursuit follower.",
        ),
        DeclareLaunchArgument(
            "trajectory_csv",
            default_value=PathJoinSubstitution([
                FindPackageShare("waypoint_follower"),
                "trajectories",
                "warthog_trajectory.csv",
            ]),
            description="CSV trajectory recorded by trajectory_recorder.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/warthog/localization/odom",
            description="Odometry topic used by waypoint_follower nodes.",
        ),
        DeclareLaunchArgument(
            "pose_source_type",
            default_value="odom",
            description="Pose source mode for waypoint_follower nodes.",
        ),
        DeclareLaunchArgument(
            "enable_plot",
            default_value="true",
            description="Start the live Matplotlib trajectory plotter with pure pursuit.",
        ),
        follower,
        plotter,
    ])
