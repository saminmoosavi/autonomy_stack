#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_setup_path = LaunchConfiguration("nav2_setup_path")
    slam_setup_path = LaunchConfiguration("slam_setup_path")
    namespace = LaunchConfiguration("namespace")
    input_scan_topic = LaunchConfiguration("input_scan_topic")
    nav2_scan_topic = LaunchConfiguration("nav2_scan_topic")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("clearpath_nav2_demos"),
                "launch",
                "nav2.launch.py",
            ])
        ),
        launch_arguments={
            "setup_path": nav2_setup_path,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("clearpath_nav2_demos"),
                "launch",
                "slam.launch.py",
            ])
        ),
        launch_arguments={
            "setup_path": slam_setup_path,
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("clearpath_viz"),
                "launch",
                "view_navigation.launch.py",
            ])
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    scan_relay_node = Node(
        package="warthog_bringup",
        executable="scan_relay",
        name="scan_relay",
        output="screen",
        parameters=[{
            "input_topic": input_scan_topic,
            "output_topic": nav2_scan_topic,
        }],

    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "nav2_setup_path",
            default_value="/home/user/warthog_setup/",
            description="Setup path passed to clearpath_nav2_demos nav2.launch.py.",
        ),
        DeclareLaunchArgument(
            "slam_setup_path",
            default_value="/home/user/warthog_setup/",
            description="Setup path passed to clearpath_nav2_demos slam.launch.py.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="w200_0105",
            description="Robot namespace passed to clearpath_viz view_navigation.launch.py.",
        ),
        DeclareLaunchArgument(
            "input_scan_topic",
            default_value="/ouster/scan",
            description="LaserScan topic published by the Warthog lidar.",
        ),
        DeclareLaunchArgument(
            "nav2_scan_topic",
            default_value="/w200_0105/sensors/lidar2d_0/scan",
            description="Namespaced LaserScan topic expected by Nav2/SLAM defaults.",
        ),
        scan_relay_node,
        nav2_launch,
        slam_launch,
        rviz_launch,
    ])
