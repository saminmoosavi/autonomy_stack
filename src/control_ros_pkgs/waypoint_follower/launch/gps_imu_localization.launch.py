#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    gps_topic = LaunchConfiguration("gps_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    frame_id = LaunchConfiguration("frame_id")
    child_frame_id = LaunchConfiguration("child_frame_id")
    publish_tf = LaunchConfiguration("publish_tf")
    enable_plot = LaunchConfiguration("enable_plot")

    localizer = Node(
        package="waypoint_follower",
        executable="gps_imu_ekf_localizer",
        name="gps_imu_ekf_localizer",
        output="screen",
        parameters=[
            params_file,
            {
                "gps_topic": gps_topic,
                "imu_topic": imu_topic,
                "odom_topic": odom_topic,
                "frame_id": frame_id,
                "child_frame_id": child_frame_id,
                "publish_tf": publish_tf,
            },
        ],
    )

    plotter = Node(
        package="waypoint_follower",
        executable="ekf_comparison_plotter",
        name="ekf_comparison_plotter",
        output="screen",
        condition=IfCondition(enable_plot),
        parameters=[
            params_file,
            {
                "gps_topic": gps_topic,
                "imu_topic": imu_topic,
                "odom_topic": odom_topic,
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
            description="YAML parameters for the GPS+IMU EKF localizer.",
        ),
        DeclareLaunchArgument(
            "gps_topic",
            default_value="/vectornav/gnss",
            description="NavSatFix topic used for EKF position updates.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/vectornav/imu",
            description="Imu topic used for EKF prediction and yaw updates.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/warthog/localization/odom",
            description="Odometry topic published by the GPS+IMU EKF.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="odom",
            description="Parent frame for the EKF odometry.",
        ),
        DeclareLaunchArgument(
            "child_frame_id",
            default_value="base_link",
            description="Child frame for the EKF odometry.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Publish odom to base_link TF from the EKF.",
        ),
        DeclareLaunchArgument(
            "enable_plot",
            default_value="true",
            description="Start the EKF comparison plotter.",
        ),
        localizer,
        plotter,
    ])
