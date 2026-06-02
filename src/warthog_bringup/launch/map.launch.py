#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lio_sam_params_file = LaunchConfiguration("lio_sam_params_file")
    input_points_topic = LaunchConfiguration("input_points_topic")
    input_imu_topic = LaunchConfiguration("input_imu_topic")
    output_points_topic = LaunchConfiguration("output_points_topic")
    output_imu_topic = LaunchConfiguration("output_imu_topic")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    stamp_with_now = LaunchConfiguration("stamp_with_now")
    stamp_points_with_now = LaunchConfiguration("stamp_points_with_now")
    stamp_imu_with_now = LaunchConfiguration("stamp_imu_with_now")
    log_stamp_offsets = LaunchConfiguration("log_stamp_offsets")

    sensor_relay = Node(
        package="warthog_bringup",
        executable="lio_sam_sensor_relay",
        name="lio_sam_sensor_relay",
        output="screen",
        parameters=[{
            "input_points_topic": input_points_topic,
            "input_imu_topic": input_imu_topic,
            "output_points_topic": output_points_topic,
            "output_imu_topic": output_imu_topic,
            "lidar_frame_id": lidar_frame_id,
            "imu_frame_id": imu_frame_id,
            "stamp_with_now": stamp_with_now,
            "stamp_points_with_now": stamp_points_with_now,
            "stamp_imu_with_now": stamp_imu_with_now,
            "log_stamp_offsets": log_stamp_offsets,
        }],
    )

    lio_sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("lio_sam"),
                "launch",
                "run.launch.py",
            ])
        ),
        launch_arguments={
            "params_file": lio_sam_params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "lio_sam_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("lio_sam"),
                "config",
                "params.yaml",
            ]),
            description="LIO-SAM parameter file.",
        ),
        DeclareLaunchArgument(
            "input_points_topic",
            default_value="/ouster/points",
            description="Ouster PointCloud2 topic to relay into LIO-SAM.",
        ),
        DeclareLaunchArgument(
            "input_imu_topic",
            default_value="/vectornav/imu",
            description="VectorNav sensor_msgs/Imu topic to relay into LIO-SAM.",
        ),
        DeclareLaunchArgument(
            "output_points_topic",
            default_value="/points",
            description="PointCloud2 topic expected by LIO-SAM.",
        ),
        DeclareLaunchArgument(
            "output_imu_topic",
            default_value="/imu/data",
            description="IMU topic expected by LIO-SAM.",
        ),
        DeclareLaunchArgument(
            "lidar_frame_id",
            default_value="laser_sensor_frame",
            description="Frame id to write into relayed PointCloud2 messages.",
        ),
        DeclareLaunchArgument(
            "imu_frame_id",
            default_value="base_link",
            description="Frame id to write into relayed Imu messages.",
        ),
        DeclareLaunchArgument(
            "stamp_with_now",
            default_value="true",
            description="Replace both sensor timestamps with relay receive time.",
        ),
        DeclareLaunchArgument(
            "stamp_points_with_now",
            default_value="true",
            description="Replace point cloud timestamps with relay receive time.",
        ),
        DeclareLaunchArgument(
            "stamp_imu_with_now",
            default_value="true",
            description="Replace IMU timestamps with relay receive time.",
        ),
        DeclareLaunchArgument(
            "log_stamp_offsets",
            default_value="false",
            description="Log latest IMU minus point cloud timestamp offset.",
        ),
        sensor_relay,
        lio_sam_launch,
    ])
