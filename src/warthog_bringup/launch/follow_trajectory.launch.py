#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def liorf_localization_nodes(
    params_file,
    point_cloud_topic,
    imu_topic,
    lidar_frame_id,
    baselink_frame_id,
    sensor_type,
):
    parameter_sources = [
        params_file,
        {
            "pointCloudTopic": point_cloud_topic,
            "imuTopic": imu_topic,
            "lidarFrame": lidar_frame_id,
            "baselinkFrame": baselink_frame_id,
            "sensor": sensor_type,
        },
    ]

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="liorf_localization",
            executable="liorf_localization_imuPreintegration",
            name="liorf_localization_imuPreintegration",
            parameters=parameter_sources,
            output="screen",
        ),
        Node(
            package="liorf_localization",
            executable="liorf_localization_imageProjection",
            name="liorf_localization_imageProjection",
            parameters=parameter_sources,
            output="screen",
        ),
        Node(
            package="liorf_localization",
            executable="liorf_localization_mapOptmization",
            name="liorf_localization_mapOptmization",
            parameters=parameter_sources,
            output="screen",
        ),
    ]


def generate_launch_description():
    liorf_params_file = LaunchConfiguration("liorf_params_file")
    waypoint_params_file = LaunchConfiguration("waypoint_params_file")
    trajectory_csv = LaunchConfiguration("trajectory_csv")
    odom_topic = LaunchConfiguration("odom_topic")
    pose_source_type = LaunchConfiguration("pose_source_type")
    enable_plot = LaunchConfiguration("enable_plot")
    input_points_topic = LaunchConfiguration("input_points_topic")
    input_imu_topic = LaunchConfiguration("input_imu_topic")
    output_points_topic = LaunchConfiguration("output_points_topic")
    output_imu_topic = LaunchConfiguration("output_imu_topic")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    baselink_frame_id = LaunchConfiguration("baselink_frame_id")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    stamp_with_now = LaunchConfiguration("stamp_with_now")
    stamp_points_with_now = LaunchConfiguration("stamp_points_with_now")
    stamp_imu_with_now = LaunchConfiguration("stamp_imu_with_now")
    log_stamp_offsets = LaunchConfiguration("log_stamp_offsets")
    sensor_type = LaunchConfiguration("sensor_type")

    sensor_relay = Node(
        package="warthog_bringup",
        executable="lio_sam_sensor_relay",
        name="liorf_sensor_relay",
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

    waypoint_follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("waypoint_follower"),
                "launch",
                "follow_trajectory.launch.py",
            ])
        ),
        launch_arguments={
            "params_file": waypoint_params_file,
            "trajectory_csv": trajectory_csv,
            "odom_topic": odom_topic,
            "pose_source_type": pose_source_type,
            "enable_plot": enable_plot,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "liorf_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("liorf_localization"),
                "config",
                "localization.yaml",
            ]),
            description="LIORF localization parameter file.",
        ),
        DeclareLaunchArgument(
            "waypoint_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("waypoint_follower"),
                "config",
                "warthog_waypoint_follower.yaml",
            ]),
            description="Waypoint follower parameter file.",
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
            default_value="/liorf_localization/mapping/odometry",
            description="LIORF odometry topic used by waypoint_follower nodes.",
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
        DeclareLaunchArgument(
            "input_points_topic",
            default_value="/ouster/points",
            description="Ouster PointCloud2 topic to relay into LIORF.",
        ),
        DeclareLaunchArgument(
            "input_imu_topic",
            default_value="/vectornav/imu",
            description="VectorNav sensor_msgs/Imu topic to relay into LIORF.",
        ),
        DeclareLaunchArgument(
            "output_points_topic",
            default_value="/points",
            description="PointCloud2 topic relayed into LIORF.",
        ),
        DeclareLaunchArgument(
            "output_imu_topic",
            default_value="/imu/data",
            description="IMU topic expected by the default LIORF config.",
        ),
        DeclareLaunchArgument(
            "lidar_frame_id",
            default_value="lidar_link",
            description="Frame id to write into relayed PointCloud2 messages.",
        ),
        DeclareLaunchArgument(
            "baselink_frame_id",
            default_value="base_link",
            description="LIORF base link frame parameter.",
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
        DeclareLaunchArgument(
            "sensor_type",
            default_value="ouster",
            description="LIORF lidar sensor type parameter.",
        ),
        sensor_relay,
        *liorf_localization_nodes(
            liorf_params_file,
            output_points_topic,
            output_imu_topic,
            lidar_frame_id,
            baselink_frame_id,
            sensor_type,
        ),
        waypoint_follow_launch,
    ])
