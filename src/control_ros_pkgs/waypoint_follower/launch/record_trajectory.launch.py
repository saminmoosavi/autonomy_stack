#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _scoped_topic(namespace, topic_name):
    topic_name = str(topic_name).strip()
    if topic_name.startswith("/"):
        return topic_name

    namespace = str(namespace).strip("/")
    if not namespace:
        return "/" + topic_name.strip("/")

    return "/" + namespace + "/" + topic_name.strip("/")


def _launch_nodes(context):
    use_sim = _as_bool(LaunchConfiguration("use_sim").perform(context))
    explicit_namespace = LaunchConfiguration("robot_namespace").perform(context).strip()
    sim_namespace = LaunchConfiguration("sim_robot_namespace").perform(context)
    real_namespace = LaunchConfiguration("real_robot_namespace").perform(context)
    robot_namespace = explicit_namespace or (sim_namespace if use_sim else real_namespace)

    localization_source = LaunchConfiguration(
        "sim_localization_source" if use_sim else "real_localization_source"
    ).perform(context)
    odom_name = LaunchConfiguration(
        "sim_odom_name" if use_sim else "real_odom_name"
    ).perform(context)
    pose_name = LaunchConfiguration(
        "sim_pose_name" if use_sim else "real_pose_name"
    ).perform(context)
    odom_topic = _scoped_topic(robot_namespace, odom_name)
    pose_topic = _scoped_topic(robot_namespace, pose_name)
    tf_fixed_frame = LaunchConfiguration(
        "sim_tf_fixed_frame" if use_sim else "real_tf_fixed_frame"
    ).perform(context)
    tf_robot_frame = LaunchConfiguration(
        "sim_tf_robot_frame" if use_sim else "real_tf_robot_frame"
    ).perform(context)

    common_parameters = [
        LaunchConfiguration("params_file"),
        {
            "localization_source": localization_source,
            "odom_topic": odom_topic,
            "pose_topic": pose_topic,
            "tf_fixed_frame": tf_fixed_frame,
            "tf_robot_frame": tf_robot_frame,
            "redis_host": LaunchConfiguration("redis_host"),
            "redis_port": LaunchConfiguration("redis_port"),
            "redis_db": LaunchConfiguration("redis_db"),
            "redis_stream_name": LaunchConfiguration("redis_stream_name"),
            "redis_target_node": LaunchConfiguration("redis_target_node"),
        },
    ]

    recorder = Node(
        package="waypoint_follower",
        executable="trajectory_recorder",
        name="trajectory_recorder",
        output="screen",
        parameters=[
            *common_parameters,
            {
                "output_csv": LaunchConfiguration("trajectory_csv"),
            },
        ],
    )

    plotter = Node(
        package="waypoint_follower",
        executable="trajectory_record_plotter",
        name="trajectory_record_plotter",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_plot")),
        parameters=common_parameters,
    )

    return [recorder, plotter]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use simulation localization from odometry instead of real robot Redis localization.",
        ),
        DeclareLaunchArgument(
            "sim_localization_source",
            default_value="pose",
            description="Localization source used when use_sim is true: pose, odom, tf, or redis.",
        ),
        DeclareLaunchArgument(
            "real_localization_source",
            default_value="redis",
            description="Localization source used when use_sim is false: redis, pose, odom, or tf.",
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="Optional robot namespace override. Empty selects sim_robot_namespace or real_robot_namespace.",
        ),
        DeclareLaunchArgument(
            "sim_robot_namespace",
            default_value="a200_0000",
            description="Robot namespace used when use_sim is true.",
        ),
        DeclareLaunchArgument(
            "real_robot_namespace",
            default_value="w200_0105",
            description="Robot namespace used when use_sim is false.",
        ),
        DeclareLaunchArgument(
            "sim_odom_name",
            default_value="odometry/filtered",
            description="Odometry topic name inside the simulation namespace.",
        ),
        DeclareLaunchArgument(
            "real_odom_name",
            default_value="platform/odom",
            description="Odometry topic name inside the real robot namespace. Only used if localization_source is odom.",
        ),
        DeclareLaunchArgument(
            "sim_pose_name",
            default_value="amcl_pose",
            description="Pose topic name inside the simulation namespace.",
        ),
        DeclareLaunchArgument(
            "real_pose_name",
            default_value="amcl_pose",
            description="Pose topic name inside the real robot namespace. Only used if localization_source is pose.",
        ),
        DeclareLaunchArgument(
            "sim_tf_fixed_frame",
            default_value="a200_0000/odom",
            description="TF fixed frame used when sim_localization_source is tf.",
        ),
        DeclareLaunchArgument(
            "sim_tf_robot_frame",
            default_value="a200_0000/base_link",
            description="TF robot frame used when sim_localization_source is tf.",
        ),
        DeclareLaunchArgument(
            "real_tf_fixed_frame",
            default_value="map",
            description="TF fixed frame used when real_localization_source is tf.",
        ),
        DeclareLaunchArgument(
            "real_tf_robot_frame",
            default_value="base_link",
            description="TF robot frame used when real_localization_source is tf.",
        ),
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
            "redis_stream_name",
            default_value="warthog:odom",
            description="Redis stream containing the latest XML graph entry.",
        ),
        DeclareLaunchArgument(
            "redis_target_node",
            default_value="warthog",
            description="Node name whose pose should be read from the Redis XML graph.",
        ),
        DeclareLaunchArgument(
            "enable_plot",
            default_value="true",
            description="Start the live Matplotlib plot while recording trajectory.",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
