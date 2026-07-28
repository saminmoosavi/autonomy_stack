#!/usr/bin/env python3
'''
Command line examples:

In Docker, source the workspace first, then resolve config paths from the
installed ROS package:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  EVO_CFG="$(ros2 pkg prefix evo_skill_ros)/share/evo_skill_ros/config"

Simulation:
  ros2 launch evo_skill_ros evo_plan_run.launch.py \
    namespace:=/a200_0000 \
    robot_name:=jackal_1 \
    target_region:=R10 \
    graph_file:=$EVO_CFG/graph.json \
    domain_file:=$EVO_CFG/factory_sim_domain.pddl \
    plan_file:=$EVO_CFG/evoskill_plan_sim.txt \
    costmap_edit_max_radius:=1.0 \
    require_map:=false

Actual robot:
  ros2 launch evo_skill_ros evo_plan_run.launch.py \
    namespace:=/j100_0611 \
    robot_name:=jackal_1 \
    target_region:=fire \
    graph_file:=$EVO_CFG/graph_lab.json \
    domain_file:=$EVO_CFG/lens_lab_domain.pddl \
    plan_file:=$EVO_CFG/evoskill_plan_lab.txt \
    costmap_edit_max_radius:=1.0 \
    require_map:=true \
    tracking_topic:=/yolo/tracking \
    points_topic:=/sensors/camera_0/points \
    odom_topic:=/platform/odom/filtered \
    tracker_out_topic:=/tracks

'''
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("evo_skill_ros")
    default_graph_file = PathJoinSubstitution([package_share, "config", "graph.json"])
    default_domain_file = PathJoinSubstitution([package_share, "config", "factory_sim_domain.pddl"])
    default_plan_file = PathJoinSubstitution([package_share, "config", "plan.txt"])
    default_exclude_file = PathJoinSubstitution([package_share, "config", "exclude.json"])

    namespace = LaunchConfiguration("namespace")
    graph_file = LaunchConfiguration("graph_file")
    domain_file = LaunchConfiguration("domain_file")
    plan_file = LaunchConfiguration("plan_file")
    frame_id = LaunchConfiguration("frame_id")
    tracker_target_frame = LaunchConfiguration("tracker_target_frame")
    robot_name = LaunchConfiguration("robot_name")
    target_region = LaunchConfiguration("target_region")
    tracks_topic = LaunchConfiguration("tracks_topic")

    tracker_node = Node(
        package="evo_skill_ros",
        executable="tracker_with_yolo",
        name="tracker_with_yolo",
        namespace=namespace,
        output="screen",
        parameters=[{
            "namespace": namespace,
            "tracking_topic": LaunchConfiguration("tracking_topic"),
            "points_topic": LaunchConfiguration("points_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "out_topic": LaunchConfiguration("tracker_out_topic"),
            "target_frame": tracker_target_frame,
        }],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # Records robot pose + detected objects (2D bbox and map-frame position) so
    # "plan expected A at R3, perception saw B" is recoverable after the run.
    # namespace= is only so the /tf remap resolves to <ns>/tf, same as tracker.
    # ParameterValue(value_type=str) pins the type at the launch layer, since
    # launch_ros YAML-infers a resolved LaunchConfiguration: obs_log_cameras:=0
    # would otherwise arrive as an int while 0,1,2,3 arrives as a str. The node
    # also declares these dynamically typed, so both layers are safe.
    observation_log_node = Node(
        package="evo_skill_ros",
        executable="observation_logger",
        name="observation_logger",
        namespace=namespace,
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_observation_log")),
        parameters=[{
            "use_sim_time": True,
            "namespace": namespace,
            "graph_file": graph_file,
            "exclude_file": ParameterValue(
                LaunchConfiguration("obs_exclude_file"), value_type=str),
            "cameras": ParameterValue(
                LaunchConfiguration("obs_log_cameras"), value_type=str),
            "log_classes": ParameterValue(
                LaunchConfiguration("obs_log_classes"), value_type=str),
            "observations_file": ParameterValue(
                LaunchConfiguration("obs_log_file"), value_type=str),
            "belief_file": ParameterValue(
                LaunchConfiguration("obs_belief_file"), value_type=str),
            "log_period_s": LaunchConfiguration("obs_log_period_s"),
            "region_snap_max_m": LaunchConfiguration("obs_region_snap_max_m"),
            "target_frame": tracker_target_frame,
            "base_frame": ParameterValue(
                LaunchConfiguration("obs_log_base_frame"), value_type=str),
        }],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    evo_plan_deploy_node = Node(
        package="evo_skill_ros",
        executable="evo_plan_deploy",
        name="evo_plan_deploy",
        output="screen",
        parameters=[{
            "ns": namespace,
            "graph_file": graph_file,
            "domain_file": domain_file,
            "plan_file": plan_file,
            "frame_id": frame_id,
            "robot_name": robot_name,
            "target_region": target_region,
            "tracks_topic": tracks_topic,
            "enable_json_log": LaunchConfiguration("enable_json_log"),
            "json_log_file": LaunchConfiguration("json_log_file"),
            "enable_stl_replan": LaunchConfiguration("enable_stl_replan"),
            "max_nav2_replans": LaunchConfiguration("max_nav2_replans"),
            "stl_replan_cooldown_s": LaunchConfiguration("stl_replan_cooldown_s"),
            "eventual_goal_check_distance": LaunchConfiguration("eventual_goal_check_distance"),
            "require_map": LaunchConfiguration("require_map"),
            "costmap_edit_max_radius": LaunchConfiguration("costmap_edit_max_radius"),
        }],
    )

    visualizer_node = Node(
        package="evo_skill_ros",
        executable="factory_world_visualizer",
        name="factory_world_visualizer",
        output="screen",
        parameters=[{
            "json_file": graph_file,
            "frame_id": frame_id,
        }],
    )

    scand_metrics_node = Node(
        package="evo_skill_ros",
        executable="scand_metrics",
        name="scand_metrics",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_metrics")),
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--ns",          LaunchConfiguration("namespace"),
            "--world",       LaunchConfiguration("metrics_world"),
            "--duration",    LaunchConfiguration("metrics_duration"),
            "--stop-on-success", LaunchConfiguration("metrics_stop_on_success"),
            "--goal-topic",  LaunchConfiguration("metrics_goal_topic"),
            "--actors-sdf",  LaunchConfiguration("metrics_actors_sdf"),
            "--json-out",    LaunchConfiguration("metrics_json_out"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="/a200_0000",
            description="Robot namespace used by tracker and Nav2.",
        ),
        DeclareLaunchArgument(
            "graph_file",
            default_value=default_graph_file,
            description="Factory graph JSON used to map PDDL move goals to coordinates.",
        ),
        DeclareLaunchArgument(
            "domain_file",
            default_value=default_domain_file,
            description="PDDL domain file used to validate the text plan.",
        ),
        DeclareLaunchArgument(
            "plan_file",
            default_value=default_plan_file,
            description="Text file containing the PDDL plan to deploy.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="map",
            description="Global frame for Nav2 goals and tracked detections.",
        ),
        DeclareLaunchArgument(
            "tracker_target_frame",
            default_value="map",
            description="TF frame that tracker_with_yolo transforms camera detections into.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="jackal_1",
            description="PDDL robot object name used by the plan file.",
        ),
        DeclareLaunchArgument(
            "target_region",
            default_value="R10",
            description="Expected final graph region for validating the PDDL plan.",
        ),
        DeclareLaunchArgument(
            "tracks_topic",
            default_value="/a200_0000/tracks",
            description="Detection3D topic monitored by the STL layer.",
        ),
        DeclareLaunchArgument(
            "tracking_topic",
            default_value="/yolo/tracking",
            description="YOLO DetectionArray tracking topic.",
        ),
        DeclareLaunchArgument(
            "points_topic",
            default_value="/sensors/camera_0/points",
            description="PointCloud2 topic suffix under namespace for tracker depth lookup.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/platform/odom/filtered",
            description="Odometry topic suffix under namespace for tracker frame fallback.",
        ),
        DeclareLaunchArgument(
            "tracker_out_topic",
            default_value="/tracks",
            description="Tracker output topic suffix under namespace.",
        ),
        DeclareLaunchArgument(
            "enable_json_log",
            default_value="true",
            description="Write PDDL/Nav2/STL events to a JSON file.",
        ),
        DeclareLaunchArgument(
            "json_log_file",
            default_value="/home/user/autonomy_stack_ros_humble/evo_plan_deploy_log.json",
            description="JSON event log file for evo_plan_deploy.",
        ),
        DeclareLaunchArgument(
            "enable_stl_replan",
            default_value="true",
            description="Trigger Nav2 replanning when STL obstacle avoidance is violated.",
        ),
        DeclareLaunchArgument(
            "max_nav2_replans",
            default_value="3",
            description="Maximum number of STL feedback Nav2 replans per node run.",
        ),
        DeclareLaunchArgument(
            "stl_replan_cooldown_s",
            default_value="2.0",
            description="Minimum time between STL feedback replans.",
        ),
        DeclareLaunchArgument(
            "eventual_goal_check_distance",
            default_value="2.5",
            description="Only check the STL eventual-goal condition within this distance of the target.",
        ),
        DeclareLaunchArgument(
            "require_map",
            default_value="false",
            description="Require an occupancy grid before deploying the plan.",
        ),
        DeclareLaunchArgument(
            "costmap_edit_max_radius",
            default_value="1.0",
            description="Maximum radius in meters for STL-triggered costmap inflation.",
        ),
        DeclareLaunchArgument(
            "enable_metrics",
            default_value="false",
            description="Launch scand_metrics alongside the planner to collect social-compliance metrics.",
        ),
        DeclareLaunchArgument(
            "metrics_world",
            default_value="warehouse",
            description="Gazebo world name passed to scand_metrics --world.",
        ),
        DeclareLaunchArgument(
            "metrics_duration",
            default_value="0",
            description="scand_metrics --duration: auto-stop after N seconds (0 = run until Ctrl-C / node shutdown).",
        ),
        DeclareLaunchArgument(
            "metrics_stop_on_success",
            default_value="false",
            description="scand_metrics --stop-on-success: stop and write results once the goal is reached cleanly.",
        ),
        DeclareLaunchArgument(
            "metrics_goal_topic",
            default_value="/ppddl_nav2_goals",
            description="nav_msgs/Path topic that scand_metrics uses to track the plan goal (published by evo_plan_deploy).",
        ),
        DeclareLaunchArgument(
            "metrics_actors_sdf",
            default_value="",
            description="Path to the world .sdf for scripted <actor> pedestrian trajectories (empty = disabled).",
        ),
        DeclareLaunchArgument(
            "metrics_json_out",
            default_value="/home/user/autonomy_stack_ros_humble/scand_metrics_out.json",
            description="Path where scand_metrics writes the JSON summary at exit.",
        ),
        DeclareLaunchArgument(
            "enable_observation_log",
            default_value="false",
            description="Log robot pose + detected object bboxes/map positions for plan-vs-perception desync.",
        ),
        DeclareLaunchArgument(
            "obs_log_file",
            default_value="/home/user/autonomy_stack_ros_humble/observations.jsonl",
            description="JSONL stream, one appended snapshot per interval.",
        ),
        DeclareLaunchArgument(
            "obs_belief_file",
            default_value="/home/user/autonomy_stack_ros_humble/belief.json",
            description="Cumulative per-region expected-vs-observed belief map.",
        ),
        DeclareLaunchArgument(
            "obs_log_period_s",
            default_value="1.0",
            description="Snapshot interval in SIM seconds (the node runs with use_sim_time).",
        ),
        DeclareLaunchArgument(
            "obs_log_cameras",
            default_value="0",
            description=("Comma-separated cameras to log, e.g. '0,1,2,3' with MULTICAM=1. "
                         "Each adds a PointCloud2 subscription; with '0' only, objects "
                         "beside/behind the robot are absent even though /tracks saw them."),
        ),
        DeclareLaunchArgument(
            "obs_log_classes",
            default_value="",
            description="Comma-separated YOLO class allowlist (case-insensitive). Empty = log all.",
        ),
        DeclareLaunchArgument(
            "obs_exclude_file",
            default_value=default_exclude_file,
            description=("JSON class denylist for logging only; takes precedence over "
                         "obs_log_classes. Optional -- a missing file excludes nothing."),
        ),
        DeclareLaunchArgument(
            "obs_log_base_frame",
            default_value="base_link",
            description="Robot body frame for the map->base TF pose lookup.",
        ),
        DeclareLaunchArgument(
            "obs_region_snap_max_m",
            default_value="6.0",
            description=("Max distance to a graph region centroid for attribution. Regions are "
                         "points, not polygons, so without this everything snaps to something."),
        ),
        tracker_node,
        evo_plan_deploy_node,
        visualizer_node,
        scand_metrics_node,
        observation_log_node,
    ])
