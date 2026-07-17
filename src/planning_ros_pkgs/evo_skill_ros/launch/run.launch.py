#!/usr/bin/env python3
'''
ros2 launch evo_skill_ros run.launch.py \
  fast_downward_cmd:=$HOME/autonomy_stack_ros_humble/fast_downward/fast-downward.py \
  fast_downward_search:='astar(lmcut())' \
  graph_file:=config/graph_lab.json \
  domain_file:=config/lens_lab_domain.pddl \
  rviz_config:=$(ros2 pkg prefix evo_skill_ros)/share/evo_skill_ros/rviz/evo_plan_navigation.rviz \
  target_region:=top \
  namespace:=/j100_0611
  
'''


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("evo_skill_ros")
    default_graph_file = PathJoinSubstitution([package_share, "config", "graph_lab.json"])
    default_domain_file = PathJoinSubstitution([package_share, "config", "lens_lab_domain.pddl"])
    default_rviz_config = PathJoinSubstitution([package_share, "rviz", "evo_plan_navigation.rviz"])

    namespace = LaunchConfiguration("namespace")
    graph_file = LaunchConfiguration("graph_file")
    domain_file = LaunchConfiguration("domain_file")
    rviz_config = LaunchConfiguration("rviz_config")
    frame_id = LaunchConfiguration("frame_id")
    tracker_target_frame = LaunchConfiguration("tracker_target_frame")
    target_region = LaunchConfiguration("target_region")
    tracks_topic = LaunchConfiguration("tracks_topic")
    fast_downward_cmd = LaunchConfiguration("fast_downward_cmd")
    fast_downward_search = LaunchConfiguration("fast_downward_search")
    fast_downward_timeout_s = LaunchConfiguration("fast_downward_timeout_s")
    enable_json_log = LaunchConfiguration("enable_json_log")
    json_log_file = LaunchConfiguration("json_log_file")
    enable_stl_replan = LaunchConfiguration("enable_stl_replan")
    max_nav2_replans = LaunchConfiguration("max_nav2_replans")
    stl_replan_cooldown_s = LaunchConfiguration("stl_replan_cooldown_s")
    eventual_goal_check_distance = LaunchConfiguration("eventual_goal_check_distance")
    wait_tf_warn_after_s = LaunchConfiguration("wait_tf_warn_after_s")

    wait_for_tf_node = Node(
        package="evo_skill_ros",
        executable="wait_for_tf",
        name="wait_for_tf",
        namespace=namespace,
        output="screen",
        parameters=[{
            "namespace": namespace,
            "points_topic": LaunchConfiguration("points_topic"),
            "target_frame": tracker_target_frame,
            "warn_after_s": wait_tf_warn_after_s,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

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
            "out_topic": "/tracks",
            "target_frame": tracker_target_frame,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    pddl_nav2_stl_sat_node = Node(
        package="evo_skill_ros",
        executable="pddl_nav2_stl_sat",
        name="pddl_nav2_stl_sat",
        output="screen",
        parameters=[{
            "ns": namespace,
            "graph_file": graph_file,
            "domain_file": domain_file,
            "frame_id": frame_id,
            "target_region": target_region,
            "tracks_topic": tracks_topic,
            "fast_downward_cmd": fast_downward_cmd,
            "fast_downward_search": fast_downward_search,
            "fast_downward_timeout_s": fast_downward_timeout_s,
            "enable_json_log": enable_json_log,
            "json_log_file": json_log_file,
            "enable_stl_replan": enable_stl_replan,
            "max_nav2_replans": max_nav2_replans,
            "stl_replan_cooldown_s": stl_replan_cooldown_s,
            "eventual_goal_check_distance": eventual_goal_check_distance,
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

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("clearpath_viz"),
                "launch",
                "view_navigation.launch.py",
            ])
        ),
        launch_arguments={
            "rviz_config": rviz_config,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="/a200_0000",
            description="Robot namespace used by tracker and Nav2/STL nodes.",
        ),
        DeclareLaunchArgument(
            "graph_file",
            default_value=default_graph_file,
            description="Factory graph JSON used by the planner and visualizer.",
        ),
        DeclareLaunchArgument(
            "domain_file",
            default_value=default_domain_file,
            description="PDDL domain file used by the planner.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config loaded by clearpath_viz view_navigation.launch.py.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="map",
            description="Global frame for planning, visualization, and tracked detections.",
        ),
        DeclareLaunchArgument(
            "tracker_target_frame",
            default_value="map",
            description="TF frame that tracker_with_yolo transforms camera detections into.",
        ),
        DeclareLaunchArgument(
            "target_region",
            default_value="R10",
            description="Target graph region for the PDDL/Nav2/STL flow.",
        ),
        DeclareLaunchArgument(
            "tracks_topic",
            default_value="/a200_0000/tracks",
            description="Detection3D topic published by tracker_with_yolo and monitored by STL.",
        ),
        DeclareLaunchArgument(
            "fast_downward_cmd",
            default_value="fast-downward.py",
            description="Fast Downward driver command or absolute path.",
        ),
        DeclareLaunchArgument(
            "fast_downward_search",
            default_value="astar(lmcut())",
            description="Fast Downward search configuration.",
        ),
        DeclareLaunchArgument(
            "fast_downward_timeout_s",
            default_value="30.0",
            description="Fast Downward planning timeout in seconds.",
        ),
        DeclareLaunchArgument(
            "enable_json_log",
            default_value="true",
            description="Write PDDL/Nav2/STL events to a JSON file.",
        ),
        DeclareLaunchArgument(
            "json_log_file",
            default_value="/home/samin/autonomy_stack_ros_humble/pddl_nav2_stl_sat_log.json",
            description="JSON event log file for the PDDL/Nav2/STL node.",
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
            "wait_tf_warn_after_s",
            default_value="10.0",
            description="Seconds before logging that launch is still waiting for camera-to-target TF.",
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
        wait_for_tf_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_tf_node,
                on_exit=[
                    tracker_node,
                    pddl_nav2_stl_sat_node,
                    visualizer_node,
                ],
            )
        ),
        rviz_launch,
    ])
