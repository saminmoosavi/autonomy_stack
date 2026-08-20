from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # -----------------------------
    # Launch Arguments
    # -----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration("namespace")
    detections_topic = LaunchConfiguration('detections_topic')
    points_topic = LaunchConfiguration('points_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    map_frame = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    follow_distance = LaunchConfiguration('follow_distance')
    search_angular_speed = LaunchConfiguration('search_angular_speed')
    search_duration = LaunchConfiguration('search_duration')

    return LaunchDescription([

        # Topics
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument("namespace", default_value="j100_0612"),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/yolo/detections',
            description='YOLO detections topic'
        ),

        DeclareLaunchArgument(
            'points_topic',
            default_value='/camera/camera_0/depth/color/points',
            description='Point cloud topic'
        ),

        # Frames
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_0_link',
            description='Camera frame (should match camera_info.header.frame_id)'
        ),

        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global frame'
        ),

        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity command topic used for search rotation'
        ),

        # Behavior
        DeclareLaunchArgument(
            'follow_distance',
            default_value='1.5',
            description='Distance to keep from person (meters)'
        ),
        DeclareLaunchArgument(
            'search_angular_speed',
            default_value='0.4',
            description='Angular velocity while rotating to search for a person (rad/s)'
        ),
        DeclareLaunchArgument(
            'search_duration',
            default_value='5.0',
            description='Maximum time to rotate while searching for a person (seconds)'
        ),

        # -----------------------------
        # Node
        # -----------------------------
        Node(
            package='person_follower',
            executable='person_follower',
            name='person_follower',
            namespace=namespace,
            output='screen',
            parameters=[{
                'detections_topic': detections_topic,
                'points_topic': points_topic,
                'camera_frame': camera_frame,
                'map_frame': map_frame,
                'base_frame': base_frame,
                'cmd_vel_topic': cmd_vel_topic,
                'follow_distance': follow_distance,
                'search_angular_speed': search_angular_speed,
                'search_duration': search_duration,
                'namespace': namespace,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/tf', 'tf'), # Remap global /tf to relative tf (becomes /my_namespace/tf)
                ('/tf_static', 'tf_static'), # Remap global /tf_static to relative tf_static
            ],
        )
    ])
