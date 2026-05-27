from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    pkg_clearpath_gz = FindPackageShare('clearpath_gz')

    gz_sim_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'gz_sim.launch.py']
    )

    robot_spawn_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py']
    )

    # -----------------------------
    # 1. Launch Gazebo ONCE
    # -----------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch])
    )

    # -----------------------------
    # 2. Spawn Robot 1
    # -----------------------------
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments={
            'setup_path': '/home/user/clearpath',
            'generate': 'true',
            'rviz': 'false',
        }.items()
    )

    # -----------------------------
    # 3. Spawn Robot 2 (DELAYED)
    # -----------------------------
    robot2 = TimerAction(
        period=8.0,   # IMPORTANT: avoid collision with generator
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([robot_spawn_launch]),
                launch_arguments={
                    'setup_path': '/home/user/clearpath_2',
                    'generate': 'true',
                    'rviz': 'false',
                    'x': '3.0',
                    'y': '3.0',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        gz_sim,
        robot1,
        robot2,
    ])
