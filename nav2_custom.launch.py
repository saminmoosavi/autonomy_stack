from ament_index_python.packages import get_package_share_directory

from clearpath_config.common.utils.yaml import read_yaml
from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
    DeclareLaunchArgument('params_file',
                          default_value='/home/user/autonomy_stack_ros_humble/j100_nav2.yaml',
                          description='Nav2 params file override'),
]


def launch_setup(context, *args, **kwargs):
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    params_file = LaunchConfiguration('params_file')

    config = read_yaml(setup_path.perform(context) + 'robot.yaml')
    clearpath_config = ClearpathConfig(config)
    namespace = clearpath_config.system.namespace

    launch_nav2 = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        PushRosNamespace(namespace),
        SetRemap('/' + namespace + '/global_costmap/sensors/lidar2d_0/scan',
                 '/' + namespace + '/sensors/lidar2d_0/scan'),
        SetRemap('/' + namespace + '/local_costmap/sensors/lidar2d_0/scan',
                 '/' + namespace + '/sensors/lidar2d_0/scan'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('params_file', params_file),
                ('use_composition', 'False'),
                ('namespace', namespace),
            ]
        ),
    ])

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
