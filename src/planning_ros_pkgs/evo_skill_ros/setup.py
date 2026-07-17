from glob import glob
from os.path import isfile

from setuptools import find_packages, setup

package_name = 'evo_skill_ros'


def package_files(pattern):
    return [path for path in glob(pattern) if isfile(path)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', package_files('config/*')),
        ('share/' + package_name + '/launch', package_files('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', package_files('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="Samin Moosavi",
    maintainer_email="saminmoosavi@yahoo.com",
    description="Neuro-Symbolic Robot Planning with Evolved Spatio-Temporal Guarantees",
    license="BSD-3-Clause",
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pddl_stl_evolution = evo_skill_ros.pddl_stl.pipeline:main',
            'pddl_stl_nav2 = evo_skill_ros.nodes.pddl_stl_nav2_node:main',
            'pddl_nav2_stl_sat = evo_skill_ros.nodes.pddl_nav2_stl_sat:main',
            'evo_plan_deploy = evo_skill_ros.nodes.eveo_plan_deploy:main',
            'factory_world_visualizer = evo_skill_ros.nodes.factory_world_visualizer:main',
            'tracker_with_yolo = evo_skill_ros.nodes.tracker_with_yolo:main',
            'wait_for_tf = evo_skill_ros.nodes.wait_for_tf:main',
            'scand_metrics = evo_skill_ros.nodes.scand_metrics:main',
        ],
    },
)
