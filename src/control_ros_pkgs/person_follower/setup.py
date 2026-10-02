from setuptools import find_packages, setup
from glob import glob

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", glob('launch/*.launch.py')),
        ('share/' + package_name + "/config", glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samin Moosavi',
    maintainer_email='saminmoosavi@yahoo.com',
    description='A person follower integrated with Nav2',
    license='BSD-3-Clause',

    entry_points={
        'console_scripts': [
            "person_follower = person_follower.nodes.person_follower:main",
            "save_trajectory = person_follower.nodes.save_trajectory:main",
            "waypoint_follower = person_follower.nodes.waypoint_follower:main",

        ],
    },
)
