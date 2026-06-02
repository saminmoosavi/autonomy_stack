from glob import glob

from setuptools import find_packages, setup


package_name = "waypoint_follower"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/trajectories", glob("trajectories/*") + ["trajectories/.gitkeep"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samin Moosavi",
    maintainer_email="saminmoosavi@yahoo.com",
    description="Trajectory recorder and pure pursuit waypoint follower for Clearpath Warthog.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "ekf_comparison_plotter = waypoint_follower.ekf_comparison_plotter:main",
            "gps_imu_ekf_localizer = waypoint_follower.gps_imu_ekf_localizer:main",
            "trajectory_recorder = waypoint_follower.trajectory_recorder:main",
            "pure_pursuit_follower = waypoint_follower.pure_pursuit_follower:main",
            "trajectory_plotter = waypoint_follower.trajectory_plotter:main",
            "trajectory_record_plotter = waypoint_follower.trajectory_record_plotter:main",
        ],
    },
)
