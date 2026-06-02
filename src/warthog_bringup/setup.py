from glob import glob
from os.path import isfile

from setuptools import find_packages, setup

package_name = "warthog_bringup"


def package_files(pattern):
    return [path for path in glob(pattern) if isfile(path)]


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", package_files("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samin Moosavi",
    maintainer_email="saminmoosavi@yahoo.com",
    description="Combined Clearpath Nav2, SLAM, and RViz bringup for the Warthog.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "lio_sam_sensor_relay = warthog_bringup.lio_sam_sensor_relay:main",
            "scan_relay = warthog_bringup.scan_relay:main",
        ],
    },
)
