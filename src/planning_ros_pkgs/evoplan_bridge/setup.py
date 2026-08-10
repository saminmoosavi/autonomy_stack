from setuptools import find_packages, setup

package_name = 'evoplan_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="Samin Moosavi",
    maintainer_email="saminmoosavi@yahoo.com",
    description="Online EvoPlan bridge: Phi_mob runtime shield and symbolic-replan client",
    license="BSD-3-Clause",
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # No console_scripts on purpose: this package is a library imported by
    # evo_skill_ros's evo_plan_deploy node, not a node of its own.
)
