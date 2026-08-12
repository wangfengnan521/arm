from setuptools import setup
import os
from glob import glob

package_name = "x5a_control_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="x5a user",
    maintainer_email="user@example.com",
    description="ARX X5A control bridge",
    license="BSD",
    entry_points={
        "console_scripts": [
            "control_bridge = x5a_control_bridge.bridge_node:main",
            "test_follow_joint_trajectory = x5a_control_bridge.test_follow_joint_trajectory:main",
            "test_moveit_execute = x5a_control_bridge.test_moveit_execute:main",
        ],
    },
)
