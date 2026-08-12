from glob import glob
from setuptools import setup

package_name = "x5a_moveit_official_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="x5a user",
    maintainer_email="user@example.com",
    description="MoveIt standard trajectory adapter for the official ARX X5 RobotCmd/RobotStatus API",
    license="BSD",
    entry_points={
        "console_scripts": [
            "official_trajectory_adapter = x5a_moveit_official_adapter.official_trajectory_adapter:main",
        ]
    },
)
