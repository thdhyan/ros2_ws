from setuptools import setup
import os
from glob import glob

package_name = "turtlebot_node"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/spawn_turtlebot.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS Maintainer",
    maintainer_email="ros@todo.todo",
    description="ROS2 node for Turtlebot control and navigation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "turtlebot = turtlebot_node.turtlebot:main"
        ],
    },
)
