from setuptools import setup
import os
from glob import glob

package_name = "gazebo_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # ("share/" + package_name + "/launch", ["launch/*.launch.py"]),
        # Include the launch files in the package
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="Gazebo node for ROS2 workspace",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gazebo_node = gazebo_node.gazebo_node:main",
        ],
    },
)
