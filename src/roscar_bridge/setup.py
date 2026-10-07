import os
from glob import glob

from setuptools import find_packages, setup

package_name = "roscar_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="ROS2 ↔ ESP32-S3 串口桥接节点（EB90 帧下行 / 文本行上行）",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = roscar_bridge.bridge_node:main",
        ],
    },
)
