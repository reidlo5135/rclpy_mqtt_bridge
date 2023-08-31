import os
from setuptools import setup

package_name = "rclpy_mqtt_bridge"
bridge_package_name = package_name + ".bridge"
mqtt_package_name = package_name + ".mqtt"

setup(
    name=package_name,
    version="0.1.1",
    packages=[package_name, bridge_package_name, mqtt_package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    package_dir={"": "src"},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reidlo",
    maintainer_email="naru5135@wavem.net",
    description="ROS2(foxy) [RCLPY - MQTT] bridge server",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dynamic_bridge = rclpy_mqtt_bridge.main:main",
        ],
    },
)
