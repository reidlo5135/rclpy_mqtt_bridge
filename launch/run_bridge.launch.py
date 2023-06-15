import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('rclpy_mqtt_bridge'),
            'config',
            'demo_params.yaml'
        )

    node=Node(
        package = 'rclpy_mqtt_bridge',
        name = 'rclpy_mqtt_bridge_node',
        executable = 'rclpy_mqtt_bridge_node',
        parameters = [config]
    )

    ld.add_action(node)
    return ld