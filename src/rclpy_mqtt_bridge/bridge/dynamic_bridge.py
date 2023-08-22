import rclpy
import json
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

from rclpy.node import Node
from rclpy_mqtt_bridge.mqtt import broker
from rosbridge_library.internal import message_conversion
from typing import List
from typing import Tuple


class dynamic_bridge(Node):
    _rclpy_node_name: str = "rclpy_mqtt_bridge"
    mqtt_manager: broker.mqtt_broker = broker.mqtt_broker()

    def __init__(self) -> None:
        super().__init__(self._rclpy_node_name)
        self.get_logger().info("===== {} created =====".format(self._rclpy_node_name))
        self._bridge()

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().warn("Ctrl-C detected")
            self.mqtt_manager.client.disconnect()
            self.mqtt_manager.client.loop_stop()
            
        self.destroy_node()
    

    def _publisher_to_subscription(self, topic_name: str, topic_type: str):
        publishers: int = self.count_publishers(topic_name)

        if publishers > 0:
            self.get_logger().info("[{}] is a publisher".format(topic_name))
            
            is_ignored_topic_name: bool = (topic_name == "/parameter_events") or (topic_name == "/rosout")
            
            if is_ignored_topic_name:
                self.get_logger().warn("ignoring [{}] publisher".format(topic_name))
                return

            split_topic_type: list[str] = topic_type.split("/", 3)
            parsed_topic_type: str = (
                f"{split_topic_type[0]}.{split_topic_type[1]}.{split_topic_type[2]}"
            )
            self.get_logger().info("parsed topic type : [{}]".format(parsed_topic_type))

            ros_message_type = eval(parsed_topic_type)
            self.create_subscription(
                ros_message_type, topic_name, self._subscription_callback, 10
            )
            self.get_logger().info("[{}] subscription created".format(topic_name))
        else:
            self.get_logger().info("[{}] is not a publisher".format(topic_name))

    def _subscription_callback(self, msg):
        self.get_logger().info("subscription received message : [{}]".format(msg))
        serialized_msgs: str = json.dumps(message_conversion.extract_values(msg))
        self.mqtt_manager.publish(topic="/chatter", payload=serialized_msgs)

    def _bridge(self):
        _rclpy_node_name: str = self.get_name()
        _rclpy_node_namespace: str = self.get_namespace()

        topic_and_types: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()

        for topic_name, topic_type_list in topic_and_types:
            for topic_type in topic_type_list:
                self.get_logger().info(
                    "topics : [{}], type : [{}]".format(topic_name, topic_type)
                )
                self._publisher_to_subscription(topic_name, topic_type)
