import rclpy
import json
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

from rclpy.node import Node
from rosbridge_library.internal import message_conversion
from rclpy_mqtt_bridge.mqtt import broker
from typing import List
from typing import Tuple


class dynamic_bridge(Node):
    __rclpy_node_name__: str = "rclpy_mqtt_bridge"
    __mqtt_manager__: broker.mqtt_broker = broker.mqtt_broker()
    __established_ros_topic_list__: List[str] = []

    def __init__(self) -> None:
        super().__init__(self.__rclpy_node_name__)
        self.get_logger().info(
            "===== {} created =====".format(self.__rclpy_node_name__)
        )
        
        self.__bridge__()
        timer_loop: float = 5.0
        self.create_timer(timer_loop, self.__bridge__)

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().warn("Ctrl-C detected")
            self.__mqtt_manager__.client.disconnect()
            self.__mqtt_manager__.client.loop_stop()

        self.destroy_node()

    def __publisher_to_subscription__(self, topic_name: str, topic_type: str):
        publishers: int = self.count_publishers(topic_name)

        if publishers > 0:
            self.get_logger().info("[{}] is a publisher".format(topic_name))

            is_ignored_topic_name: bool = (topic_name == "/parameter_events") or (topic_name == "/rosout")

            if is_ignored_topic_name:
                self.get_logger().warn("ignoring [{}] publisher".format(topic_name))
                return
            elif topic_name in self.__established_ros_topic_list__:
                self.get_logger().warn("[{}] publisher is already established... ignoring".format(topic_name))
                return

            split_topic_type: list[str] = topic_type.split("/", 3)
            parsed_topic_type: str = (
                f"{split_topic_type[0]}.{split_topic_type[1]}.{split_topic_type[2]}"
            )
            self.get_logger().info("parsed topic type : [{}]".format(parsed_topic_type))

            ros_message_type = eval(parsed_topic_type)
            self.create_subscription(
                ros_message_type, topic_name, self.__subscription_callback__, 10
            )
            
            if topic_name not in self.__established_ros_topic_list__:
                self.get_logger().info("===== [{}] pub to sub connection established =====".format(topic_name))
                self.__established_ros_topic_list__.append(topic_name)
            else:
                return
                
        else:
            self.get_logger().info("[{}] is not a publisher".format(topic_name))

    def __subscription_callback__(self, ros_message):
        self.get_logger().info("subscription received message : [{}]".format(ros_message))
        serialized_ros_message: str = json.dumps(message_conversion.extract_values(ros_message))
        self.__mqtt_manager__.publish(topic="/chatter", payload=serialized_ros_message)

    def __bridge__(self):
        __rclpy_node_name__: str = self.get_name()
        __rclpy_node_namespace__: str = self.get_namespace()

        topic_and_types: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()

        for topic_name, topic_type_list in topic_and_types:
            for topic_type in topic_type_list:
                self.get_logger().info(
                    "topics : [{}], type : [{}]".format(topic_name, topic_type)
                )
                self.__publisher_to_subscription__(topic_name, topic_type)
