import rclpy
import json
import paho.mqtt.client as mqtt
import std_msgs.msg
import rcl_interfaces
import sensor_msgs.msg
import geometry_msgs.msg

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rosbridge_library.internal import message_conversion
from rclpy_mqtt_bridge.mqtt import broker
from importlib import import_module
from typing import List
from typing import Tuple
from typing import Any
from typing import Dict


class dynamic_bridge(Node):
    __rclpy_node_name__: str = "rclpy_mqtt_bridge"
    __rclpy_flags__: str = "RCLPY"
    __mqtt_manager__: broker.mqtt_broker = broker.mqtt_broker()
    __mqtt_response_topic_format: str = "/mqtt"
    __established_rcl_publishers_list__: List[str] = []
    __established_rcl_subscriptions_list__: List[str] = []

    def __init__(self) -> None:
        super().__init__(self.__rclpy_node_name__)
        self.get_logger().info(
            "===== {} [{}] created =====".format(self.__rclpy_flags__, self.__rclpy_node_name__)
        )
        
        self.__bridge__()
        timer_loop: float = 3.0
        self.create_timer(timer_loop, self.__bridge__)

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().warn("{} [{}] terminated with ctrl-c".format(self.__rclpy_flags__, self.__rclpy_node_name__))
            self.__mqtt_manager__.client.disconnect()
            self.__mqtt_manager__.client.loop_stop()

        self.destroy_node()
    
    def __lookup_object__(self, object_path: str) -> Any:
        self.get_logger().info("{} lookup object path : {}".format(self.__rclpy_flags__, object_path))
                
        split_topic_type: list[str] = object_path.split("/", 3)
                
        module_name: str = (f"{split_topic_type[0]}.{split_topic_type[1]}")
        self.get_logger().info("{} lookup object module_name : {}".format(self.__rclpy_flags__, module_name))
        
        object_name: str = split_topic_type[2]
                
        module = import_module(module_name, self.__rclpy_node_name__)
        obj: Any = getattr(module, object_name)
                
        return obj
            
    def __parse_rcl_topic_type__(self, topic_type: str) -> Any:
        split_topic_type: list[str] = topic_type.split("/", 3)
        parsed_topic_type: str = (
            f"{split_topic_type[0]}.{split_topic_type[1]}.{split_topic_type[2]}"
        )
        self.get_logger().info("{} parsed topic type : [{}]".format(self.__rclpy_flags__, parsed_topic_type))

        parsed_rcl_topic_type: Any = eval(parsed_topic_type)
        
        return parsed_rcl_topic_type

    def __publisher_to_subscription__(self, topic_name: str, topic_type: str) -> None:
        publishers: int = self.count_publishers(topic_name)

        if publishers > 0:
            self.get_logger().info("{} [{}] is a publisher".format(self.__rclpy_flags__, topic_name))

            is_ignored_topic_name: bool = (topic_name == "/parameter_events") or (topic_name == "/rosout")

            if is_ignored_topic_name:
                self.get_logger().warn("{} ignoring [{}] publisher".format(self.__rclpy_flags__, topic_name))
                return
            elif topic_name in self.__established_rcl_subscriptions_list__:
                self.get_logger().warn("{} [{}] pub to sub connection is already established... ignoring".format(self.__rclpy_flags__, topic_name))
                return
            
            parsed_rcl_topic_type: Any = self.__parse_rcl_topic_type__(topic_type)
            
            def __rcl_subscription_callback__(ros_message) -> None:
                # self.get_logger().info("subscription received message : [{}]".format(ros_message))
                rcl_serialized_message: str = json.dumps(message_conversion.extract_values(ros_message))
                mqtt_topic_name: str = self.__mqtt_response_topic_format + topic_name
                self.__mqtt_manager__.publish(topic=mqtt_topic_name, payload=rcl_serialized_message)
        
            rcl_subscription: Subscription = self.create_subscription(
                parsed_rcl_topic_type, topic_name, __rcl_subscription_callback__, 10
            )
            
            if topic_name not in self.__established_rcl_subscriptions_list__:
                self.get_logger().info("===== {} [{}] pub to sub connection established =====".format(self.__rclpy_flags__, topic_name))
                self.__established_rcl_subscriptions_list__.append(topic_name)
            else:
                return
        else:
            self.get_logger().info("{} [{}] is not a publisher".format(self.__rclpy_flags__, topic_name))

    def __subscription_to_publisher__(self, topic_name: str, topic_type: str) -> None:
        subscriptions: int = self.count_subscribers(topic_name)
        
        if subscriptions > 0:
            self.get_logger().info("{} [{}] is a subscription".format(self.__rclpy_flags__, topic_name))
            
            if topic_name in self.__established_rcl_publishers_list__:
                self.get_logger().warn("{} [{}] sub to pub connection is already established... ignoring".format(self.__rclpy_flags__, topic_name))
                return
            
            parsed_rcl_topic_type: Any = self.__parse_rcl_topic_type__(topic_type)
            
            rcl_publisher: Publisher = self.create_publisher(parsed_rcl_topic_type, topic_name, 10)
            
            def mqtt_subscription_callback(client: mqtt.Client, user_data: Dict, mqtt_message: mqtt.MQTTMessage):
                self.get_logger().info(
                    "{} MQTT received message : {}".format(self.__rclpy_flags__, mqtt_message.payload.decode())
                )
                
                rcl_deserialized_message: Any = json.loads(mqtt_message.payload)
                self.get_logger().info("{} deserialized message : [{}]".format(self.__rclpy_flags__, rcl_deserialized_message))
                rcl_message_type: Any = self.__lookup_object__(topic_type)
                
                created_rcl_messages: Any = message_conversion.populate_instance(rcl_deserialized_message, rcl_message_type())
                rcl_publisher.publish(created_rcl_messages)
            
            if topic_name not in self.__established_rcl_publishers_list__:
                self.get_logger().info("===== {} [{}] sub to pub connection established =====".format(self.__rclpy_flags__, topic_name))
                self.__established_rcl_publishers_list__.append(topic_name)
                self.__mqtt_manager__.subscribe(topic_name)
                self.__mqtt_manager__.client.message_callback_add(topic_name, mqtt_subscription_callback)
            else:
                return
        else :
            self.get_logger().info("{} [{}] is not a subscription".format(self.__rclpy_flags__, topic_name))
    
    def __bridge__(self) -> None:
        __rclpy_node_name__: str = self.get_name()
        __rclpy_node_namespace__: str = self.get_namespace()

        topic_and_types: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()

        for topic_name, topic_type_list in topic_and_types:
            for topic_type in topic_type_list:
                self.get_logger().info(
                    "{} topics : [{}], type : [{}]".format(self.__rclpy_flags__, topic_name, topic_type)
                )
                self.__publisher_to_subscription__(topic_name, topic_type)
                self.__subscription_to_publisher__(topic_name, topic_type)


__all__ = ['dynamic_bridge']