import rclpy
import rclpy.action
import rcl_interfaces.msg
import std_msgs.msg
import sensor_msgs.msg
import geographic_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import nav2_msgs.msg
import action_msgs.msg

import json
import paho.mqtt.client as mqtt
import importlib

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy import Future
from rclpy.action.client import ActionClient
from rclpy.action import get_action_names_and_types
from rosbridge_library.internal import message_conversion
from rclpy_mqtt_bridge.mqtt import broker
from typing import List
from typing import Tuple
from typing import Any
from typing import Dict


class dynamic_bridge(Node):
    __rclpy_node_name__: str = "rclpy_mqtt_bridge"
    __rclpy_flags__: str = "RCLPY"

    __mqtt_manager__: broker.mqtt_broker = broker.mqtt_broker()
    __mqtt_request_topic_format__: str = "/request"
    __mqtt_response_topic_format__: str = "/response"

    __established_rcl_publishers_list__: List[str] = []
    __established_rcl_subscriptions_list__: List[str] = []
    __established_rcl_service_client_list__: List[str] = []
    __established_rcl_action_client_list__: List[str] = []

    def __init__(self) -> None:
        super().__init__(self.__rclpy_node_name__)
        self.get_logger().info("===== {} [{}] created =====".format(self.__rclpy_flags__, self.__rclpy_node_name__))

        self.__bridge__()
        timer_loop: float = 2.5
        self.create_timer(timer_loop, self.__bridge__)

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().warn("===== {} [{}] terminated with ctrl-c =====".format(self.__rclpy_flags__, self.__rclpy_node_name__))
            self.__mqtt_manager__.client.disconnect()
            self.__mqtt_manager__.client.loop_stop()

        self.destroy_node()

    def __lookup_object__(self, object_path: str) -> Any:
        self.get_logger().info("{} lookup object path : {}".format(self.__rclpy_flags__, object_path))

        split_topic_type: list[str] = object_path.split("/", 3)

        module_name: str = f"{split_topic_type[0]}.{split_topic_type[1]}"
        self.get_logger().info("{} lookup object module_name : {}".format(self.__rclpy_flags__, module_name))

        object_name: str = split_topic_type[2]

        module = importlib.import_module(module_name, self.__rclpy_node_name__)
        obj: Any = getattr(module, object_name)

        return obj

    def __parse_rcl_topic_type__(self, topic_type: str) -> Any:
        split_topic_type: list[str] = topic_type.split("/", 3)

        parsed_topic_type: str = (f"{split_topic_type[0]}.{split_topic_type[1]}.{split_topic_type[2]}")
        self.get_logger().info("{} parsed topic type : [{}]".format(self.__rclpy_flags__, parsed_topic_type))

        parsed_rcl_topic_type: Any = eval(parsed_topic_type)

        return parsed_rcl_topic_type

    def __publisher_to_subscription__(self, rcl_topic_name: str, topic_type: str) -> None:
        publishers: int = self.count_publishers(rcl_topic_name)

        if publishers > 0:
            self.get_logger().info("{} [{}] is a publisher".format(self.__rclpy_flags__, rcl_topic_name))

            is_ignored_topic_name: bool = (rcl_topic_name == "/parameter_events") or (rcl_topic_name == "/rosout")

            if is_ignored_topic_name:
                self.get_logger().warn("{} ignoring [{}] publisher".format(self.__rclpy_flags__, rcl_topic_name))
                return
            elif rcl_topic_name in self.__established_rcl_subscriptions_list__:
                self.get_logger().warn("{} [{}] pub to sub connection is already established... ignoring".format(self.__rclpy_flags__, rcl_topic_name))
                return

            parsed_topic_type: Any = self.__parse_rcl_topic_type__(topic_type)

            def rcl_subscription_callback(rcl_callback_message: Any) -> None:
                mqtt_serialized_message: str = json.dumps(message_conversion.extract_values(rcl_callback_message))
                mqtt_topic_name: str = self.__mqtt_response_topic_format__ + rcl_topic_name
                self.__mqtt_manager__.publish(topic=mqtt_topic_name, payload=mqtt_serialized_message)

            self.create_subscription(parsed_topic_type, rcl_topic_name, rcl_subscription_callback, 10)

            if rcl_topic_name not in self.__established_rcl_subscriptions_list__:
                self.get_logger().info("===== {} [{}] pub to sub connection established =====".format(self.__rclpy_flags__, rcl_topic_name))
                self.__established_rcl_subscriptions_list__.append(rcl_topic_name)
            else:
                return
        else:
            self.get_logger().info("{} [{}] is not a publisher".format(self.__rclpy_flags__, rcl_topic_name))

    def __subscription_to_publisher__(self, rcl_topic_name: str, topic_type: str) -> None:
        subscriptions: int = self.count_subscribers(rcl_topic_name)

        if subscriptions > 0:
            self.get_logger().info("{} [{}] is a subscription".format(self.__rclpy_flags__, rcl_topic_name))

            if rcl_topic_name in self.__established_rcl_publishers_list__:
                self.get_logger().warn("{} [{}] sub to pub connection is already established... ignoring".format(self.__rclpy_flags__, rcl_topic_name))
                return

            parsed_rcl_topic_type: Any = self.__parse_rcl_topic_type__(topic_type)

            rcl_publisher: Publisher = self.create_publisher(parsed_rcl_topic_type, rcl_topic_name, 10)

            def mqtt_subscription_callback(client: mqtt.Client, user_data: Dict, mqtt_message: mqtt.MQTTMessage):
                mqtt_topic: str = mqtt_message.topic
                mqtt_decoded_payload: str = mqtt_message.payload.decode()

                self.get_logger().info("{} MQTT received message [{}] from [{}]".format(self.__rclpy_flags__, mqtt_decoded_payload, mqtt_message.topic))

                is_rcl_mqtt_topic_equals: bool = rcl_publisher.topic in mqtt_topic

                if is_rcl_mqtt_topic_equals == False:
                    self.get_logger().error("{} topic RCL & MQTT topics is not matching each other")
                    return
                else:
                    rcl_deserialized_message: Any = json.loads(mqtt_message.payload)
                    self.get_logger().info("{} deserialized message : [{}]".format(self.__rclpy_flags__, rcl_deserialized_message))
                    rcl_message_type: Any = self.__lookup_object__(topic_type)

                    created_rcl_messages: Any = message_conversion.populate_instance(rcl_deserialized_message, rcl_message_type())
                    rcl_publisher.publish(created_rcl_messages)

            if rcl_topic_name not in self.__established_rcl_publishers_list__:
                self.get_logger().info("===== {} [{}] sub to pub connection established =====".format(self.__rclpy_flags__, rcl_topic_name))
                self.__established_rcl_publishers_list__.append(rcl_topic_name)
                
                mqtt_topic_name: str = self.__mqtt_request_topic_format__ + rcl_topic_name
                self.__mqtt_manager__.subscribe(mqtt_topic_name)
                self.__mqtt_manager__.client.message_callback_add(mqtt_topic_name, mqtt_subscription_callback)
            else:
                return
        else:
            self.get_logger().info("{} [{}] is not a subscription".format(self.__rclpy_flags__, rcl_topic_name))

    def __service_request__(self, service_name: str, service_type: str) -> None:
        is_ignoring_required_service_server: bool = "parameter" in service_name

        if is_ignoring_required_service_server:
            self.get_logger().warn("{} ignoring [{}] service client...".format(self.__rclpy_flags__, service_name))
            return
        elif service_name in self.__established_rcl_service_client_list__:
            self.get_logger().warn("{} [{}] service client connection is already established... ignoring".format(self.__rclpy_flags__, service_name))
            return

        parsed_rcl_service_type: Any = self.__parse_rcl_topic_type__(service_type)
        
        rcl_service_client: Client = self.create_client(parsed_rcl_service_type, service_name)

        def mqtt_subscription_callback(client: mqtt.Client, user_data: Dict, mqtt_message: mqtt.MQTTMessage):
            mqtt_topic: str = mqtt_message.topic
            mqtt_decoded_payload: str = mqtt_message.payload.decode()

            self.get_logger().info("{} MQTT service request received message [{}] from [{}]".format(self.__rclpy_flags__, mqtt_decoded_payload, mqtt_message.topic))

            is_rcl_mqtt_topic_equals: bool = rcl_service_client.srv_name in mqtt_topic

            if is_rcl_mqtt_topic_equals == False:
                self.get_logger().error("{} service request RCL & MQTT topics is not matching each other")
                return
            else:
                rcl_deserialized_request_message: Any = json.loads(mqtt_message.payload)
                self.get_logger().info("{} service request deserialized message : [{}]".format(self.__rclpy_flags__, rcl_deserialized_request_message))

                rcl_message_type: Any = self.__lookup_object__(service_type)
                
                service_server_waiting_time: float = 1.0
                
                is_service_server_ready: bool = rcl_service_client.wait_for_service(service_server_waiting_time)
                
                while not is_service_server_ready:
                    self.get_logger().error("{} [{}] service server is not ready yet...".format(self.__rclpy_flags__, service_name))

                    mqtt_response_topic: str = (self.__mqtt_response_topic_format__ + service_name)
                    mqtt_service_server_error_message: str = ("{} service server is not ready".format(service_name))
                    self.__mqtt_manager__.publish(mqtt_response_topic, mqtt_service_server_error_message)
                    break
                
                self.get_logger().info("{} calling to [{}] service server".format(self.__rclpy_flags__, service_name))
                
                created_rcl_messages: Any = message_conversion.populate_instance(rcl_deserialized_request_message, rcl_message_type())
                service_request = created_rcl_messages.Request()
                service_future: Future = rcl_service_client.call_async(service_request)
                service_call_async_result: Any = service_future.result()

                mqtt_serialized_response_message: str = json.dumps(message_conversion.extract_values(service_call_async_result))
                mqtt_response_topic: str = (self.__mqtt_response_topic_format__ + service_name)
                self.__mqtt_manager__.publish(mqtt_response_topic, mqtt_serialized_response_message)
            if service_name not in self.__established_rcl_publishers_list__:
                self.get_logger().info("===== {} [{}] service client connection established =====".format(self.__rclpy_flags__, service_name))
                self.__established_rcl_service_client_list__.append(service_name)
                self.__mqtt_manager__.subscribe(service_name)
                self.__mqtt_manager__.client.message_callback_add(
                    service_name, mqtt_subscription_callback
                )
            else:
                return

    def __action_request__(self, action_name: str, action_type: str) -> None:
        if action_name in self.__established_rcl_action_client_list__:
            self.get_logger().warn("{} [{}] action client connection is already established... ignoring".format(self.__rclpy_flags__, action_name))
            return

        parsed_rcl_action_type: Any = self.__parse_rcl_topic_type__(action_type)
        
        rcl_action_client: ActionClient = ActionClient(self, parsed_rcl_action_type, action_name)

        def mqtt_subscription_callback(client: mqtt.Client, user_data: Dict, mqtt_message: mqtt.MQTTMessage):
            mqtt_topic: str = mqtt_message.topic
            mqtt_decoded_payload: str = mqtt_message.payload.decode()

            self.get_logger().info("{} MQTT action send goal received message [{}] from [{}]".format(self.__rclpy_flags__, mqtt_decoded_payload, mqtt_message.topic))

            is_rcl_mqtt_topic_equals: bool = rcl_action_client._action_name in mqtt_topic
            
            if is_rcl_mqtt_topic_equals == False:
                self.get_logger().error("{} action send goal RCL & MQTT topics is not matching each other")
                return
            else:
                rcl_deserialized_request_message: Any = json.loads(mqtt_message.payload)
                self.get_logger().info("{} action send goal deserialized message : [{}]".format(self.__rclpy_flags__, rcl_deserialized_request_message))

                rcl_message_type: Any = self.__lookup_object__(action_type)
                
                action_server_waiting_time: float = 1.0
                is_action_server_ready: Any = rcl_action_client.wait_for_server(action_server_waiting_time)

                if not is_action_server_ready:
                    self.get_logger().error("{} [{}] action server is not ready yet...".format(self.__rclpy_flags__, action_name))

                    mqtt_response_topic: str = (self.__mqtt_response_topic_format__ + action_name)
                    mqtt_action_server_error_message: str = ("{} action server is not ready".format(action_name))
                    self.__mqtt_manager__.publish(mqtt_response_topic, mqtt_action_server_error_message)
                    return
                else:
                    created_rcl_messages: Any = message_conversion.populate_instance(rcl_deserialized_request_message, rcl_message_type())

                    action_goal = created_rcl_messages.Goal()
                    action_future: Future = rcl_action_client.send_goal_async(action_goal)
                    action_send_goal_result: Any = action_future.result()

                    mqtt_serialized_response_message: str = json.dumps(message_conversion.extract_values(action_send_goal_result))
                    mqtt_response_topic: str = (self.__mqtt_response_topic_format__ + action_name)
                    self.__mqtt_manager__.publish(mqtt_response_topic, mqtt_serialized_response_message)
            if action_name not in self.__established_rcl_publishers_list__:
                self.get_logger().info("===== {} [{}] action client connection established =====".format(self.__rclpy_flags__, action_name))
                
                self.__established_rcl_action_client_list__.append(action_name)
                
                mqtt_topic_name: str = self.__mqtt_request_topic_format__ + action_name
                self.__mqtt_manager__.subscribe(mqtt_topic_name)
                self.__mqtt_manager__.client.message_callback_add(mqtt_topic_name, mqtt_subscription_callback)
            else:
                return

    def __bridge__(self) -> None:
        __rclpy_node_name__: str = self.get_name()
        __rclpy_node_namespace__: str = self.get_namespace()

        topic_and_types: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()

        for topic_name, topic_type_list in topic_and_types:
            for topic_type in topic_type_list:
                self.get_logger().info("{} topics : [{}], type : [{}]".format(self.__rclpy_flags__, topic_name, topic_type))
                self.__publisher_to_subscription__(topic_name, topic_type)
                self.__subscription_to_publisher__(topic_name, topic_type)


        service_and_types: List[Tuple[str, List[str]]] = self.get_service_names_and_types()

        for service_name, service_type_list in service_and_types:
            for service_type in service_type_list:
                self.get_logger().info("{} service : [{}], type : [{}]".format(self.__rclpy_flags__, service_name, service_type))
                self.__service_request__(service_name, service_type)


        action_and_types: List[Tuple[str, List[str]]] = get_action_names_and_types(self)

        for action_name, action_type_list in action_and_types:
            for action_type in action_type_list:
                self.get_logger().info("{} action : [{}], type : [{}]".format(self.__rclpy_flags__, action_name, action_type))



__all__ = ["dynamic_bridge"]
