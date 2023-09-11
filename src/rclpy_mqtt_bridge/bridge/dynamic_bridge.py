import rclpy
import rclpy.action
import rcl_interfaces.msg
import diagnostic_msgs.msg
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


""" Description
    
    This class for register & execute rclpy node what provides bridge logic between rclpy - mqtt by extending rclpy.Node.
    
    Attributes:
        - __rclpy_node_name__: rclpy node's name (str)
        - __rclpy_flags_: logging flag of rclpy (str)
        - __mqtt_manager__: broker.mqtt_broker class' instance (broker.mqtt_broker)
        - __mqtt_request_topic_format__: default format for mqtt publisher address (str)
        - __mqtt_response_topic_format__: default format for mqtt subscription address (str)
        - __established_rcl_publishers_list__: rclpy publishers' topics list what is already established bridge connections between rclpy - mqtt (List[str])
        - __established_rcl_publishers_dict__: rclpy publishers dict what is already established bridge connections between rclpy - mqtt (Dict)
        - __established_rcl_subscriptions_list__: rclpy subscriptions' topics list what is already established bridge connections between rclpy - mqtt (List[str])
        - __established_rcl_subscriptions_dict__: rclpy subscriptions dict what is already established bridge connections between rclpy - mqtt (Dict)
        - __established_rcl_service_clients_list__: rclpy service clients' topics list what is already established bridge connections between rclpy - mqtt (List[str])
        - __established_rcl_service_clients_dict__: rclpy service clients dict what is already established bridge connections between rclpy - mqtt (Dict)
        - __established_rcl_action_clients_list__: rclpy action clients' topics list what is already established bridge connections between rclpy - mqtt (List[str])
        - __established_rcl_action_clients_dict__: rclpy action clients dict what is already established bridge connections between rclpy - mqtt (Dict)
    
    Methods:
        - __init__: Constructor method for this class. Initialize rclpy by self.__rclpy_node_name__ and invoke self.__brige__() and eventually execute initialized rclpy.Node by invoking rclpy.spin()
        - __lookup_object__: Look up module's object by object_path.
        - __cast_topic_type_to_topic_class__: Cast topic_type(str) to topic_class(rclpy message class).
        - __publisher_to_subscription__: Make connection bridge between current ROS2 publisher into rclpy subscription when count by publisher's topic name is larger than 0.
        - __subscription_to_publisher__: Make connection bridge between current ROS2 subsription into rclpy publisher when count by subscription's topic name is larger than 0.
        - __service_to_client__: Make connection bridge between current ROS2 service server into rclpy service client when count by service server's topic name is larger than 0.
        - __action_to_client__: Make connection bridge between current ROS2 action server into rclpy action client when count by action server's topic name is larger than 0.
        - __bridge__: Look up current ROS2 topic / service / action and invoke each connection bridge methods.
"""
class dynamic_bridge(Node):
    __rclpy_node_name__: str = "rclpy_mqtt_bridge"
    __rclpy_flags__: str = "RCLPY"

    __mqtt_manager__: broker.mqtt_broker = broker.mqtt_broker()
    __mqtt_request_topic_format__: str = "wavem/atcplus/rms"
    __mqtt_response_topic_format__: str = "wavem/atcplus/ros"

    __established_rcl_publishers_list__: List[str] = []
    __established_rcl_publishers_dict__: Dict = {}
    
    __established_rcl_subscriptions_list__: List[str] = []
    __established_rcl_subscriptions_dict__: Dict = {}
    
    __established_rcl_service_clients_list__: List[str] = []
    __established_rcl_service_clients_dict__: Dict = {}
    
    __established_rcl_action_client_list__: List[str] = []
    __established_rcl_action_clients_dict__: Dict = {}

    def __init__(self) -> None:
        
        """ Description
        
        Constructor method for this class.
        
        Initialize rclpy by self.__rclpy_node_name__ and invoke self.__brige__() and eventually execute initialized rclpy.Node by invoking rclpy.spin()
        
        Args:
            - self: This class' instance
        
        Returns:
            - None
        
        Usage:
            try:
                dynamic_bridge()
            except ROSInterruptException:
                pass
            rclpy.shutdown()
        """
            
        super().__init__(self.__rclpy_node_name__)
        self.get_logger().info("===== {} [{}] created =====".format(self.__rclpy_flags__, self.__rclpy_node_name__))

        self.__bridge__()
        timer_loop: float = 2.0
        self.create_timer(timer_loop, self.__bridge__)

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().warn("===== {} [{}] terminated with ctrl-c =====".format(self.__rclpy_flags__, self.__rclpy_node_name__))
            self.__mqtt_manager__.client.disconnect()
            self.__mqtt_manager__.client.loop_stop()

        self.destroy_node()


    def __lookup_object__(self, object_path: str) -> Any:
    
        """ Description
        
        Look up module's object by object_path.
        
        Args:
            - self: This class' instance
            - object_path: target import module's path (str)

        Returns:
            - obj: imported module's object (Any)
            
        Usage:
            rcl_message_type: Any = self.__lookup_object__(topic_type)
        
        """
        
        self.get_logger().info("{} lookup object path : {}".format(self.__rclpy_flags__, object_path))

        split_topic_type: list[str] = object_path.split("/", 3)

        module_name: str = f"{split_topic_type[0]}.{split_topic_type[1]}"
        self.get_logger().info("{} lookup object module_name : {}".format(self.__rclpy_flags__, module_name))

        object_name: str = split_topic_type[2]

        module = importlib.import_module(module_name, self.__rclpy_node_name__)
        obj: Any = getattr(module, object_name)

        return obj


    def __cast_topic_type_to_topic_class__(self, topic_type: str) -> Any:
        
        """ Description
        
        Cast topic_type(str) to topic_class(rclpy message class).
        
        Args:
            - self: This class' instance
            - topic_type: casting target rclpy topic_type(str)

        Returns:
            - casted_topic_class: casted rclpy message class (Any)
                
        Usage:
            rcl_message_class: Any = self.__cast_topic_type_to_topic_class__(topic_type)
        """
        topic_type_split_arr: list[str] = topic_type.split("/", 3)

        topic_type_module_name: str = (f"{topic_type_split_arr[0]}.{topic_type_split_arr[1]}.{topic_type_split_arr[2]}")
        self.get_logger().info("{} parsed topic type : [{}]".format(self.__rclpy_flags__, topic_type_module_name))

        casted_topic_class: Any = eval(topic_type_module_name)

        return casted_topic_class


    def __publisher_to_subscription__(self, rcl_topic_name: str, rcl_topic_type: str) -> None:
    
        """ Description
        
        Make connection bridge between current ROS2 publisher into rclpy subscription when count by publisher's topic name is larger than 0.
        
        Args:
            - self: This class' instance
            - rcl_topic_name: target ROS2 publisher topic name (str)
            - rcl_topic_type: target ROS2 publisher topic type (str)

        Returns:
            None
            
        Usage:
            rcl_topic_and_types_list: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()
            
            for rcl_topic_name, rcl_topic_type_list in rcl_topic_and_types_list:
                for topic_type in topic_type_list:
                    self.__publisher_to_subscription__(rcl_topic_name, rcl_topic_type)
        """
        
        rcl_publishers_count: int = self.count_publishers(rcl_topic_name)
        
        is_ignored_rcl_topic_name: bool = (rcl_topic_name == "/parameter_events") or (rcl_topic_name == "/rosout")

        if rcl_publishers_count > 0:
            self.get_logger().info("{} [{}] is a publisher".format(self.__rclpy_flags__, rcl_topic_name))

            if is_ignored_rcl_topic_name:
                self.get_logger().warn("{} ignoring [{}] publisher".format(self.__rclpy_flags__, rcl_topic_name))
                return
            elif rcl_topic_name in self.__established_rcl_subscriptions_list__:
                self.get_logger().warn("{} [{}] pub to sub connection is already established... ignoring".format(self.__rclpy_flags__, rcl_topic_name))
                return

            rcl_message_class: Any = self.__cast_topic_type_to_topic_class__(rcl_topic_type)

            def rcl_subscription_callback(rcl_callback_message: Any) -> None:
                
                """ Description
                
                Publish to MQTT subscription with rclpy subscription callback data after serialize rclpy subscription callback data into json.
                
                Args:
                    - self: This class' instance
                    - rcl_callback_message: returned rclpy subscription callback data (Any)
                Returns:
                    None
                    
                Usage:
                    established_rcl_subscription: Subscription = self.create_subscription(rcl_message_class, rcl_topic_name, rcl_subscription_callback, 10)
                """
                
                mqtt_serialized_message: str = json.dumps(message_conversion.extract_values(rcl_callback_message))
                mqtt_topic_name: str = self.__mqtt_response_topic_format__ + rcl_topic_name
                self.__mqtt_manager__.publish(topic=mqtt_topic_name, payload=mqtt_serialized_message)
            
            established_rcl_subscription: Subscription = self.create_subscription(rcl_message_class, rcl_topic_name, rcl_subscription_callback, 10)

            if rcl_topic_name not in self.__established_rcl_subscriptions_list__:
                self.get_logger().info("===== {} [{}] pub to sub connection established =====".format(self.__rclpy_flags__, rcl_topic_name))
                self.__established_rcl_subscriptions_list__.append(rcl_topic_name)
                self.__established_rcl_subscriptions_dict__[rcl_topic_name] = established_rcl_subscription
            else:
                return
        else:
            self.get_logger().info("{} [{}] is not a publisher".format(self.__rclpy_flags__, rcl_topic_name))
            return


    def __subscription_to_publisher__(self, rcl_topic_name: str, topic_type: str) -> None:
        
        """ Description
        
        Make connection bridge between current ROS2 subsription into rclpy publisher when count by subscription's topic name is larger than 0.
        
        Args:
            - self: This class' instance
            - rcl_topic_name: target ROS2 subscription topic name (str)
            - rcl_topic_type: target ROS2 subscription topic type (str)

        Returns:
            None
            
        Usage:
            rcl_topic_and_types_list: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()
            
            for rcl_topic_name, rcl_topic_type_list in rcl_topic_and_types_list:
                for rcl_topic_type in rcl_topic_type_list:
                    self.__subscription_to_publisher__(rcl_topic_name, rcl_topic_type)
        """
        
        rcl_subscriptions_count: int = self.count_subscribers(rcl_topic_name)

        if rcl_subscriptions_count > 0:
            self.get_logger().info("{} [{}] is a subscription".format(self.__rclpy_flags__, rcl_topic_name))

            if rcl_topic_name in self.__established_rcl_publishers_list__:
                self.get_logger().warn("{} [{}] sub to pub connection is already established... ignoring".format(self.__rclpy_flags__, rcl_topic_name))
                return

            parsed_rcl_topic_type: Any = self.__cast_topic_type_to_topic_class__(topic_type)

            established_rcl_publisher: Publisher = self.create_publisher(parsed_rcl_topic_type, rcl_topic_name, 10)

            def mqtt_subscription_callback(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
                
                """ Description
                
                Publish to ROS2 subscription with mqtt subscription callback message data after deserialize json into rclpy message class.
                
                Args:
                    - self: This class' instance
                    - mqtt_client: MQTT subscription callback MQTT client (mqtt.Client)
                    - mqtt_user_data: MQTT subscription callbak mqtt user data (Dict)
                    - mqtt_message: MQTT subscription callback message data (mqtt.MQTTMessage)
                Returns:
                    None
                    
                Usage:
                    self.__mqtt_manager__.subscribe(mqtt_topic_name)
                    self.__mqtt_manager__.client.message_callback_add(mqtt_topic_name, mqtt_subscription_callback)
                """
                
                mqtt_topic: str = mqtt_message.topic
                mqtt_decoded_payload: str = mqtt_message.payload.decode()

                self.get_logger().info("{} MQTT received message [{}] from [{}]".format(self.__rclpy_flags__, mqtt_decoded_payload, mqtt_message.topic))

                is_rcl_mqtt_topic_equals: bool = established_rcl_publisher.topic in mqtt_topic

                if is_rcl_mqtt_topic_equals == False:
                    self.get_logger().error("{} topic RCL & MQTT topics is not matching each other")
                    return
                else:
                    rcl_deserialized_message: Any = json.loads(mqtt_message.payload)
                    self.get_logger().info("{} deserialized message : [{}]".format(self.__rclpy_flags__, rcl_deserialized_message))
                    rcl_message_type: Any = self.__lookup_object__(topic_type)

                    created_rcl_messages: Any = message_conversion.populate_instance(rcl_deserialized_message, rcl_message_type())
                    established_rcl_publisher.publish(created_rcl_messages)

            if rcl_topic_name not in self.__established_rcl_publishers_list__:
                self.get_logger().info("===== {} [{}] sub to pub connection established =====".format(self.__rclpy_flags__, rcl_topic_name))
                self.__established_rcl_publishers_list__.append(rcl_topic_name)
                self.__established_rcl_publishers_dict__[rcl_topic_name] = established_rcl_publisher
                
                mqtt_topic_name: str = self.__mqtt_request_topic_format__ + rcl_topic_name
                self.__mqtt_manager__.subscribe(mqtt_topic_name)
                self.__mqtt_manager__.client.message_callback_add(mqtt_topic_name, mqtt_subscription_callback)
            else:
                return
        else:
            self.get_logger().info("{} [{}] is not a subscription".format(self.__rclpy_flags__, rcl_topic_name))
            return

    def __service_to_client__(self, rcl_service_name: str, rcl_service_type: str) -> None:
        
        """ Description
        
        Make connection bridge between current ROS2 service server into rclpy service client when count by service server's topic name is larger than 0.
        
        Args:
            - self: This class' instance
            - rcl_service_name: target ROS2 service server name (str)
            - rcl_service_type: target ROS2 service server type (str)

        Returns:
            None
            
        Usage:
            rcl_service_and_types_list: List[Tuple[str, List[str]]] = self.get_service_names_and_types()
            
            for rcl_service_name, rcl_service_type_list in rcl_service_and_types_list:
                for rcl_service_type in rcl_service_type_list:
                    self.get_logger().info("{} service : [{}], type : [{}]".format(self.__rclpy_flags__, rcl_service_name, rcl_service_type))
                    self.__service_to_client__(rcl_service_name, rcl_service_type)
        """
        
        is_ignoring_required_service_server: bool = "parameter" in rcl_service_name

        if is_ignoring_required_service_server:
            self.get_logger().warn("{} ignoring [{}] service client...".format(self.__rclpy_flags__, rcl_service_name))
            return
        elif rcl_service_name in self.__established_rcl_service_clients_list__:
            self.get_logger().warn("{} [{}] service client connection is already established... ignoring".format(self.__rclpy_flags__, rcl_service_name))
            return

        parsed_rcl_service_type: Any = self.__cast_topic_type_to_topic_class__(rcl_service_type)
        
        established_rcl_service_client: Client = self.create_client(parsed_rcl_service_type, rcl_service_name)

        def mqtt_subscription_callback(client: mqtt.Client, user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            
            """ Description
                
            Request to ROS2 service server with mqtt subscription callback message data after deserialize json into rclpy message class.
                
            Args:
                - self: This class' instance
                - mqtt_client: MQTT subscription callback MQTT client (mqtt.Client)
                - mqtt_user_data: MQTT subscription callbak mqtt user data (Dict)
                - mqtt_message: MQTT subscription callback message data (mqtt.MQTTMessage)
            Returns:
                None
                    
            Usage:
                self.__mqtt_manager__.subscribe(mqtt_topic_name)
                self.__mqtt_manager__.client.message_callback_add(mqtt_topic_name, mqtt_subscription_callback)
            """
            
            mqtt_topic: str = mqtt_message.topic
            mqtt_decoded_payload: str = mqtt_message.payload.decode()

            self.get_logger().info("{} MQTT service request received message [{}] from [{}]".format(self.__rclpy_flags__, mqtt_decoded_payload, mqtt_message.topic))

            is_rcl_mqtt_topic_equals: bool = established_rcl_service_client.srv_name in mqtt_topic

            if is_rcl_mqtt_topic_equals == False:
                self.get_logger().error("{} service request RCL & MQTT topics is not matching each other")
                return
            else:
                rcl_deserialized_request_message: Any = json.loads(mqtt_message.payload)
                self.get_logger().info("{} service request deserialized message : [{}]".format(self.__rclpy_flags__, rcl_deserialized_request_message))

                rcl_message_type: Any = self.__lookup_object__(rcl_service_type)
                
                rcl_service_server_waiting_time: float = 1.0
                
                is_rcl_service_server_ready: bool = established_rcl_service_client.wait_for_service(rcl_service_server_waiting_time)
                
                while not is_rcl_service_server_ready:
                    self.get_logger().error("{} [{}] service server is not ready yet...".format(self.__rclpy_flags__, rcl_service_name))

                    mqtt_response_topic: str = (self.__mqtt_response_topic_format__ + rcl_service_name)
                    mqtt_service_server_error_message: str = ("{} service server is not ready".format(rcl_service_name))
                    self.__mqtt_manager__.publish(mqtt_response_topic, mqtt_service_server_error_message)
                    break
                
                self.get_logger().info("{} calling to [{}] service server".format(self.__rclpy_flags__, rcl_service_name))
                
                created_rcl_messages: Any = message_conversion.populate_instance(rcl_deserialized_request_message, rcl_message_type())
                rcl_service_request = created_rcl_messages.Request()
                rcl_service_future: Future = established_rcl_service_client.call_async(rcl_service_request)
                rcl_service_call_async_result: Any = rcl_service_future.result()

                mqtt_serialized_response_message: str = json.dumps(message_conversion.extract_values(rcl_service_call_async_result))
                mqtt_response_topic: str = (self.__mqtt_response_topic_format__ + rcl_service_name)
                self.__mqtt_manager__.publish(mqtt_response_topic, mqtt_serialized_response_message)
            if rcl_service_name not in self.__established_rcl_publishers_list__:
                self.get_logger().info("===== {} [{}] service client connection established =====".format(self.__rclpy_flags__, rcl_service_name))
                self.__established_rcl_service_clients_list__.append(rcl_service_name)
                self.__established_rcl_service_clients_dict__[rcl_service_name] = established_rcl_service_client
                self.__mqtt_manager__.subscribe(rcl_service_name)
                self.__mqtt_manager__.client.message_callback_add(rcl_service_name, mqtt_subscription_callback)
            else:
                return


    def __action_to_client__(self, rcl_action_name: str, rcl_action_type: str) -> None:
        
        """ Description
        
        Make connection bridge between current ROS2 action server into rclpy action client when count by action server's topic name is larger than 0.
        
        Args:
            - self: This class' instance
            - rcl_action_name: target ROS2 action server name (str)
            - rcl_action_type: target ROS2 action server type (str)

        Returns:
            None
            
        Usage:
            rcl_action_and_types_list: List[Tuple[str, List[str]]] = get_action_names_and_types(self)

            for rcl_action_name, rcl_action_type_list in rcl_action_and_types_list:
                for rcl_action_type in rcl_action_type_list:
                    self.get_logger().info("{} action : [{}], type : [{}]".format(self.__rclpy_flags__, rcl_action_name, rcl_action_type))
        """
        
        if rcl_action_name in self.__established_rcl_action_client_list__:
            self.get_logger().warn("{} [{}] action client connection is already established... ignoring".format(self.__rclpy_flags__, rcl_action_name))
            return

        parsed_rcl_action_type: Any = self.__cast_topic_type_to_topic_class__(rcl_action_type)
        
        established_rcl_action_client: ActionClient = ActionClient(self, parsed_rcl_action_type, rcl_action_name)

        def mqtt_subscription_callback(mqtt_client: mqtt.Client, mqtt_user_data: Dict, mqtt_message: mqtt.MQTTMessage) -> None:
            
            """ Description
                
            Send goal to ROS2 action server with mqtt subscription callback message data after deserialize json into rclpy message class.
                
            Args:
                - self: This class' instance
                - mqtt_client: MQTT subscription callback MQTT client (mqtt.Client)
                - mqtt_user_data: MQTT subscription callbak mqtt user data (Dict)
                - mqtt_message: MQTT subscription callback message data (mqtt.MQTTMessage)
            Returns:
                None
                    
            Usage:
                self.__mqtt_manager__.subscribe(mqtt_topic_name)
                self.__mqtt_manager__.client.message_callback_add(mqtt_topic_name, mqtt_subscription_callback)
            """
            
            mqtt_topic: str = mqtt_message.topic
            mqtt_decoded_payload: str = mqtt_message.payload.decode()

            self.get_logger().info("{} MQTT action send goal received message [{}] from [{}]".format(self.__rclpy_flags__, mqtt_decoded_payload, mqtt_message.topic))

            is_rcl_mqtt_topic_equals: bool = established_rcl_action_client._action_name in mqtt_topic
            
            if is_rcl_mqtt_topic_equals == False:
                self.get_logger().error("{} action send goal RCL & MQTT topics is not matching each other")
                return
            else:
                rcl_deserialized_request_message: Any = json.loads(mqtt_message.payload)
                self.get_logger().info("{} action send goal deserialized message : [{}]".format(self.__rclpy_flags__, rcl_deserialized_request_message))

                rcl_message_type: Any = self.__lookup_object__(rcl_action_type)
                
                action_server_waiting_time: float = 1.0
                is_action_server_ready: Any = established_rcl_action_client.wait_for_server(action_server_waiting_time)

                if not is_action_server_ready:
                    self.get_logger().error("{} [{}] action server is not ready yet...".format(self.__rclpy_flags__, rcl_action_name))

                    mqtt_response_topic: str = (self.__mqtt_response_topic_format__ + rcl_action_name)
                    mqtt_action_server_error_message: str = ("{} action server is not ready".format(rcl_action_name))
                    self.__mqtt_manager__.publish(mqtt_response_topic, mqtt_action_server_error_message)
                    return
                else:
                    created_rcl_messages: Any = message_conversion.populate_instance(rcl_deserialized_request_message, rcl_message_type())

                    action_goal = created_rcl_messages.Goal()
                    action_future: Future = established_rcl_action_client.send_goal_async(action_goal)
                    action_send_goal_result: Any = action_future.result()

                    mqtt_serialized_response_message: str = json.dumps(message_conversion.extract_values(action_send_goal_result))
                    mqtt_response_topic: str = (self.__mqtt_response_topic_format__ + rcl_action_name)
                    self.__mqtt_manager__.publish(mqtt_response_topic, mqtt_serialized_response_message)
            if rcl_action_name not in self.__established_rcl_publishers_list__:
                self.get_logger().info("===== {} [{}] action client connection established =====".format(self.__rclpy_flags__, rcl_action_name))
                
                self.__established_rcl_action_client_list__.append(rcl_action_name)
                
                mqtt_topic_name: str = self.__mqtt_request_topic_format__ + rcl_action_name
                self.__mqtt_manager__.subscribe(mqtt_topic_name)
                self.__mqtt_manager__.client.message_callback_add(mqtt_topic_name, mqtt_subscription_callback)
            else:
                return


    def __bridge__(self) -> None:
        
        """ Description
        
        Look up current ROS2 topic / service / action and invoke each connection bridge methods.
        
        Args:
            - self: This class' instance
            
        Returns:
            None
            
        Usage:
            self.__bridge__()
        """

        rcl_topic_and_types_list: List[Tuple[str, List[str]]] = self.get_topic_names_and_types()

        for rcl_topic_name, rcl_topic_type_list in rcl_topic_and_types_list:
            for rcl_topic_type in rcl_topic_type_list:
                self.get_logger().info("{} topics : [{}], type : [{}]".format(self.__rclpy_flags__, rcl_topic_name, rcl_topic_type))
                self.__publisher_to_subscription__(rcl_topic_name, rcl_topic_type)
                self.__subscription_to_publisher__(rcl_topic_name, rcl_topic_type)


        rcl_service_and_types_list: List[Tuple[str, List[str]]] = self.get_service_names_and_types()

        for rcl_service_name, rcl_service_type_list in rcl_service_and_types_list:
            for rcl_service_type in rcl_service_type_list:
                self.get_logger().info("{} service : [{}], type : [{}]".format(self.__rclpy_flags__, rcl_service_name, rcl_service_type))
                self.__service_to_client__(rcl_service_name, rcl_service_type)


        rcl_action_and_types_list: List[Tuple[str, List[str]]] = get_action_names_and_types(self)

        for rcl_action_name, rcl_action_type_list in rcl_action_and_types_list:
            for rcl_action_type in rcl_action_type_list:
                self.get_logger().info("{} action : [{}], type : [{}]".format(self.__rclpy_flags__, rcl_action_name, rcl_action_type))



__all__ = ["dynamic_bridge"]
