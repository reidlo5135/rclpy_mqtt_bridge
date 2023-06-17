from abc import ABCMeta
from typing import Optional, Type, Dict, Union

import inject
import paho.mqtt.client as mqtt

from ..util import lookup_object, extract_values, populate_instance
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


def create_bridge(factory: Union[str, "Bridge"], ros_message_type: str, topic_from: str, topic_to: str,
                  ros_frequency: Optional[float] = None, **kwargs) -> "Bridge":
    if isinstance(factory, str):
        factory = lookup_object(factory)

    if not issubclass(factory, Bridge):
        raise ValueError("factory should be Bridge subclass")

    if isinstance(ros_message_type, str):
        ros_message_type = lookup_object(ros_message_type)

    return factory(topic_from=topic_from, topic_to=topic_to, ros_message_type=ros_message_type, ros_frequency=ros_frequency, **kwargs)


class Bridge(object, metaclass=ABCMeta):
    mqtt_client_ = inject.attr(mqtt.Client)
    serialize_ = inject.attr('serializer')
    deserialize_ = inject.attr('deserializer')
    extract_private_path_ = inject.attr('mqtt_private_path_extractor')


class RosToMqttBridge(Bridge):
    def __init__(self, topic_from: str, topic_to: str, ros_message_type, ros_frequency: Optional[float] = None, **kwargs):
        self.ros_node_ = kwargs["ros_node"]
        self.topic_from_ = topic_from
        self.topic_to_ = self.extract_private_path_(topic_to)
        self.ros_last_published_time_ = self.ros_node_.get_clock().now()
        self.ros_interval_ = Duration(seconds=0) if ros_frequency is None else Duration(seconds=(1.0 / ros_frequency))
        self.ros_node_.create_subscription(ros_message_type, topic_from, self.ros_callback, 10)
        
        self.ros_node_.get_logger().info("[ROS to MQTT] created ROS subscription with topic : {}, message type : {}".format(topic_from, ros_message_type))

    def ros_callback(self, msg):
        self.ros_node_.get_logger().info("ROS received from {}".format(self.topic_from_))
        now = self.ros_node_.get_clock().now()
        if now - self.ros_last_published_time_ >= self.ros_interval_:
            self.mqtt_publish(msg)
            self.ros_last_published_time_ = now

    def mqtt_publish(self, msg):
        payload = self.serialize_(extract_values(msg))
        self.mqtt_client_.publish(topic=self.topic_to_, payload=payload)


class MqttToRosBridge(Bridge):
    def __init__(self, topic_from: str, topic_to: str, ros_message_type, ros_frequency: Optional[float] = None,
                 ros_queue_size: int = 10, **kwargs):
        self.ros_node = kwargs["ros_node"]
        self.topic_from_ = self.extract_private_path_(topic_from)
        self.topic_to_ = topic_to
        self.ros_message_type_ = ros_message_type
        self.ros_queue_size_ = ros_queue_size
        self.ros_last_published_time_ = self.ros_node.get_clock().now()
        self.ros_interval_ = None if ros_frequency is None else Duration(seconds=(1.0 / ros_frequency))

        self.mqtt_client_.subscribe(self.topic_from_)
        self.mqtt_client_.message_callback_add(self.topic_from_, self.mqtt_callback)
        self.ros_publisher_ = self.ros_node.create_publisher(self.ros_message_type_, self.topic_to_, 10)
        
        self.ros_node.get_logger().info("[MQTT to ROS] created ROS publisher with topic : {}, message type : {}".format(self.topic_to_, self.ros_message_type_))

    def mqtt_callback(self, client: mqtt.Client, userdata: Dict, mqtt_message: mqtt.MQTTMessage):
        self.ros_node.get_logger().info("MQTT received from {}".format(mqtt_message.topic))
        now = self.ros_node.get_clock().now()

        if self.ros_interval_ is None or now - self.ros_last_published_time_ >= self.ros_interval_:
            try:
                ros_created_message = self.create_ros_message(mqtt_message)
                self.ros_publisher_.publish(ros_created_message)
                self.ros_last_published_time_ = now
            except Exception as e:
                self.ros_node.get_logger().error(e)

    def create_ros_message(self, mqtt_msg: mqtt.MQTTMessage):
        if self.serialize_.__name__ == "packb":
            ros_message_dict = self.deserialize_(mqtt_msg.payload, raw=False)
        else:
            ros_message_dict = self.deserialize_(mqtt_msg.payload)
        return populate_instance(ros_message_dict, self.ros_message_type_())


__all__ = ['create_bridge', 'Bridge', 'RosToMqttBridge', 'MqttToRosBridge']
