import time
import paho.mqtt.client as mqtt
from rclpy.publisher import Publisher


class mqtt_logger:
    __info__: str = "[INFO]"
    __warn__: str = "[WARN]"
    __error__: str = "[ERROR]"

    def __init__(self) -> None:
        pass

    def __get_current_time__(self) -> str:
        current_time: float = time.time()
        time_stamp: int = int(current_time)
        micro_seconds: int = int((current_time - time_stamp) * 1e9)
        formatted_time: str = "[{}.{}]".format(time_stamp, micro_seconds)

        return formatted_time

    def __log__(
        self, start_color: str, level: str, message: str, end_color: str
    ) -> None:
        rclpy_node_name: str = "rclpy_mqtt_bridge"
        formatted_current_time: str = self.__get_current_time__()
        print(
            start_color
            + "{} {} [{}]".format(level, formatted_current_time, rclpy_node_name)
            + ": "
            + message
            + end_color
        )

    def info(self, message: str) -> None:
        start_color: str = ""
        end_color: str = ""
        self.__log__(start_color, self.__info__, message, end_color)

    def warn(self, message: str) -> None:
        start_color: str = "\033[33m"
        end_color: str = "\033[0m"
        self.__log__(start_color, self.__info__, message, end_color)

    def error(self, message: str) -> None:
        start_color: str = "\033[31m"
        end_color: str = "\033[0m"
        self.__log__(start_color, self.__info__, message, end_color)


class mqtt_broker:
    __broker_address__: str = "localhost"
    __broker_port__: int = 1883
    __client_name__: str = "rclpy_mqtt_bridge"
    __client_keep_alive__: int = 60
    __mqtt_logger__: mqtt_logger = mqtt_logger()

    client: mqtt.Client = mqtt.Client(
        __client_name__, clean_session=True, userdata=None, transport="tcp"
    )

    def __init__(self) -> None:
        self.client.on_connect = self.__on_connect__
        self.client.on_message = self.__on_message__
        self.client.connect(
            self.__broker_address__, self.__broker_port__, self.__client_keep_alive__
        )

        if self.client.is_connected:
            self.__mqtt_logger__.info("===== MQTT connected =====")
            self.client.loop_start()
        else:
            self.__mqtt_logger__.error("===== MQTT failed to connect =====")

    def __on_connect__(self, client, user_data, flags, rc) -> None:
        if rc == 0:
            self.__mqtt_logger__.info(
                "===== MQTT connection succeeded result code : [{}] =====".format(
                    str(rc)
                )
            )
        else:
            self.__mqtt_logger__.error(
                "===== MQTT connection failed result code : [{}] =====".format(str(rc))
            )

    def __on_message__(self, client, user_data, msg) -> None:
        self.__mqtt_logger__.info(
            "MQTT received message : {}".format(msg.payload.decode())
        )

    def publish(self, topic, payload) -> None:
        self.client.publish(topic=topic, payload=payload)

    def subscribe(self, topic) -> None:
        self.__mqtt_logger__.info("MQTT granted subscription from [{}]".format(topic))
        self.client.subscribe(topic=topic)


__all__ = ["mqtt_broker"]
