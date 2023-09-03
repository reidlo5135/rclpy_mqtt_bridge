import time
import paho.mqtt.client as mqtt

from cryptography.fernet import Fernet
from typing import Any

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
    
    def __generate_secret_key__(self) -> bytes:
        self.__mqtt_logger__.info("MQTT generated security key")
        generated_key: bytes = Fernet.generate_key()
        return generated_key
    
    def __create_cipher_suite__(self, key: Any) -> Fernet:
        self.__mqtt_logger__.info("MQTT created cipher suite")
        fernet: Fernet = Fernet(key)
        return fernet
    
    def __encrpyt_data__(self, data: Any, cipher_suite: Fernet) -> bytes:
        encoded_data: Any = data.encode()
        encrypted_data: bytes = cipher_suite.encrypt(encoded_data)
        self.__mqtt_logger__.info("MQTT encrypting data : [{}]".format(str(encrypted_data)))
        return encrypted_data

    def __decrypt_data__(self, encrypted_data: Any, cipher_suite: Fernet) -> Any:
        decrypted_data: Any = cipher_suite.decrypt(encrypted_data).decode()
        self.__mqtt_logger__.info("MQTT decrypting data : [{}]".format(str(decrypted_data)))
        return decrypted_data

    def __on_message__(self, client: Any, user_data: Any, msg: Any) -> None:
        self.__mqtt_logger__.info(
            "MQTT received message : {}".format(msg.payload.decode())
        )

    def publish(self, topic: str, payload: Any) -> None:
        self.client.publish(topic=topic, payload=payload, qos=0)

    def subscribe(self, topic: str) -> None:
        self.__mqtt_logger__.info("MQTT granted subscription from [{}]".format(topic))
        self.client.subscribe(topic=topic, qos=0)


__all__ = ["mqtt_broker"]
