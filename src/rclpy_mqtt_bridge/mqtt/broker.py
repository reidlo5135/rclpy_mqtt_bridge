import paho.mqtt.client as mqtt
import time


class mqtt_logger:
    __info__: str = "[INFO]"
    __warn__: str = "[WARN]"
    __error__: str = "[ERROR]"

    def __init__(self) -> None:
        pass

    def __get_current_time__(self):
        current_time: float = time.time()
        time_stamp: int = int(current_time)
        micro_seconds: int = int((current_time - time_stamp) * 1e9)
        formatted_time: str = "[{}.{}]".format(time_stamp, micro_seconds)

        return formatted_time

    def info(self, message: str):
        formatted_current_time: str = self.__get_current_time__()
        print("{} {}".format(self.__info__, formatted_current_time) + " " + message)

    def warn(self, message: str):
        formatted_current_time: str = self.__get_current_time__()
        print("{} {}".format(self.__warn__, formatted_current_time) + " " + message)

    def error(self, message: str):
        formatted_current_time: str = self.__get_current_time__()
        print("{} {}".format(self.__error__, formatted_current_time) + " " + message)


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

    def __on_connect__(self, client, user_data, flags, rc):
        self.__mqtt_logger__.info(
            "MQTT connected with result code : {}".format(str(rc))
        )

    def __on_message__(self, client, user_data, msg):
        self.__mqtt_logger__.info(
            "MQTT received message : {}".format(msg.payload.decode())
        )

    def publish(self, topic, payload):
        self.__mqtt_logger__.info(
            "MQTT publish into [{}] with payload [{}]".format(topic, payload)
        )
        self.client.publish(topic=topic, payload=payload)

    def subscribe(self, topic):
        self.__mqtt_logger__.info("MQTT subsribe into [{}]".format(topic))
        self.client.subscribe(topic=topic)
