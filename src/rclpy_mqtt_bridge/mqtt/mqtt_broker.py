import paho.mqtt.client as mqtt
import time

class mqtt_broker:
    _instance = None
    _broker_address = "tcp://localhost"
    _broker_port = 1883
    _client_name = "rclpy_mqtt_bridge"
    client = mqtt.Client()

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.client = mqtt.Client(cls._client_name, clean_session=True, userdata=None, transport="tcp")
            cls._instance.client.on_connect = cls._instance.on_connect
            cls._instance.client.on_message = cls._instance.on_message
            cls._instance.client.connect(cls._broker_address, cls._broker_port, 60)
        return cls._instance

    def on_connect(self, _client, user_data, flags, rc):
        print("MQTT connected with result code : {}".format(str(rc)))

    def on_message(self, _client, user_data, msg):
        print("MQTT received message : {}".format(msg.payload.decode()))