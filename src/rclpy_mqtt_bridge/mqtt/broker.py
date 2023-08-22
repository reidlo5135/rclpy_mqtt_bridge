import paho.mqtt.client as mqtt


class mqtt_broker:
    _broker_address: str = "localhost"
    _broker_port: int = 1883
    _client_name: str = "rclpy_mqtt_bridge"
    _client_keep_alive: int = 60
    
    client: mqtt.Client = mqtt.Client(
        _client_name, clean_session=True, userdata=None, transport="tcp"
    )

    def __init__(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self._broker_address, self._broker_port, self._client_keep_alive)

        if self.client.is_connected:
            print("===== MQTT connected =====")
            self.client.loop_start()
        else:
            print("===== MQTT failed to connect =====")

    def on_connect(self, client, user_data, flags, rc):
        print("MQTT connected with result code : {}".format(str(rc)))

    def on_message(self, client, user_data, msg):
        print("MQTT received message : {}".format(msg.payload.decode()))
    
    def publish(self, topic, payload):
        print("MQTT publish into [{}] with payload [{}]".format(topic, payload))
        self.client.publish(topic=topic, payload=payload)
    
    def subscribe(self, topic):
        print("MQTT subsribe into [{}]".format(topic))
        self.client.subscribe(topic=topic)
