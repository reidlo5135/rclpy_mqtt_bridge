import time
import paho.mqtt.client as mqtt

from cryptography.fernet import Fernet
from typing import Any


""" Description

    This class for make logger in mqtt methods
    
    Attributes:
        - __info__: info logging flag of mqtt (str)
        - __warn__: warn logging flag of mqtt (str)
        - __error__: error logging flag of mqtt (str)
    
    Methods:
        - __init__: Constructor method for this class.
        - __get_current_time__: Get current unix time and formatt into str.
        - __log__: Log with logging level and color.
        - info: Log in info level.
        - warn: Log in warn level.
        - error: Log in error level.

"""
class mqtt_logger:
    __info__: str = "[INFO]"
    __warn__: str = "[WARN]"
    __error__: str = "[ERROR]"

    def __init__(self) -> None:
        """ Description
        
        Constructor method for this class.
        
        Args:
            - self: This class' instance

        Returns:
            None
            
        Usage:
            __mqtt_logger__: mqtt_logger = mqtt_logger()
        """
        
        pass
    

    def __get_current_time__(self) -> str:
        
        """ Description
        
        Get current unix time and formatt into str.
        
        Args:
            - self: This class' instance

        Returns:
            - formatted_time: formatted current unix time (str)
            
        Usage:
            formatted_current_time: str = self.__get_current_time__()
        
        """
        
        current_time: float = time.time()
        time_stamp: int = int(current_time)
        micro_seconds: int = int((current_time - time_stamp) * 1e9)
        formatted_time: str = "[{}.{}]".format(time_stamp, micro_seconds)

        return formatted_time


    def __log__(self, start_color: str, level: str, message: str, end_color: str) -> None:
        
        """ Description
        
        Log with logging level and color.

        Args:
            - self: This class' instance
            - start_color: start color format (str)
            - level: logging level (str)
            - message: logging message (str)
            - end_color: end color format (str)

        Returns:
            None
            
        Usage:
            start_color: str = ""
            end_color: str = ""
            self.__log__(start_color, self.__info__, message, end_color)
        """
        
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
        
        """ Description
        
        Log in info level
        
        Args:
            - self: This class' instance
            - message: logging message (str)

        Returns:
            None
            
        Usage:
            self.__mqtt_logger__.info("MQTT generated security key")
        
        """
        
        start_color: str = ""
        end_color: str = ""
        self.__log__(start_color, self.__info__, message, end_color)


    def warn(self, message: str) -> None:
        
        """ Description
        
        Log in warn level
        
        Args:
            - self: This class' instance
            - message: logging message (str)

        Returns:
            None
            
        Usage:
            self.__mqtt_logger__.warn("MQTT generated security key")
        
        """
        
        start_color: str = "\033[33m"
        end_color: str = "\033[0m"
        self.__log__(start_color, self.__info__, message, end_color)


    def error(self, message: str) -> None:
        
        """ Description
        
        Log in error level
        
        Args:
            - self: This class' instance
            - message: logging message (str)

        Returns:
            None
            
        Usage:
            self.__mqtt_logger__.error("MQTT generated security key")
        
        """
        
        start_color: str = "\033[31m"
        end_color: str = "\033[0m"
        self.__log__(start_color, self.__info__, message, end_color)


""" Description

    This class for usage of paho.mqtt.client
    
    Attributes:
        - __broker_address__: The address of MQTT broker (str)
        - __broker_port__: port number of MQTT broker (str)
        - __client_name__: client name for register to MQTT broker (str)
        - __client_keep_alive__: limit time of this MQTT client (int)
        - __mqtt_logger__: The instance of mqtt_logger class (mqtt_logger)
        - client: The instance of mqtt.Client
    
    Methods:
        - __init__: Constructor method for this class. Register on Connect / Message methods and connect into MQTT broker
        - __on_connect__: Callback method that invoked when MQTT connected.
        - __generate_secret_key__: Generate MQTT secret key.
        - __create_cipher_suite__: Create MQTT cipher suite.
        - __encrypt_data__: Encode and encrypt MQTT message data.
        - __decrypt_data__: Decode and decrypt MQTT message data.
        - __on_message__: Callback method that invoked when MQTT subscription received message.
        - publish: MQTT publish into mqtt subscription with topic, payload.
        - subscribe: MQTT subscribe with topic.

"""
class mqtt_broker:
    __broker_address__: str = "localhost"
    __broker_port__: int = 1883
    __client_name__: str = "rclpy_mqtt_bridge"
    __client_keep_alive__: int = 60
    __mqtt_logger__: mqtt_logger = mqtt_logger()

    client: mqtt.Client = mqtt.Client(__client_name__, clean_session=True, userdata=None, transport="tcp")


    def __init__(self) -> None:
        
        """ Description
        
        Constructor method for this class. 
        
        Register on Connect / Message methods and connect into mqtt broker
        
        Args:
            - self: This class' instance

        Returns:
            None
            
        Usage:
            __mqtt_manager__: broker.mqtt_broker = broker.mqtt_broker()
        """
        
        self.client.on_connect = self.__on_connect__
        self.client.on_message = self.__on_message__
        self.client.connect(self.__broker_address__, self.__broker_port__, self.__client_keep_alive__)

        if self.client.is_connected:
            self.__mqtt_logger__.info("===== MQTT connected =====")
            self.client.loop_start()
        else:
            self.__mqtt_logger__.error("===== MQTT failed to connect =====")


    def __on_connect__(self, client: Any, user_data: Any, flags: Any, rc: Any) -> None:
        
        """ Description
        
        Callback method that invoked when MQTT connected.
        
        Args:
            - self: This class' instance
            - client: MQTT client instance (Any)
            - user_data: MQTT user data (Any)
            - flags: MQTT connection flags (Any)
            - rc: MQTT connection result code (Any)
            
        Returns:
            None
            
        Usage:
            self.client.on_connect = self.__on_connect__
        """
        
        if rc == 0:
            self.__mqtt_logger__.info("===== MQTT connection succeeded result code : [{}] =====".format(str(rc)))
        else:
            self.__mqtt_logger__.error("===== MQTT connection failed result code : [{}] =====".format(str(rc)))
    
    
    def __generate_secret_key__(self) -> bytes:
        
        """ Description
        
        Get current unix time and formatt into str.
        
        Args:
            - self: This class' instance

        Returns:
            - generated_key: Generated MQTT secret key by Fernet (bytes)
            
        Usage:

        """
        
        self.__mqtt_logger__.info("MQTT generated security key")
        generated_key: bytes = Fernet.generate_key()
        return generated_key
    
    
    def __create_cipher_suite__(self, key: Any) -> Fernet:
        
        """ Description
        
        Create MQTT cipher suite.
        
        Args:
            - self: This class' instance
            - key: MQTT secret key (Any)
            
        Returns:
            - fernet: Created Fernet instance with key (Fernet)
            
        Usage:
        
        """
        
        self.__mqtt_logger__.info("MQTT created cipher suite")
        fernet: Fernet = Fernet(key)
        return fernet
    
    
    def __encrpyt_data__(self, data: Any, cipher_suite: Fernet) -> bytes:
        
        """ Description
        
        Args:
            - self: This class' instance
            - data: Target data (Any)
            - cipher_suite: Created cipher_suite (Fernet)

        Returns:
            - encrypted_data: Encyprted MQTT data by cipher_suite (bytes)
            
        Usage:
        
        """
        
        encoded_data: Any = data.encode()
        encrypted_data: bytes = cipher_suite.encrypt(encoded_data)
        self.__mqtt_logger__.info("MQTT encrypting data : [{}]".format(str(encrypted_data)))
        return encrypted_data


    def __decrypt_data__(self, encrypted_data: Any, cipher_suite: Fernet) -> Any:
        
        """ Description
        
        Args:
            - self: This class' instance
            - encrypted_data: Already encrypted_data (Any)
            - cipher_suite: Created cipher_suite (Fernet)
            
        Returns:
            - decrypted_data: Decrypted MQTT data by cipher_suite
            
        Usage:
        
        """
        
        decrypted_data: Any = cipher_suite.decrypt(encrypted_data).decode()
        self.__mqtt_logger__.info("MQTT decrypting data : [{}]".format(str(decrypted_data)))
        return decrypted_data


    def __on_message__(self, client: Any, user_data: Any, msg: Any) -> None:
        
        """ Description
        
        Callback method that invoked when MQTT subscription received message.
        
        Args:
            - self: This class' instance
            - client: MQTT client instance (Any)
            - user_data: MQTT user data (Any)
            - msg: Received MQTT message data (Any)

        Returns:
            None
            
        Usage:
        
        """
        
        self.__mqtt_logger__.info("MQTT received message : {}".format(msg.payload.decode()))


    def publish(self, topic: str, payload: Any) -> None:
        
        """ Description
        
        MQTT publish into mqtt subscription with topic, payload.
        
        Args:
            - self: This class' instance
            - topic: Target MQTT topic (str)
            - payload: Target MQTT payload (Any)

        Returns:
            None
            
        Usage:
        
        """
        
        self.client.publish(topic=topic, payload=payload, qos=0)


    def subscribe(self, topic: str) -> None:
        
        """ Description
        
        MQTT subscribe with topic.
        
        Args:
            - self: This class' instance
            - topic: Target MQTT topic (str)

        Returns:
            None
            
        Usage:
        
        """
        
        self.__mqtt_logger__.info("MQTT granted subscription from [{}]".format(topic))
        self.client.subscribe(topic=topic, qos=0)


__all__ = ["mqtt_broker"]
