# rclpy_mqtt_bridge

## Document
- [rclpy_mqtt_bridge - For RCLPY(foxy)-MQTT connection](#rclpy_mqtt_bridge)
  - [Document](#document)
  - [Environment](#1-environment)
  - [SetUp Installation](#2-setup-installation)
    - [Prerequisites](#2-1-prerequisites)
    - [Installing rosbridge library](#2-2-installing-rosbridge-library)
  - [Clone & Build Project](#3-clone--build-project)
    - [Clone Project](#3-1-clone-project)
    - [Build Project](#3-2-build-project)
  - [Usage Examples](#4-usage-examples)
    - [ROS2 to MQTT](#4-1-ros2-to-mqtt)
    - [MQTT to ROS2](#4-2-mqtt-to-ros2)


## 1. Environment
* <img src="https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white">
* <img src="https://img.shields.io/badge/mqtt-660066?style=for-the-badge&logo=mqtt&logoColor=white">
* <img src="https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white">
* <img src="https://img.shields.io/badge/ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white">

## 2. SetUp Installation

### 2-1. Prerequisites

Before installing, please ensure the following softare is installed and configured on your system:

- [ubuntu](https://ubuntu.com/) version required 20.04 - **INSTALL [ubuntu](https://ubuntu.com/)**

- [mosquitto](https://mosquitto.org/) version required 1.6.9 - **INSTALL [mosquitto](https://mosquitto.org/)**

- [ROS2](https://index.ros.org/doc/ros2/Installation/) version required Foxy-Fitzroy -
  **INSTALL [ROS2 Foxy-Fitzroy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)**

### 2-2. Installing rosbridge library
```bash
sudo apt-get install ros-foxy-rosbridge-library
```

## 3. Clone & Build Project

### 3-1. Clone Project
```bash
cd ${your ros2 workspace}/src
git clone https://github.com/reidlo5135/rclpy_mqtt_bridge.git
```

### 3-2. Build Project
```bash
cd ${your ros2 workspace}/src/rclpy_mqtt_bridge

# install required pip3 modules
# msgpack-python>=0.4.8
# paho-mqtt>=1.2
# cryptography>=2.8
pip3 install -r requirements.txt

# build ros2 package
colcon build --symlink-install
cd ~/${your ros2 workspace}
colcon build --packages-select rclpy_dynamic_bridge
source install/setup.bash
source /opt/ros/foxy/setup.bash
```

## 4. Usage Examples

### 4-1. ROS2 to MQTT
```bash
source /opt/ros/foxy/setup.bash

# run /chatter publisher
ros2 run demo_nodes_cpp talker

# run rclpy_mqtt_bridge(as if bridge runs normally, bridge will check current ros2 topics and establish bridge connections every single 2.5s)
ros2 run rclpy_mqtt_bridge dynamic_bridge

# make your MQTT subscription and check subscription callback data
# subscription callback data example
# {"data": "Hello World: 53"}
# {"data": "Hello World: 54"}
mosquitto_sub -h localhost -p 1883 -t "/response/chatter"
```

### 4-2. MQTT to ROS2
```bash
source /opt/ros/foxy/setup.bash

# run /chatter subscriber
ros2 run demo_nodes_py listener

# run rclpy_mqtt_bridge(as if bridge runs normally, bridge will check current ros2 topics and establish bridge connections every single 2.5s)
ros2 run rclpy_mqtt_bridge dynamic_bridge

# publish your ROS2 data by MQTT publisher
# publishing message data example
# '{"data": "hi bridge"}'
mosquitto_pub -h localhost -p 1883 -t "/request/chatter" -m "'{"data":"hi bridge"}'"

# check subscription callback data in ros2 listener node terminal
# subscription callback data example
# [INFO] [1693712975.803897966] [listener]: I heard: [hi bridge]
```