# rclpy_mqtt_bridge

## install
```bash
pip3 install -r requirements.txt
```

## compile
```bash
cd ros2_ws/src/rclpy_mqtt_bridge
colcon build --symlink-install
cd ../..
colcon build --packages-select rclpy_mqtt_bridge
```

## run
```bash
source /opt/ros/foxy/setup.bash
ros2 run rclpy_mqtt_bridge dynamic_bridge
```