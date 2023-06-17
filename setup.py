from setuptools import setup
import os
from glob import glob

package_name = 'rclpy_mqtt_bridge'
bridge_package_name = package_name + '.bridge'
mqtt_package_name = package_name +  '.mqtt'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, bridge_package_name, mqtt_package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))  
    ],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reidlo',
    maintainer_email='naru5135@wavem.net',
    description='rclpy-mqtt bridge',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rclpy_mqtt_bridge_node = rclpy_mqtt_bridge.rclpy_mqtt_bridge_node:main'
        ],
    },
)
