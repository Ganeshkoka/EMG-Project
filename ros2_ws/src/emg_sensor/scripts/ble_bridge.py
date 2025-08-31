import asyncio, rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from bleak import BleakClient, BleakScanner

#initial thoughts
'''
I need to create a ROS node that essentially replaces the functionality of command_publisher.py
To do this, I need to publish to the same topic: /hand_command
I need to publish the same type of message: std_msgs/String
Instead of using a timer, I think I just have to check the GATT table to check for notifications?
To test things first, I'm going to write just a simple program that tests receivnig notifications and printing the statments in terminal, so I know the nRF to Jetson workflow works
