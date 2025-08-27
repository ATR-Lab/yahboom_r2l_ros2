#!/usr/bin/env python3

"""
Launch file for Bluetooth ROS2 Bridge - AR App to Robot Communication

This launch file starts the new Bluetooth ROS2 bridge that connects
iPhone AR apps to robot control systems using proven BLE server logic.

Usage:
    ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py
    ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=2

Features:
- Starts bluetooth_ros2_bridge node with car_id parameter
- Configures topic namespacing for multiplayer racing (/car_X/*)
- Sets up proper node naming for multi-robot scenarios

Author: Yahboom R2L Racing Team
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Bluetooth ROS2 bridge."""
    
    # Declare launch arguments
    car_id_arg = DeclareLaunchArgument(
        'car_id',
        default_value='1',
        description='Car ID for multiplayer racing (1-4)'
    )
    
    jetson_mode_arg = DeclareLaunchArgument(
        'jetson_mode',
        default_value='false',
        description='Enable Jetson Nano compatibility mode (BlueZ 5.53)'
    )
    
    # Get launch configuration
    car_id = LaunchConfiguration('car_id')
    jetson_mode = LaunchConfiguration('jetson_mode')
    
    # Create the Bluetooth ROS2 bridge node
    bluetooth_bridge_node = Node(
        package='yahboomcar_bluetooth',
        executable='bluetooth_ros2_bridge',
        name=['bluetooth_bridge_car', car_id],
        namespace=['car_', car_id],  # Topics become /car_X/cmd_vel, etc.
        parameters=[{
            'car_id': car_id,
            'jetson_mode': jetson_mode
        }],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    return LaunchDescription([
        car_id_arg,
        jetson_mode_arg,
        bluetooth_bridge_node
    ])