#!/usr/bin/env python3

"""
Launch file for Bluetooth Bridge Node

Launches the Bluetooth bridge node with proper car_id and namespace configuration
for multiplayer racing system integration.

Usage:
    ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=1
    ros2 launch yahboomcar_bluetooth bluetooth_bridge.launch.py car_id:=2
    
Author: Yahboom R2L Multiplayer Racing Team
License: MIT
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Bluetooth bridge node."""
    
    # Launch arguments
    car_id_arg = DeclareLaunchArgument(
        'car_id',
        default_value='1',
        description='Unique car identifier for multiplayer racing (1-4)'
    )
    
    # Get launch configurations
    car_id = LaunchConfiguration('car_id')
    
    # Create namespace from car_id
    namespace = ['/car_', car_id]
    
    # Bluetooth bridge node
    bluetooth_bridge_node = Node(
        package='yahboomcar_bluetooth',
        executable='bluetooth_bridge_node',
        name='bluetooth_bridge_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'car_id': car_id,
        }],
        # No remapping needed - topics are already correctly named in the node
        # and will be automatically namespaced to /car_X/cmd_vel, /car_X/voltage, etc.
    )
    
    return LaunchDescription([
        # Arguments
        car_id_arg,
        
        # Nodes
        bluetooth_bridge_node,
    ])