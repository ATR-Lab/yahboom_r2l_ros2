#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    yahboomcar_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    
    # RViz configuration file
    rviz_config_file = os.path.join(yahboomcar_bringup_dir, 'rviz', 'odom.rviz')
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='odom_rviz',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rviz_node
    ])