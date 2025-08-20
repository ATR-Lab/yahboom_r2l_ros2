#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    yahboomcar_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    
    # Include the main bringup launch file
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboomcar_bringup_dir, 'launch', 'bringup.launch.py')
        )
    )
    
    # Patrol node
    patrol_node = Node(
        package='yahboomcar_bringup',
        executable='patrol_a1.py',
        name='YahboomCarPatrol',
        output='screen',
        parameters=[{
            'circle_adjust': 2.0
        }]
    )

    return LaunchDescription([
        bringup_launch,
        patrol_node
    ])