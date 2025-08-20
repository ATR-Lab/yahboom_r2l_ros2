#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    yahboomcar_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    
    # Include yahboomcar launch file
    yahboomcar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yahboomcar_bringup_dir, 'launch', 'yahboomcar.launch.py')
        )
    )
    
    # IMU calibration output file path
    imu_calib_output_file = os.path.join(yahboomcar_bringup_dir, 'param', 'imu_calib.yaml')
    
    # IMU calibration node
    imu_calib_node = Node(
        package='imu_calib',
        executable='do_calib',
        name='do_calib',
        output='screen',
        parameters=[{
            'output_file': imu_calib_output_file
        }],
        remappings=[
            ('/sub_imu', '/imu/imu_raw')
        ]
    )

    return LaunchDescription([
        yahboomcar_launch,
        imu_calib_node
    ])