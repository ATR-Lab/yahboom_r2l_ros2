#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Use joint_state_publisher_gui'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=EnvironmentVariable('ROBOT_TYPE', default_value='R2L'),
        description='Robot type [X1,X3,X3plus,R2,X7,R2L]'
    )

    # Get launch configurations
    use_gui = LaunchConfiguration('use_gui')
    robot_type = LaunchConfiguration('robot_type')

    # Get package directories
    yahboomcar_description_dir = get_package_share_directory('yahboomcar_description')

    # Hardware driver node (simplified - should have robot_type condition)
    mcnamu_driver_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver.py',
        name='driver_node',
        output='screen',
        parameters=[{
            'xlinear_speed_limit': 1.0,
            'ylinear_speed_limit': 1.0,
            'angular_speed_limit': 5.0,
            'imu_link': 'imu_link'
        }],
        remappings=[
            ('/pub_vel', '/vel_raw'),
            ('/pub_imu', '/imu/imu_raw'),
            ('/pub_mag', '/mag/mag_raw')
        ]
        # TODO: Add condition for robot_type == 'R2L'
    )

    # Warning node (simplified - should have robot_type condition)
    warning_node = Node(
        package='yahboomcar_bringup',
        executable='warning.py',
        name='warning',
        output='screen'
        # TODO: Add condition for robot_type != 'R2L'
    )

    # Robot description (simplified for R2L robot type)
    xacro_file = os.path.join(yahboomcar_description_dir, 'urdf', 'yahboomcar_R2.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint state publisher (GUI version)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(use_gui)
    )

    # Joint state publisher (non-GUI version)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui)
    )

    return LaunchDescription([
        # Arguments
        use_gui_arg,
        robot_type_arg,

        # Nodes
        mcnamu_driver_node,
        warning_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node
    ])