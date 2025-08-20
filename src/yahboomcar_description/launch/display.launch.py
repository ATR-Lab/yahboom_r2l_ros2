#!/usr/bin/env python3
"""
Launch file for displaying Yahboom robot in RViz2
Equivalent to the ROS1 display.launch file
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindPackageShare, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='R2',  # Default for R2L robot
        description='Robot type [X1,X3,X3plus,R2,X7,R2L]'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 automatically'
    )

    # Get launch configurations
    use_gui = LaunchConfiguration('use_gui')
    robot_type = LaunchConfiguration('robot_type')
    use_rviz = LaunchConfiguration('rviz')
    
    # Path to URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'urdf',
        [robot_type, '.urdf.xacro']
    ])
    
    # Robot description parameter
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Joint state publisher GUI (when GUI is enabled)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        output='screen'
    )
    
    # Joint state publisher (when GUI is disabled)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui),
        output='screen'
    )
    
    # RViz2 node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'rviz',
        'yahboomcar.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        use_gui_arg,
        robot_type_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node
    ])