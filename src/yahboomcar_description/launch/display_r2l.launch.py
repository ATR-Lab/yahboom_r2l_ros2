#!/usr/bin/env python3
"""
Launch file specifically for Yahboom R2L robot display in RViz2
Simplified version with R2L defaults
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
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
    
    # Path to R2 URDF file (used for R2L robot)
    urdf_file = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'urdf',
        'yahboomcar_R2.urdf.xacro'
    ])
    
    # Robot description parameter
    robot_description = Command(['xacro ', urdf_file])
    
    # Get launch configuration
    use_gui = LaunchConfiguration('use_gui')
    
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
        output='screen'
    )

    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node
    ])