#!/usr/bin/env python3
"""
ROS2 Launch file for Yahboom R2L Robot Bringup
Equivalent to the ROS1 bringup.launch file

Includes:
- Robot description (URDF)
- Hardware driver (mcnamu_driver_ros2.py) 
- IMU filtering (imu_filter_madgwick)
- EKF localization (robot_localization)
- Joint/Robot state publishers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false', 
        description='Flag to start RViz visualization'
    )
    
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Flag to enable EKF localization'
    )
    
    nav_use_rotvel_arg = DeclareLaunchArgument(
        'nav_use_rotvel',
        default_value='false',
        description='Use rotational velocity for navigation'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='R2L',
        description='Robot type [X3, R2, R2L]'
    )
    
    # Get launch configurations
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ekf = LaunchConfiguration('use_ekf')
    nav_use_rotvel = LaunchConfiguration('nav_use_rotvel')
    robot_type = LaunchConfiguration('robot_type')
    
    # Robot description setup
    urdf_file = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'urdf',
        'yahboomcar_R2.urdf.xacro'  # Using R2 URDF for R2L robot
    ])
    
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot state publisher (common to both EKF and non-EKF modes)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
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
    
    # Hardware driver node (main driver)
    hardware_driver_node = Node(
        package='yahboomcar_bringup',
        executable='mcnamu_driver.py',
        name='yahboomcar_driver',
        output='screen',
        parameters=[{
            'car_type': robot_type,
            'xlinear_speed_limit': 1.0,
            'ylinear_speed_limit': 1.0, 
            'angular_speed_limit': 5.0,
            'nav_use_rotvel': nav_use_rotvel,
            'imu_link': 'imu_link'
        }],
        remappings=[
            ('/pub_vel', '/vel_raw'),
            ('/pub_imu', '/imu/imu_raw'),
            ('/pub_mag', '/mag/mag_raw')
        ]
    )
    
    # IMU filter node (Madgwick filter)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_node',
        name='imu_filter_madgwick',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[{
            'fixed_frame': 'base_link',
            'use_mag': True,
            'publish_tf': False,
            'use_magnetic_field_msg': True,
            'world_frame': 'enu',
            'orientation_stddev': 0.05,
            'angular_scale': 1.05
        }],
        remappings=[
            ('imu/data_raw', '/imu/imu_raw'),
            ('imu/mag', '/mag/mag_raw'), 
            ('imu/data', '/imu/imu_data'),
            ('imu/mag_out', '/mag/mag_field')
        ]
    )
    
    # Robot localization EKF node
    robot_localization_file = PathJoinSubstitution([
        FindPackageShare('yahboomcar_bringup'),
        'param',
        'robot_localization.yaml'
    ])
    
    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[robot_localization_file, {
            'odom_frame': 'odom',
            'world_frame': 'odom',
            'base_link_frame': 'base_footprint'
        }],
        remappings=[
            ('odometry/filtered', 'odom'),
            ('imu0', '/imu/imu_data'),
            ('odom0', 'odom_raw')
        ]
    )
    
    # RViz node (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('yahboomcar_description'),
        'rviz', 
        'yahboomcar_r2l.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # Include joystick control launch (from yahboomcar_ctrl package)
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yahboomcar_ctrl'),
                'launch',
                'yahboom_joy.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        # Launch arguments
        use_gui_arg,
        use_rviz_arg,
        use_ekf_arg,
        nav_use_rotvel_arg,
        robot_type_arg,
        
        # Core nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        hardware_driver_node,
        
        # Sensor processing (EKF mode)
        imu_filter_node,
        ekf_localization_node,
        
        # Visualization
        rviz_node,
        
        # Control interface
        joy_launch
    ])