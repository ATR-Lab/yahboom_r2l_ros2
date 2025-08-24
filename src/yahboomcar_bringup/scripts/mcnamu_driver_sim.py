#!/usr/bin/env python3
# encoding: utf-8
"""
Simulation version of mcnamu_driver for testing multiplayer namespace implementation
Does not require physical robot hardware - publishes dummy sensor data
"""
import sys
import math
import rclpy
from rclpy.node import Node
import random
import threading
from math import pi
from time import sleep
from sensor_msgs.msg import Imu, MagneticField, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool

class yahboomcar_driver_sim(Node):
    def __init__(self):
        super().__init__('yahboomcar_driver')
        # 弧度转角度
        # Radians turn angle
        self.RA2DE = 180 / pi
        
        # Declare parameters with defaults
        self.declare_parameter('car_type', 'R2')
        self.declare_parameter('car_id', 1)
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('prefix', '')
        self.declare_parameter('xlinear_speed_limit', 1.0)
        self.declare_parameter('ylinear_speed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 1.0)
        self.declare_parameter('nav_use_rotvel', False)
        
        # Get parameter values
        self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
        self.car_id = self.get_parameter('car_id').get_parameter_value().integer_value
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        self.Prefix = self.get_parameter('prefix').get_parameter_value().string_value
        self.xlinear_limit = self.get_parameter('xlinear_speed_limit').get_parameter_value().double_value
        self.ylinear_limit = self.get_parameter('ylinear_speed_limit').get_parameter_value().double_value
        self.angular_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
        self.nav_use_rotvel = self.get_parameter('nav_use_rotvel').get_parameter_value().bool_value
        
        # Command arbitration state
        self.emergency_stopped = False
        self.manual_override = False
        self.last_command_time = self.get_clock().now()
        self.last_manual_command_time = self.get_clock().now()
        self.command_timeout = 1.0  # 1 second timeout for game commands
        self.manual_override_timeout = 2.0  # 2 second timeout for manual override
        
        # Current motion state for simulation
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        self.get_logger().info(f"Initialized SIMULATION robot car with ID: {self.car_id}")
        
        # Subscribers - Original cmd_vel interface with priority arbitration
        self.sub_cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        
        # Manual override commands (from master control UI) - Priority 2  
        self.sub_manual_cmd_vel = self.create_subscription(Twist, 'manual_cmd_vel', self.manual_cmd_vel_callback, 1)
        
        # Emergency stop commands (from master control UI) - Priority 1
        self.sub_emergency_stop = self.create_subscription(Bool, 'emergency_stop', self.emergency_stop_callback, 1)
        
        # System-wide emergency stop - Priority 1
        self.sub_emergency_stop_all = self.create_subscription(Bool, '/system/emergency_stop_all', self.emergency_stop_all_callback, 1)
        
        # Actuator controls
        self.sub_RGBLight = self.create_subscription(Int32, "RGBLight", self.RGBLightcallback, 100)
        self.sub_Buzzer = self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 100)
        
        # Publishers
        self.EdiPublisher = self.create_publisher(Float32, 'edition', 100)
        self.volPublisher = self.create_publisher(Float32, 'voltage', 100)
        self.staPublisher = self.create_publisher(JointState, 'joint_states', 100)
        self.velPublisher = self.create_publisher(Twist, "pub_vel", 100)
        self.imuPublisher = self.create_publisher(Imu, "pub_imu", 100)
        self.magPublisher = self.create_publisher(MagneticField, "pub_mag", 100)
        
        # Create timer for safety timeout checking
        self.safety_timer = self.create_timer(0.1, self._safety_timeout_check)  # 10Hz safety checks
        
        # Create timer for publishing simulation data
        self.pub_timer = self.create_timer(0.1, self.pub_data)  # 10Hz data publishing

    def pub_data(self):
        """Publish simulation sensor data"""
        try:
            # Simulate battery voltage (between 11.5V and 12.6V)
            battery_voltage = 11.8 + random.random() * 0.8
            voltage_msg = Float32()
            voltage_msg.data = battery_voltage
            self.volPublisher.publish(voltage_msg)
            
            # Publish current velocity feedback
            vel_msg = Twist()
            vel_msg.linear.x = self.current_linear_x
            vel_msg.linear.y = self.current_linear_y  
            vel_msg.angular.z = self.current_angular_z
            self.velPublisher.publish(vel_msg)
            
            # Simulate IMU data
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_link
            # Add some simulated IMU data
            imu_msg.angular_velocity.z = self.current_angular_z + random.random() * 0.1
            imu_msg.linear_acceleration.x = self.current_linear_x + random.random() * 0.1
            self.imuPublisher.publish(imu_msg)
            
            # Simulate magnetometer data
            mag_msg = MagneticField()
            mag_msg.header.stamp = self.get_clock().now().to_msg() 
            mag_msg.header.frame_id = self.imu_link
            mag_msg.magnetic_field.x = random.random() * 0.5
            mag_msg.magnetic_field.y = random.random() * 0.5
            mag_msg.magnetic_field.z = 25.0 + random.random() * 5.0  # Typical Earth's magnetic field
            self.magPublisher.publish(mag_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing simulation data: {str(e)}")

    def cmd_vel_callback(self, msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
        if not isinstance(msg, Twist): return
        
        # Update last command time for timeout detection
        self.last_command_time = self.get_clock().now()
        
        # Only execute if no higher priority commands are active
        if not self.emergency_stopped and not self.manual_override:
            # 下发线速度和角速度
            # Issue linear vel and angular vel
            vx = msg.linear.x
            vy = msg.linear.y
            angular = msg.angular.z
            
            # Apply speed limiting for safety (40% max as per master UI design)
            vx = max(-0.4, min(0.4, vx * 0.7))
            vy = max(-0.4, min(0.4, vy))
            angular = max(-2.0, min(2.0, angular))
            
            # Store current motion for simulation
            self.current_linear_x = vx
            self.current_linear_y = vy
            self.current_angular_z = angular
            
            self.get_logger().debug(f"Car {self.car_id} CMD_VEL: vx={vx:.2f}, vy={vy:.2f}, ang={angular:.2f}")
    
    def manual_cmd_vel_callback(self, msg):
        """Handle manual override commands from master control UI - Priority 2"""
        if not isinstance(msg, Twist):
            return
            
        # Manual override takes precedence over game commands
        if not self.emergency_stopped:
            # Update manual override state and timestamp
            if not self.manual_override:
                self.get_logger().info(f"Car {self.car_id}: Manual override ACTIVATED")
            self.manual_override = True
            self.last_manual_command_time = self.get_clock().now()
            
            # Execute manual command directly (highest priority after emergency stop)
            vx = msg.linear.x
            vy = msg.linear.y
            angular = msg.angular.z
            
            # Apply speed limiting for safety
            vx = max(-0.4, min(0.4, vx * 0.7))
            vy = max(-0.4, min(0.4, vy))
            angular = max(-2.0, min(2.0, angular))
            
            # Store current motion for simulation
            self.current_linear_x = vx
            self.current_linear_y = vy
            self.current_angular_z = angular
            
            self.get_logger().debug(f"Car {self.car_id} MANUAL: vx={vx:.2f}, vy={vy:.2f}, ang={angular:.2f}")
    
    def emergency_stop_callback(self, msg):
        """Handle individual car emergency stop - Priority 1"""
        if not isinstance(msg, Bool):
            return
            
        self.emergency_stopped = msg.data
        if self.emergency_stopped:
            self.get_logger().warn(f"Car {self.car_id}: EMERGENCY STOP ACTIVATED")
            self._execute_emergency_stop()
        else:
            self.get_logger().info(f"Car {self.car_id}: Emergency stop released")
    
    def emergency_stop_all_callback(self, msg):
        """Handle system-wide emergency stop - Priority 1"""
        if not isinstance(msg, Bool):
            return
            
        self.emergency_stopped = msg.data
        if self.emergency_stopped:
            self.get_logger().warn(f"Car {self.car_id}: SYSTEM-WIDE EMERGENCY STOP")
            self._execute_emergency_stop()
    
    def RGBLightcallback(self, msg):
        """Simulate RGB light control"""
        if not isinstance(msg, Int32):
            return
        self.get_logger().info(f"Car {self.car_id}: RGB Light pattern {msg.data}")

    def Buzzercallback(self, msg):
        """Simulate buzzer control"""
        if not isinstance(msg, Bool):
            return
        if msg.data:
            self.get_logger().info(f"Car {self.car_id}: Buzzer ON")
        else:
            self.get_logger().info(f"Car {self.car_id}: Buzzer OFF")
    

    
    def _execute_emergency_stop(self):
        """Execute emergency stop procedure"""
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        self.get_logger().warn(f"Car {self.car_id}: EMERGENCY STOP - All motion stopped")
    
    def _safety_timeout_check(self):
        """Check for command timeouts and handle safety procedures"""
        current_time = self.get_clock().now()
        time_since_last_command = (current_time - self.last_command_time).nanoseconds / 1e9
        time_since_manual_command = (current_time - self.last_manual_command_time).nanoseconds / 1e9
        
        # Check manual override timeout - automatically disable if no recent manual commands
        if self.manual_override and time_since_manual_command > self.manual_override_timeout:
            self.manual_override = False
            self.get_logger().info(f"Car {self.car_id}: Manual override DEACTIVATED (timeout)")
        
        # If no game commands received for timeout period, stop the robot
        if time_since_last_command > self.command_timeout and not self.manual_override:
            if not self.emergency_stopped:
                self.current_linear_x = 0.0
                self.current_linear_y = 0.0
                self.current_angular_z = 0.0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver = yahboomcar_driver_sim()
        driver.get_logger().info(f"SIMULATION MODE: Car {driver.car_id} ready for testing")
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error initializing simulation driver: {str(e)}")
        if 'driver' in locals():
            driver.get_logger().error(f"Error: {str(e)}")
    finally:
        if 'driver' in locals():
            pass  # No hardware to clean up in simulation
        rclpy.shutdown()

if __name__ == '__main__':
    main()