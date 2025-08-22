#!/usr/bin/env python3
"""
ROS Data Manager - Handles all ROS2 topic subscriptions and data management.
Provides a centralized interface for UI components to access robot data.
"""

from typing import Dict, Any, Optional
import time
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState, Joy
from nav_msgs.msg import Odometry


class CarData:
    """Data container for individual car information."""
    
    def __init__(self, car_id: int):
        self.car_id = car_id
        self.name = f"Car {car_id}"
        
        # Connection status
        self.webrtc_connected = False
        self.bluetooth_connected = False
        self.ros2_connected = False
        self.last_update_time = 0
        
        # Vital signs
        self.battery_voltage = 0.0
        self.battery_percentage = 0
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0
        
        # Sensor data
        self.imu_data = None
        self.magnetometer_data = None
        self.joint_states = None
        
        # Control state
        self.control_mode = "RACING"  # RACING, MANUAL, EMERGENCY_STOPPED
        self.emergency_stopped = False
        
        # Health status
        self.system_health = "OK"  # OK, WARNING, ERROR
        self.error_messages = []


class RosDataManager:
    """Manages all ROS2 data for the master control center."""
    
    def __init__(self, node: Node):
        self.node = node
        self.cars: Dict[int, CarData] = {}
        
        # System-wide data
        self.joystick_connected = False
        self.joystick_data = None
        self.selected_car_id = None
        self.system_start_time = time.time()
        
        # Initialize car data for 4 cars
        for car_id in range(1, 5):
            self.cars[car_id] = CarData(car_id)
        
        # Set up ROS2 subscriptions
        self._setup_subscriptions()
        
        # Set up ROS2 publishers for control
        self._setup_publishers()
        
        self.node.get_logger().info("ROS Data Manager initialized")
    
    def _setup_subscriptions(self):
        """Set up ROS2 topic subscriptions for all cars."""
        self.subscriptions = []
        
        # Subscribe to joystick for manual control
        self.subscriptions.append(
            self.node.create_subscription(Joy, '/joy', self._joystick_callback, 10)
        )
        
        # Subscribe to topics for each car
        for car_id in range(1, 5):
            namespace = f"/car_{car_id}"
            
            # Battery voltage
            self.subscriptions.append(
                self.node.create_subscription(
                    Float32, f"{namespace}/voltage", 
                    lambda msg, cid=car_id: self._battery_callback(msg, cid), 10
                )
            )
            
            # Velocity feedback
            self.subscriptions.append(
                self.node.create_subscription(
                    Twist, f"{namespace}/pub_vel",
                    lambda msg, cid=car_id: self._velocity_callback(msg, cid), 10
                )
            )
            
            # IMU data
            self.subscriptions.append(
                self.node.create_subscription(
                    Imu, f"{namespace}/pub_imu",
                    lambda msg, cid=car_id: self._imu_callback(msg, cid), 10
                )
            )
            
            # Odometry for position
            self.subscriptions.append(
                self.node.create_subscription(
                    Odometry, f"{namespace}/odom",
                    lambda msg, cid=car_id: self._odometry_callback(msg, cid), 10
                )
            )
            
            # Joint states
            self.subscriptions.append(
                self.node.create_subscription(
                    JointState, f"{namespace}/joint_states",
                    lambda msg, cid=car_id: self._joint_states_callback(msg, cid), 10
                )
            )
    
    def _setup_publishers(self):
        """Set up ROS2 publishers for control commands."""
        self.publishers = {}
        
        # Manual control publishers for each car
        for car_id in range(1, 5):
            namespace = f"/car_{car_id}"
            
            # Manual command velocity
            self.publishers[f"manual_cmd_vel_{car_id}"] = self.node.create_publisher(
                Twist, f"{namespace}/manual_cmd_vel", 10
            )
            
            # Emergency stop
            self.publishers[f"emergency_stop_{car_id}"] = self.node.create_publisher(
                Bool, f"{namespace}/emergency_stop", 10
            )
        
        # System-wide emergency stop
        self.publishers["emergency_stop_all"] = self.node.create_publisher(
            Bool, "/system/emergency_stop_all", 10
        )
    
    # Callback functions for ROS topics
    def _joystick_callback(self, msg: Joy):
        """Handle joystick input."""
        self.joystick_data = msg
        self.joystick_connected = True
        
        # If a car is selected for manual control, forward joystick commands
        if self.selected_car_id and self.get_car_control_mode(self.selected_car_id) == "MANUAL":
            self._send_manual_command(self.selected_car_id, msg)
    
    def _battery_callback(self, msg: Float32, car_id: int):
        """Handle battery voltage updates."""
        if car_id in self.cars:
            self.cars[car_id].battery_voltage = msg.data
            # Convert voltage to percentage (example: 11V = 0%, 12.6V = 100%)
            self.cars[car_id].battery_percentage = max(0, min(100, 
                int((msg.data - 11.0) / 1.6 * 100)))
            self._update_connection_status(car_id)
    
    def _velocity_callback(self, msg: Twist, car_id: int):
        """Handle velocity feedback."""
        if car_id in self.cars:
            self.cars[car_id].speed_linear = msg.linear.x
            self.cars[car_id].speed_angular = msg.angular.z
            self._update_connection_status(car_id)
    
    def _imu_callback(self, msg: Imu, car_id: int):
        """Handle IMU data."""
        if car_id in self.cars:
            self.cars[car_id].imu_data = msg
            self._update_connection_status(car_id)
    
    def _odometry_callback(self, msg: Odometry, car_id: int):
        """Handle odometry data for position."""
        if car_id in self.cars:
            self.cars[car_id].position_x = msg.pose.pose.position.x
            self.cars[car_id].position_y = msg.pose.pose.position.y
            
            # Extract yaw from quaternion (simplified)
            orientation = msg.pose.pose.orientation
            # This is a simplified yaw calculation - in real use, use proper quaternion math
            self.cars[car_id].heading = orientation.z * 180  # Placeholder
            self._update_connection_status(car_id)
    
    def _joint_states_callback(self, msg: JointState, car_id: int):
        """Handle joint state data."""
        if car_id in self.cars:
            self.cars[car_id].joint_states = msg
            self._update_connection_status(car_id)
    
    def _update_connection_status(self, car_id: int):
        """Update connection status based on recent message activity."""
        if car_id in self.cars:
            self.cars[car_id].last_update_time = time.time()
            self.cars[car_id].ros2_connected = True
    
    def _send_manual_command(self, car_id: int, joy_msg: Joy):
        """Convert joystick input to car commands and send."""
        if f"manual_cmd_vel_{car_id}" in self.publishers:
            cmd_vel = Twist()
            
            # Example joystick mapping (adjust based on your controller)
            # Left stick Y-axis for forward/backward
            cmd_vel.linear.x = joy_msg.axes[1] * 1.0  # Max 1 m/s for safety
            
            # Right stick X-axis for steering
            cmd_vel.angular.z = joy_msg.axes[2] * 1.5  # Max 1.5 rad/s
            
            # Apply safety limits for manual control
            cmd_vel.linear.x = max(-0.5, min(0.5, cmd_vel.linear.x))  # Reduced speed
            cmd_vel.angular.z = max(-1.0, min(1.0, cmd_vel.angular.z))
            
            self.publishers[f"manual_cmd_vel_{car_id}"].publish(cmd_vel)
    
    # Public interface methods
    def get_car_data(self, car_id: int) -> Optional[CarData]:
        """Get data for a specific car."""
        return self.cars.get(car_id)
    
    def get_all_cars(self) -> Dict[int, CarData]:
        """Get data for all cars."""
        return self.cars.copy()
    
    def set_manual_control(self, car_id: int, enable: bool):
        """Enable/disable manual control for a car."""
        if car_id in self.cars:
            self.cars[car_id].control_mode = "MANUAL" if enable else "RACING"
            self.selected_car_id = car_id if enable else None
    
    def emergency_stop(self, car_id: int):
        """Emergency stop a specific car."""
        if f"emergency_stop_{car_id}" in self.publishers:
            msg = Bool()
            msg.data = True
            self.publishers[f"emergency_stop_{car_id}"].publish(msg)
            self.cars[car_id].emergency_stopped = True
            self.cars[car_id].control_mode = "EMERGENCY_STOPPED"
    
    def emergency_stop_all(self):
        """Emergency stop all cars."""
        msg = Bool()
        msg.data = True
        self.publishers["emergency_stop_all"].publish(msg)
        
        for car in self.cars.values():
            car.emergency_stopped = True
            car.control_mode = "EMERGENCY_STOPPED"
    
    def reset_car(self, car_id: int):
        """Reset a car from emergency stop."""
        if car_id in self.cars:
            self.cars[car_id].emergency_stopped = False
            self.cars[car_id].control_mode = "RACING"
    
    def get_car_control_mode(self, car_id: int) -> str:
        """Get the current control mode for a car."""
        if car_id in self.cars:
            return self.cars[car_id].control_mode
        return "UNKNOWN"
    
    def is_joystick_connected(self) -> bool:
        """Check if joystick is connected."""
        return self.joystick_connected
    
    def get_system_uptime(self) -> float:
        """Get system uptime in seconds."""
        return time.time() - self.system_start_time