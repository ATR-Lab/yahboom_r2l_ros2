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
    
    # =============================================================================
    # DUMMY DATA FUNCTIONS - Replace with actual ROS2 data integration
    # =============================================================================
    
    def get_race_data(self) -> dict:
        """Get current race data - TODO: Replace with ROS2 /race/status topic"""
        import random
        race_time_seconds = int(time.time() - self.system_start_time)
        minutes = race_time_seconds // 60
        seconds = race_time_seconds % 60
        
        return {
            'race_time': f"{minutes:02d}:{seconds:02d}",
            'current_lap': random.randint(1, 3),
            'total_laps': 5,
            'race_active': True
        }
    
    def get_leaderboard_data(self) -> dict:
        """Get race leaderboard - TODO: Replace with ROS2 /race/leaderboard topic"""
        import random
        leader_id = random.choice([1, 2, 3, 4])
        car_names = {1: "Lightning", 2: "Thunder", 3: "Storm", 4: "Blitz"}
        return {
            'leader_car_id': leader_id,
            'leader_car_name': car_names.get(leader_id, f"Car #{leader_id}")
        }
    
    def get_environment_data(self) -> dict:
        """Get environmental data - TODO: Replace with ROS2 /environment/* topics"""
        import random
        weather_conditions = ["Clear, Dry", "Partly Cloudy", "Overcast", "Light Rain"]
        temperatures = ["18°C Cool", "22°C Optimal", "26°C Warm", "30°C Hot"]
        
        return {
            'weather': random.choice(weather_conditions),
            'temperature': random.choice(temperatures)
        }
    
    def get_network_status(self) -> dict:
        """Get network connectivity - TODO: Replace with ROS2 /system/network_status topic"""
        return {
            'connected': True,
            'quality': 'Good',
            'latency_ms': 45
        }
    
    def get_car_configuration(self) -> list:
        """Get dynamic car configuration - TODO: Replace with ROS2 /fleet/configuration topic"""
        return [
            {"id": 1, "name": "Lightning", "color": "#ff6b6b", "active": True},
            {"id": 2, "name": "Thunder", "color": "#4ecdc4", "active": True},
            {"id": 3, "name": "Storm", "color": "#45b7d1", "active": True},
            {"id": 4, "name": "Blitz", "color": "#f9ca24", "active": False}
        ]
    
    # =============================================================================
    # DUMMY ROS2 CALLBACK STUBS - Replace with actual ROS2 service calls
    # =============================================================================
    
    def set_global_speed_limit(self, limit_percent: int):
        """Set global speed limit - TODO: Publish to /system/speed_limit_override"""
        self.node.get_logger().info(f"DUMMY: Setting global speed limit to {limit_percent}%")
        # TODO: self.speed_limit_publisher.publish(Int32(data=limit_percent))
    
    def start_race(self):
        """Start race - TODO: Call ROS2 service /race/start_race"""
        self.node.get_logger().info("DUMMY: Starting race")
        # TODO: self.start_race_client.call_async(StartRace.Request())
    
    def stop_race(self):
        """Stop race - TODO: Call ROS2 service /race/stop_race"""
        self.node.get_logger().info("DUMMY: Stopping race")
        # TODO: self.stop_race_client.call_async(StopRace.Request())
    
    def reset_race(self):
        """Reset race - TODO: Call ROS2 service /race/reset_race"""
        self.node.get_logger().info("DUMMY: Resetting race")
        # TODO: self.reset_race_client.call_async(ResetRace.Request())
    
    def call_car_diagnostics(self, car_id: int):
        """Run car diagnostics - TODO: Call ROS2 service /car_X/run_diagnostics"""
        self.node.get_logger().info(f"DUMMY: Running diagnostics for car {car_id}")
        # TODO: service_name = f"/car_{car_id}/run_diagnostics"
        # TODO: self.diagnostics_clients[car_id].call_async(RunDiagnostics.Request())
    
    def call_car_reset_system(self, car_id: int):
        """Reset car system - TODO: Call ROS2 service /car_X/reset_system"""
        self.node.get_logger().info(f"DUMMY: Resetting system for car {car_id}")
        # TODO: service_name = f"/car_{car_id}/reset_system"  
        # TODO: self.reset_system_clients[car_id].call_async(ResetSystem.Request())
        
        # Update local state for UI feedback
        if car_id in self.cars:
            self.cars[car_id].emergency_stopped = False
            self.cars[car_id].control_mode = "RACING"
    
    def publish_manual_speed_limit(self, car_id: int, limit_percent: int):
        """Publish manual speed limit - TODO: Publish to /car_X/speed_limit"""
        self.node.get_logger().info(f"DUMMY: Setting speed limit for car {car_id} to {limit_percent}%")
        # TODO: topic_name = f"/car_{car_id}/speed_limit"
        # TODO: self.speed_limit_publishers[car_id].publish(Int32(data=limit_percent))
    
    def set_car_parameter(self, car_id: int, param_name: str, value):
        """Set a car-specific parameter - TODO: Use ROS2 parameter services"""
        # TODO: Call ROS2 parameter service for the specific car
        # service_name = f"/car_{car_id}/set_parameter" 
        # param_request = SetParameters.Request()
        # param_request.parameters = [Parameter(name=param_name, value=value)]
        # self.param_clients[car_id].call_async(param_request)
        
        # Log the parameter change
        self.log_user_action(f"Parameter '{param_name}' set to {value}", f"Car #{car_id}")

    def log_user_action(self, action: str, details: str = ""):
        """Log user action - TODO: Publish to /system/user_actions topic"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] USER: {action}"
        if details:
            log_entry += f" - {details}"
        self.node.get_logger().info(f"DUMMY: {log_entry}")
        # TODO: self.user_actions_publisher.publish(String(data=log_entry))