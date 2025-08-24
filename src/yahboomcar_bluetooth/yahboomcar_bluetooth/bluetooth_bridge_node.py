#!/usr/bin/env python3

"""
Bluetooth Bridge Node for iPhone AR App to Robot Communication

This node serves as a bridge between the iPhone AR app and the ROS2 robot system.
It receives JSON commands from the iPhone via Bluetooth and converts them to ROS2 messages,
while also sending robot sensor data back to the iPhone for AR positioning.

Architecture:
iPhone AR App ←Bluetooth LE→ This Bridge ←ROS2 Topics→ Robot Driver

Message Flow:
- iPhone → JSON commands → /car_X/cmd_vel (Twist messages)
- Robot sensors → /car_X/voltage, /car_X/imu/imu_raw, etc. → JSON → iPhone

Author: Yahboom R2L Multiplayer Racing Team
License: MIT
"""

import json
import asyncio
import threading
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu

try:
    import bleak
    BLUETOOTH_AVAILABLE = True
except ImportError:
    BLUETOOTH_AVAILABLE = False
    print("WARNING: bleak library not installed. Bluetooth functionality disabled.")
    print("Install with: pip3 install bleak")


class BluetoothBridgeNode(Node):
    """
    ROS2 node that bridges iPhone AR app commands to robot control system.
    
    This node acts as the communication hub between the iPhone AR app and the
    robot's ROS2 system, handling real-time command translation and sensor feedback.
    """
    
    def __init__(self):
        super().__init__('bluetooth_bridge_node')
        
        # Declare and get car_id parameter
        self.declare_parameter('car_id', 1)
        self.car_id = self.get_parameter('car_id').get_parameter_value().integer_value
        
        # Validate car_id range (1-4 for multiplayer racing)
        if not 1 <= self.car_id <= 4:
            self.get_logger().error(f"Invalid car_id: {self.car_id}. Must be between 1-4.")
            raise ValueError(f"car_id must be between 1-4, got {self.car_id}")
        
        self.get_logger().info(f"Initializing Bluetooth bridge for car {self.car_id}")
        
        # Setup ROS2 publishers and subscribers
        self._setup_ros_interface()
        
        # Bluetooth state management
        self.bluetooth_connected = False
        self.message_buffer = ""
        self.last_command_time = self.get_clock().now()
        
        # Robot state for iPhone feedback
        self.robot_state = {
            'car_id': self.car_id,
            'battery_voltage': 0.0,
            'emergency_state': False,
            'connection_quality': 1.0,
            'timestamp': 0.0
        }
        
        # Start Bluetooth service if available
        if BLUETOOTH_AVAILABLE:
            self._start_bluetooth_service()
        else:
            self.get_logger().warn("Bluetooth not available - running in mock mode")
            
        self.get_logger().info(f"Bluetooth bridge ready for car {self.car_id}")
    
    def _setup_ros_interface(self):
        """Setup ROS2 publishers and subscribers for robot communication."""
        
        # Publisher for robot commands (integrates with existing priority system)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel',  # Will be namespaced to /car_X/cmd_vel by launch file
            10
        )
        
        # Subscribers for robot sensor feedback (to send back to iPhone)
        self.voltage_subscriber = self.create_subscription(
            Float32,
            'voltage',  # Will be namespaced to /car_X/voltage
            self._voltage_callback,
            10
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu/imu_raw',  # Will be namespaced to /car_X/imu/imu_raw
            self._imu_callback,
            10
        )
        
        self.emergency_subscriber = self.create_subscription(
            Bool,
            'emergency_stop',  # Will be namespaced to /car_X/emergency_stop
            self._emergency_callback,
            10
        )
        
        # Timer for periodic sensor data transmission to iPhone
        self.sensor_timer = self.create_timer(0.2, self._send_sensor_feedback)  # 5Hz
        
    def _voltage_callback(self, msg: Float32):
        """Update robot battery voltage for iPhone feedback."""
        self.robot_state['battery_voltage'] = msg.data
        
    def _imu_callback(self, msg: Imu):
        """Process IMU data for iPhone AR positioning."""
        # Store relevant IMU data for iPhone
        # iPhone needs this for AR world alignment
        self.robot_state['imu'] = {
            'angular_velocity': {
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y
            }
        }
        
    def _emergency_callback(self, msg: Bool):
        """Track emergency stop state for iPhone awareness."""
        self.robot_state['emergency_state'] = msg.data
        
    def _send_sensor_feedback(self):
        """Send robot sensor data to iPhone for AR positioning and game state."""
        if not self.bluetooth_connected:
            return
            
        # Update timestamp
        self.robot_state['timestamp'] = self.get_clock().now().nanoseconds / 1e9
        
        # Create sensor feedback message for iPhone
        feedback_message = {
            'msg_type': 'robot_status',
            'data': self.robot_state
        }
        
        # Send to iPhone via Bluetooth (async)
        asyncio.run_coroutine_threadsafe(
            self._send_bluetooth_message(feedback_message),
            self.bluetooth_loop
        )
        
    async def _send_bluetooth_message(self, message: Dict[str, Any]):
        """Send JSON message to iPhone via Bluetooth."""
        try:
            json_str = json.dumps(message) + '\n'  # Newline-delimited JSON
            # TODO: Implement actual Bluetooth LE transmission
            self.get_logger().debug(f"Would send to iPhone: {json_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send Bluetooth message: {e}")
            
    def _process_iphone_command(self, command: Dict[str, Any]):
        """
        Process incoming command from iPhone AR app.
        
        Expected JSON format:
        {
            "msg_type": "robot_command",
            "data": {
                "movement": {"linear": {"x": 0.5}, "angular": {"z": 1.2}},
                "game_effects": {"power_up": "speed_boost", "duration": 2.0}
            }
        }
        """
        try:
            if command.get('msg_type') != 'robot_command':
                self.get_logger().warn(f"Unknown message type: {command.get('msg_type')}")
                return
                
            data = command.get('data', {})
            movement = data.get('movement', {})
            
            # Extract movement commands
            linear = movement.get('linear', {})
            angular = movement.get('angular', {})
            
            # Create ROS2 Twist message
            twist_msg = Twist()
            twist_msg.linear.x = float(linear.get('x', 0.0))
            twist_msg.linear.y = float(linear.get('y', 0.0))
            twist_msg.angular.z = float(angular.get('z', 0.0))
            
            # TODO: Process game effects (power-ups, collisions, etc.)
            game_effects = data.get('game_effects', {})
            if game_effects:
                self.get_logger().info(f"Game effects: {game_effects}")
            
            # Publish to robot (integrates with existing priority system)
            self.cmd_vel_publisher.publish(twist_msg)
            self.last_command_time = self.get_clock().now()
            
            self.get_logger().debug(
                f"iPhone command: linear=({twist_msg.linear.x:.2f}, {twist_msg.linear.y:.2f}), "
                f"angular={twist_msg.angular.z:.2f}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing iPhone command: {e}")
            
    def _parse_json_messages(self, data: str):
        """
        Parse newline-delimited JSON messages from iPhone.
        
        Handles partial messages and message boundaries correctly.
        """
        self.message_buffer += data
        
        # Process complete messages (terminated by newlines)
        while '\n' in self.message_buffer:
            message, self.message_buffer = self.message_buffer.split('\n', 1)
            message = message.strip()
            
            if message:
                try:
                    command = json.loads(message)
                    self._process_iphone_command(command)
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"Invalid JSON from iPhone: {message} - Error: {e}")
                    
    def _start_bluetooth_service(self):
        """Start Bluetooth LE service for iPhone communication."""
        if not BLUETOOTH_AVAILABLE:
            return
            
        # Start Bluetooth service in separate thread
        self.bluetooth_thread = threading.Thread(target=self._run_bluetooth_service)
        self.bluetooth_thread.daemon = True
        self.bluetooth_thread.start()
        
    def _run_bluetooth_service(self):
        """Run Bluetooth LE service in separate event loop."""
        try:
            self.bluetooth_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.bluetooth_loop)
            
            # TODO: Implement actual Bluetooth LE server
            self.get_logger().info("Bluetooth LE service started (mock mode)")
            self.bluetooth_connected = True
            
            # Keep the Bluetooth event loop running
            self.bluetooth_loop.run_forever()
            
        except Exception as e:
            self.get_logger().error(f"Bluetooth service error: {e}")
            self.bluetooth_connected = False


def main(args=None):
    """Main entry point for Bluetooth bridge node."""
    rclpy.init(args=args)
    
    try:
        bridge_node = BluetoothBridgeNode()
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error initializing Bluetooth bridge: {e}")
    finally:
        try:
            bridge_node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()