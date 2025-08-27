#!/usr/bin/env python3

"""
Bluetooth ROS2 Bridge Node - AR App to Robot Communication
==========================================================

This node bridges iPhone AR app commands to ROS2 robot control system using
proven BLE server logic from ble_server.py combined with ROS2 integration.

Architecture:
iPhone AR App ←→ BLE Server (bless) ←→ This Bridge ←→ ROS2 Topics ←→ Robot

Key Features:
- Uses proven bless BLE server architecture 
- Fire-and-forget movement commands (high frequency)
- Polling-based sensor feedback (AR app controlled frequency)
- Separate characteristics for commands vs sensor data
- Multiplayer support (car_id parameter)

Message Flow:
- AR App → JSON commands → ROS2 cmd_vel → Robot movement
- Robot → ROS2 sensors → JSON status → AR App (when polled)

Author: Yahboom R2L Racing Team
Based on: Working ble_server.py + bluetooth_bridge_node.py concepts
"""

import argparse
import asyncio
import json
import logging
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
import rclpy.parameter
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu

# BLE imports - using proven bless library from ble_server.py
try:
    from bless import (
        BlessServer,
        BlessGATTCharacteristic,
        GATTCharacteristicProperties,
        GATTAttributePermissions,
    )
    BLESS_AVAILABLE = True
except ImportError:
    BLESS_AVAILABLE = False
    print("WARNING: bless library not available. Install with: pip install bless")

# BLE Service and Characteristic UUIDs
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
STATUS_CHAR_UUID = "11111111-2222-3333-4444-555555555555"   # Single characteristic for commands and sensor data

# Configure logging for BLE operations
logger = logging.getLogger(__name__)


class BluetoothROS2Bridge(Node):
    """
    ROS2 node that bridges iPhone AR app to robot using proven BLE server logic.
    
    Combines working bless BLE server from ble_server.py with ROS2 integration
    for reliable AR app to robot communication in racing scenarios.
    """
    
    def __init__(self):
        super().__init__('bluetooth_ros2_bridge')
        
        # Get car_id parameter for multiplayer racing
        self.declare_parameter('car_id', 1)
        self.car_id = self.get_parameter('car_id').get_parameter_value().integer_value
        
        # Get jetson_mode parameter for Jetson Nano compatibility
        self.declare_parameter('jetson_mode', False)
        self.jetson_mode = self.get_parameter('jetson_mode').get_parameter_value().bool_value
        
        # Validate car_id (1-4 for racing)
        if not 1 <= self.car_id <= 4:
            self.get_logger().error(f"Invalid car_id: {self.car_id}. Must be between 1-4.")
            raise ValueError(f"car_id must be between 1-4, got {self.car_id}")
        
        self.device_name = f"YahboomRacer_Car{self.car_id}"
        self.get_logger().info(f"Initializing Bluetooth ROS2 bridge for {self.device_name}")
        if self.jetson_mode:
            self.get_logger().info("🤖 Jetson Nano compatibility mode enabled (BlueZ 5.53)")
        
        # Setup ROS2 publishers and subscribers
        self._setup_ros2_interface()
        
        # Robot sensor state (collected from ROS2, sent to AR app)
        self.robot_sensors = {
            'car_id': self.car_id,
            'battery_voltage': 0.0,
            'emergency_state': False,
            'speed': 0.0,
            'imu': {
                'angular_velocity': {'z': 0.0},
                'linear_acceleration': {'x': 0.0, 'y': 0.0}
            },
            'timestamp': 0.0,
            'connection_status': 'connected'
        }
        
        # BLE server state
        self.ble_server = None
        self.ble_connected = False
        self.command_count = 0
        self.last_command_time = time.time()
        
        # Pending response system for command responses
        self.pending_response = None
        self.start_time = time.time()
        
        # Start BLE server if available
        if BLESS_AVAILABLE:
            self.get_logger().info("✅ bless library available - starting BLE server")
            self._start_ble_server()
        else:
            self.get_logger().error("❌ bless library not available - BLE disabled")
            self.get_logger().error("Install with: pip install bless")
            
        self.get_logger().info(f"🚀 Bluetooth ROS2 bridge ready for {self.device_name}")
    
    def _setup_ros2_interface(self):
        """Setup ROS2 publishers and subscribers for robot communication."""
        
        # Publisher for robot movement commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',  # Namespaced to /car_X/cmd_vel by launch file  
            10
        )
        
        # Subscribers for robot sensor feedback
        self.voltage_subscriber = self.create_subscription(
            Float32,
            'voltage',  # Namespaced to /car_X/voltage
            self._voltage_callback,
            10
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu, 
            'imu/imu_raw',  # Namespaced to /car_X/imu/imu_raw
            self._imu_callback,
            10
        )
        
        self.emergency_subscriber = self.create_subscription(
            Bool,
            'emergency_stop',  # Namespaced to /car_X/emergency_stop
            self._emergency_callback,
            10
        )
        
        # Timer to update robot sensor data timestamp
        self.sensor_timer = self.create_timer(0.1, self._update_sensor_timestamp)
        
        self.get_logger().info("✅ ROS2 interface configured")
    
    # ROS2 Callbacks - Update robot sensor state
    def _voltage_callback(self, msg: Float32):
        """Update battery voltage from robot."""
        self.robot_sensors['battery_voltage'] = msg.data
    
    def _imu_callback(self, msg: Imu):
        """Update IMU data for AR positioning."""
        self.robot_sensors['imu'] = {
            'angular_velocity': {'z': msg.angular_velocity.z},
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y
            }
        }
    
    def _emergency_callback(self, msg: Bool):
        """Update emergency stop state."""
        self.robot_sensors['emergency_state'] = msg.data
    
    def _update_sensor_timestamp(self):
        """Update sensor data timestamp for AR app."""
        self.robot_sensors['timestamp'] = self.get_clock().now().nanoseconds / 1e9
    
    def _create_short_sensor_response(self) -> Dict:
        """Create shortened sensor response to fit BLE packet limits."""
        # Short format: {"t": "sens", "d": [bat, emg, spd, imu_z, imu_x, imu_y], "id": car_id, "up": uptime}
        return {
            "t": "sens",
            "d": [
                round(self.robot_sensors['battery_voltage'], 2),
                1 if self.robot_sensors['emergency_state'] else 0,
                round(self.robot_sensors['speed'], 2),
                round(self.robot_sensors['imu']['angular_velocity']['z'], 3),
                round(self.robot_sensors['imu']['linear_acceleration']['x'], 3),
                round(self.robot_sensors['imu']['linear_acceleration']['y'], 3)
            ],
            "id": self.car_id,
            "up": int(time.time() - self.start_time),
            "cmd": self.command_count
        }
    
    # AR App Command Processing
    def _process_ar_command(self, command_str: str) -> Optional[Dict]:
        """
        Process incoming command from AR app.
        
        Supports both simple commands (from ble_server.py tests) and 
        JSON commands (from AR app).
        """
        self.command_count += 1
        self.last_command_time = time.time()
        
        command_str = command_str.strip()
        self.get_logger().info(f"📱 AR command #{self.command_count}: {command_str[:100]}")
        
        try:
            # Try JSON format first (AR app format)
            if command_str.startswith('{'):
                return self._process_json_command(command_str)
            else:
                # Simple command format (for testing compatibility)
                return self._process_simple_command(command_str)
                
        except Exception as e:
            self.get_logger().error(f"Command processing error: {e}")
            return {"t": "err", "msg": str(e)[:30]}
    
    def _process_json_command(self, json_str: str) -> Optional[Dict]:
        """Process JSON command from AR app."""
        try:
            command = json.loads(json_str)
            
            # Handle new shorter format: {"cmd": "move", "lin": [x,y,z], "ang": [x,y,z]}
            if command.get('cmd') == 'move':
                return self._process_short_movement_command(command)
            
            # Handle original format: {"msg_type": "robot_command", "data": {...}}
            elif command.get('msg_type') == 'robot_command':
                return self._process_long_movement_command(command)
            
            else:
                self.get_logger().warn(f"Unknown command format: {list(command.keys())}")
                return None
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON from AR app: {e}")
            return {"t": "err", "msg": "bad_json"}
    
    def _process_short_movement_command(self, command: dict) -> Optional[Dict]:
        """Process shortened movement command: {"cmd": "move", "lin": [x,y,z], "ang": [x,y,z]}"""
        try:
            # Extract linear and angular arrays
            linear_array = command.get('lin', [0.0, 0.0, 0.0])
            angular_array = command.get('ang', [0.0, 0.0, 0.0])
            
            # Ensure arrays have 3 elements
            if len(linear_array) < 3:
                linear_array.extend([0.0] * (3 - len(linear_array)))
            if len(angular_array) < 3:
                angular_array.extend([0.0] * (3 - len(angular_array)))
            
            # Create and publish Twist message
            twist_msg = Twist()
            twist_msg.linear.x = float(linear_array[0])
            twist_msg.linear.y = float(linear_array[1])  
            twist_msg.linear.z = float(linear_array[2])
            twist_msg.angular.x = float(angular_array[0])
            twist_msg.angular.y = float(angular_array[1])
            twist_msg.angular.z = float(angular_array[2])
            
            # Publish to robot
            self.cmd_vel_publisher.publish(twist_msg)
            self.robot_sensors['speed'] = abs(twist_msg.linear.x)  # Update current speed
            
            self.get_logger().debug(
                f"Published short cmd_vel: linear=({twist_msg.linear.x:.2f}, {twist_msg.linear.y:.2f}), "
                f"angular={twist_msg.angular.z:.2f}"
            )
            
            # Handle game effects if present
            game_effects = command.get('fx', {})
            if game_effects:
                self.get_logger().debug(f"Game effects: {game_effects}")
            
            # Fire-and-forget: No response needed for movement commands
            return None
            
        except (ValueError, IndexError, KeyError) as e:
            self.get_logger().error(f"Error processing short movement command: {e}")
            return {"t": "err", "msg": "bad_move"}
    
    def _process_long_movement_command(self, command: dict) -> Optional[Dict]:
        """Process original movement command: {"msg_type": "robot_command", "data": {...}}"""
        try:
            # Extract movement data
            data = command.get('data', {})
            movement = data.get('movement', {})
            
            if movement:
                # Create and publish Twist message
                twist_msg = Twist()
                
                linear = movement.get('linear', {})
                angular = movement.get('angular', {})
                
                twist_msg.linear.x = float(linear.get('x', 0.0))
                twist_msg.linear.y = float(linear.get('y', 0.0))  
                twist_msg.linear.z = float(linear.get('z', 0.0))
                twist_msg.angular.x = float(angular.get('x', 0.0))
                twist_msg.angular.y = float(angular.get('y', 0.0))
                twist_msg.angular.z = float(angular.get('z', 0.0))
                
                # Publish to robot
                self.cmd_vel_publisher.publish(twist_msg)
                self.robot_sensors['speed'] = abs(twist_msg.linear.x)  # Update current speed
                
                self.get_logger().debug(
                    f"Published long cmd_vel: linear=({twist_msg.linear.x:.2f}, {twist_msg.linear.y:.2f}), "
                    f"angular={twist_msg.angular.z:.2f}"
                )
            
            # Handle game effects (future enhancement)
            game_effects = data.get('game_effects', {})
            if game_effects:
                self.get_logger().debug(f"Game effects: {game_effects}")
            
            # Fire-and-forget: No response needed for movement commands
            return None
            
        except (ValueError, KeyError) as e:
            self.get_logger().error(f"Error processing long movement command: {e}")
            return {"t": "err", "msg": "bad_move"}
    
    def _process_simple_command(self, command: str) -> Optional[Dict]:
        """Process simple text commands (for testing compatibility)."""
        command_lower = command.lower().strip()
        
        # Movement commands - publish to ROS2, no response
        if command_lower.startswith(('move', 'cmd_vel')):
            # Parse simple movement commands like "move_forward" or "cmd_vel:0.5,0.2"
            if ':' in command_lower:
                # Format: "cmd_vel:linear_x,angular_z"
                try:
                    _, params = command_lower.split(':', 1)
                    values = [float(x) for x in params.split(',')]
                    
                    twist_msg = Twist()
                    twist_msg.linear.x = values[0] if len(values) > 0 else 0.0
                    twist_msg.angular.z = values[1] if len(values) > 1 else 0.0
                    
                    self.cmd_vel_publisher.publish(twist_msg)
                    self.robot_sensors['speed'] = abs(twist_msg.linear.x)
                    
                    self.get_logger().debug(f"Simple cmd_vel: {twist_msg.linear.x:.2f}, {twist_msg.angular.z:.2f}")
                    return None  # Fire-and-forget
                    
                except (ValueError, IndexError) as e:
                    return {"t": "err", "msg": "bad_vel"}
            else:
                # Simple directional commands
                twist_msg = Twist()
                if 'forward' in command_lower:
                    twist_msg.linear.x = 0.3
                elif 'backward' in command_lower:
                    twist_msg.linear.x = -0.3
                elif 'left' in command_lower:
                    twist_msg.angular.z = 0.5
                elif 'right' in command_lower:
                    twist_msg.angular.z = -0.5
                
                self.cmd_vel_publisher.publish(twist_msg)
                self.robot_sensors['speed'] = abs(twist_msg.linear.x)
                return None  # Fire-and-forget
        
        # Status queries - return sensor data
        elif command_lower in ('status', 'get_sensors', 'sensors'):
            return self._create_short_sensor_response()
        
        # Test commands - return short responses (for ble_client_test.py compatibility)
        elif command_lower == 'ping':
            return {"t": "ping", "msg": "pong", "id": self.car_id}
        
        elif command_lower == 'hello':
            return {"t": "hello", "msg": "hi", "id": self.car_id}
        
        # Emergency commands
        elif command_lower == 'emergency_stop':
            # Publish zero velocity
            twist_msg = Twist()  # All zeros
            self.cmd_vel_publisher.publish(twist_msg)
            self.robot_sensors['speed'] = 0.0
            
            return {"t": "emg", "msg": "stopped", "id": self.car_id}
        
        # Unknown command
        else:
            return {"t": "err", "msg": "unknown", "cmd": command[:20]}
    
    # BLE Server Setup (adapted from ble_server.py)
    def _start_ble_server(self):
        """Start BLE server in separate thread."""
        self.ble_thread = threading.Thread(target=self._run_ble_server)
        self.ble_thread.daemon = True
        self.ble_thread.start()
    
    def _run_ble_server(self):
        """Run BLE server event loop."""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._setup_ble_server(loop))
        except Exception as e:
            self.get_logger().error(f"BLE server error: {e}")
            self.ble_connected = False

    async def _setup_ble_server(self, loop):
        """Setup BLE server with characteristics (adapted from ble_server.py)."""
        try:
            self.get_logger().info(f"🔵 Setting up BLE server: {self.device_name}")
            
            # Create BLE server
            self.ble_server = BlessServer(name=self.device_name, loop=loop)
            
            # Set up callbacks (must be done before adding service)
            self.ble_server.read_request_func = self._ble_read_callback
            self.ble_server.write_request_func = self._ble_write_callback
            
            # Add service
            await self.ble_server.add_new_service(SERVICE_UUID)
            
            # Add single status characteristic (AR app reads and writes)
            # Configure properties based on Jetson compatibility mode
            char_properties = (
                GATTCharacteristicProperties.read
                | GATTCharacteristicProperties.write
            )
            if self.jetson_mode:
                char_properties |= GATTCharacteristicProperties.write_without_response
                self.get_logger().info("✅ Jetson mode: Added write_without_response compatibility")
            
            char_permissions = (
                GATTAttributePermissions.readable 
                | GATTAttributePermissions.writeable
            )
            
            await self.ble_server.add_new_characteristic(
                SERVICE_UUID,
                STATUS_CHAR_UUID,
                char_properties,
                None,
                char_permissions
            )
            
            # Start server
            await self.ble_server.start()
            self.ble_connected = True
            
            self.get_logger().info("🎉 BLE server started successfully!")
            self.get_logger().info(f"📡 Device: {self.device_name}")
            self.get_logger().info(f"🔵 Service: {SERVICE_UUID}")
            self.get_logger().info(f"📱 Status: {STATUS_CHAR_UUID} (AR app reads/writes)")
            self.get_logger().info("🤖 Ready for iPhone AR app connections!")
            self.get_logger().info("📋 Single characteristic - simpler integration!")
            
            # Keep server running
            while True:
                await asyncio.sleep(1.0)
                
        except Exception as e:
            self.get_logger().error(f"BLE server setup failed: {e}")
            self.ble_connected = False

    # BLE Callbacks (single characteristic with pending response system)
    def _ble_read_callback(self, characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
        """Handle BLE read requests from AR app - returns command responses or sensor data."""
        self.get_logger().debug(f"📖 Read request for characteristic: {characteristic.uuid}")
        
        try:
            # Check if we have a pending response from a previous command
            if self.pending_response:
                response_data = self.pending_response
                self.pending_response = None  # Clear after sending
                
                self.get_logger().info(f"📤 Sending command response: {response_data['type']}")
            else:
                # Default: return current robot sensor data in short format
                response_data = self._create_short_sensor_response()
                self.get_logger().debug(f"📤 Sending sensor data")
            
            # Convert to JSON and return as bytearray (no pretty printing to save space)
            response_json = json.dumps(response_data)
            response_bytes = bytearray(response_json.encode('utf-8'))
            
            # Update the characteristic value
            characteristic.value = response_bytes
            
            self.get_logger().debug(f"📤 Response sent ({len(response_bytes)} bytes)")
            return response_bytes
            
        except Exception as e:
            self.get_logger().error(f"❌ Read request failed: {e}")
            error_response = {"t": "err", "msg": str(e)[:30]}
            return bytearray(json.dumps(error_response).encode('utf-8'))

    def _ble_write_callback(self, characteristic: BlessGATTCharacteristic, value: Any, **kwargs):
        """Handle BLE write requests from AR app."""
        self.get_logger().debug(f"✍️  Write request for characteristic: {characteristic.uuid}")
        
        try:
            # Update the characteristic value
            characteristic.value = value
            
            # Decode the incoming command
            if isinstance(value, (bytes, bytearray)):
                command = value.decode('utf-8', errors='ignore')
            else:
                command = str(value)
            
            self.command_count += 1
            self.last_command_time = time.time()
            
            self.get_logger().info(f"📨 Received command #{self.command_count}: '{command[:100]}'")
            
            # Process command and prepare response
            response = self._process_ar_command(command)
            
            # Store response for next read (or None for fire-and-forget commands)
            self.pending_response = response
            
            if response:
                self.get_logger().info(f"🔄 Command processed - Response ready: {response['type']}")
            else:
                self.get_logger().debug(f"🔄 Command processed - Fire-and-forget (no response)")
                
            self.get_logger().debug("✅ Write request processed successfully")
            
        except Exception as e:
            self.get_logger().error(f"❌ Write request failed: {e}")
            # Still queue an error response
            self.pending_response = {"t": "err", "msg": "proc_fail"}
            raise


def main(args=None):
    """Main entry point for Bluetooth ROS2 bridge."""
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Bluetooth ROS2 Bridge - AR App to Robot Communication')
    parser.add_argument('--jetson', action='store_true',
                       help='Enable Jetson Nano compatibility mode (BlueZ 5.53)')
    parser.add_argument('--car-id', type=int, default=1,
                       help='Car ID for multiplayer racing (1-4)')
    
    # Parse known args to allow ROS2 arguments to pass through
    parsed_args, unknown_args = parser.parse_known_args()
    
    # Combine ROS2 args with our parsed args
    ros_args = args if args is not None else []
    ros_args.extend(unknown_args)
    
    # Print startup info
    print("🚀 Starting Bluetooth ROS2 Bridge")
    print(f"   Car ID: {parsed_args.car_id}")
    if parsed_args.jetson:
        print("   🤖 Jetson Nano compatibility mode enabled")
    print()
    
    # Initialize ROS2 with parameter overrides
    rclpy.init(args=ros_args)
    
    try:
        # Create bridge with parameter overrides
        bridge = BluetoothROS2Bridge()
        
        # Override parameters from command line args
        bridge.set_parameters([
            rclpy.parameter.Parameter('car_id', rclpy.Parameter.Type.INTEGER, parsed_args.car_id),
            rclpy.parameter.Parameter('jetson_mode', rclpy.Parameter.Type.BOOL, parsed_args.jetson)
        ])
        
        # Re-read parameters after override
        bridge.car_id = bridge.get_parameter('car_id').get_parameter_value().integer_value
        bridge.jetson_mode = bridge.get_parameter('jetson_mode').get_parameter_value().bool_value
        
        # Update device name if car_id changed
        bridge.device_name = f"YahboomRacer_Car{bridge.car_id}"
        
        bridge.get_logger().info(f"🎯 Final configuration: Car {bridge.car_id}, Jetson mode: {bridge.jetson_mode}")
        
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Bridge initialization error: {e}")
    finally:
        try:
            bridge.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()