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
    from bluez_peripheral.gatt.service import Service
    from bluez_peripheral.gatt.characteristic import characteristic, CharacteristicFlags as CharFlags
    from bluez_peripheral.advert import Advertisement, AdvertisingIncludes
    from bluez_peripheral.agent import NoIoAgent
    from bluez_peripheral.util import get_message_bus, Adapter
    BLUETOOTH_AVAILABLE = True
    print("INFO: bluez-peripheral library available for BLE server functionality")
                
except ImportError as e:
    BLUETOOTH_AVAILABLE = False
    print(f"WARNING: bluez-peripheral library not fully available. Error: {e}")
    print("Install with: pip3 install bluez-peripheral")

# Bluetooth LE Service and Characteristic UUIDs for Racing Game
RACING_SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
COMMAND_CHAR_UUID = "87654321-4321-4321-4321-cba987654321"  # iPhone -> Robot commands
SENSOR_CHAR_UUID = "11111111-2222-3333-4444-555555555555"   # Robot -> iPhone sensor data
DEVICE_NAME_PREFIX = "YahboomRacer"


if BLUETOOTH_AVAILABLE:
    class RacingService(Service):
        """BLE GATT Service for iPhone AR app to robot communication."""
        
        def __init__(self, bridge_node):
            # Initialize racing service with custom UUID
            super().__init__(RACING_SERVICE_UUID, primary=True)
            self.bridge_node = bridge_node
        
        @characteristic(COMMAND_CHAR_UUID, CharFlags.WRITE | CharFlags.WRITE_WITHOUT_RESPONSE)
        def racing_command(self, options):
            """Placeholder for command characteristic (write-only)."""
            pass
        
        @racing_command.setter
        def racing_command(self, value, options):
            """Handle incoming command from iPhone AR app."""
            try:
                message = bytes(value).decode('utf-8')
                self.bridge_node.get_logger().info(f"📱 Received from iPhone: {message[:100]}...")
                
                # Process the message through existing JSON parser
                self.bridge_node._parse_json_messages(message + '\n')  # Add newline for delimiter
                
                # Track connection
                self.bridge_node.bluetooth_connected = True
                
            except Exception as e:
                self.bridge_node.get_logger().error(f"Command processing error: {e}")
        
        @characteristic(SENSOR_CHAR_UUID, CharFlags.READ | CharFlags.NOTIFY)
        def sensor_feedback(self, options):
            """Provide current sensor data when iPhone reads characteristic."""
            try:
                # Update timestamp
                self.bridge_node.robot_state['timestamp'] = self.bridge_node.get_clock().now().nanoseconds / 1e9
                sensor_data = json.dumps(self.bridge_node.robot_state)
                return sensor_data.encode('utf-8')
            except Exception as e:
                self.bridge_node.get_logger().error(f"Sensor read error: {e}")
                return b'{"error": "sensor_read_failed"}'
        
        def update_sensor_data(self):
            """Send notification to connected iPhone clients about sensor updates."""
            if hasattr(self, 'sensor_feedback'):
                try:
                    # Update timestamp
                    self.bridge_node.robot_state['timestamp'] = self.bridge_node.get_clock().now().nanoseconds / 1e9
                    sensor_json = json.dumps(self.bridge_node.robot_state)
                    data = sensor_json.encode('utf-8')
                    
                    # Trigger notification
                    self.sensor_feedback.changed(data)
                    self.bridge_node.get_logger().debug(f"📡 Sensor notification: {sensor_json[:50]}...")
                    
                except Exception as e:
                    self.bridge_node.get_logger().error(f"Sensor notification error: {e}")

else:
    # Fallback class when bluez-peripheral is not available
    class RacingService:
        """Dummy BLE service for when Bluetooth is not available."""
        
        def __init__(self, bridge_node):
            self.bridge_node = bridge_node
        
        def update_sensor_data(self):
            """Dummy method - does nothing when Bluetooth is not available."""
            pass


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
        self.racing_service = None
        self.ble_advertisement = None
        self.device_name = f"{DEVICE_NAME_PREFIX}_Car{self.car_id}"
        self.dbus_bus = None
        
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
            self.get_logger().info("Bluetooth client library available")
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
        if not self.bluetooth_connected or not self.racing_service:
            return
            
        # Update timestamp
        self.robot_state['timestamp'] = self.get_clock().now().nanoseconds / 1e9
        
        # Racing service handles sensor updates to connected iPhones
        # This happens automatically through BLE notifications
        pass
            
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
            
            # Run the BLE setup
            self.bluetooth_loop.run_until_complete(self._setup_ble_service())
            
        except Exception as e:
            self.get_logger().error(f"Bluetooth service error: {e}")
            self.bluetooth_connected = False
            
    async def _setup_ble_service(self):
        """Setup Bluetooth LE communication service using bluez-peripheral."""
        try:
            self.get_logger().info(f"🔵 Starting BLE server: {self.device_name}")
            
            # Suppress D-Bus TxPower warnings (known BlueZ compatibility issue)
            import logging
            logging.getLogger('root').setLevel(logging.CRITICAL)
            
            # Get D-Bus message bus
            self.dbus_bus = await get_message_bus()
            
            # Create and register racing service
            self.racing_service = RacingService(self)
            await self.racing_service.register(self.dbus_bus)
            
            # Register agent for pairing (no authentication required)
            agent = NoIoAgent()
            await agent.register(self.dbus_bus)
            
            # Get first Bluetooth adapter
            adapter = await Adapter.get_first(self.dbus_bus)
            
            # Create and register advertisement
            # Note: TxPower D-Bus warnings are expected with some BlueZ versions
            self.ble_advertisement = Advertisement(
                localName=self.device_name,
                serviceUUIDs=[RACING_SERVICE_UUID],
                appearance=0x0000,  # Generic device
                timeout=60,  # Advertise for 60 seconds, then renew
                includes=AdvertisingIncludes.NONE  # Avoid TxPower compatibility issues
            )
            
            try:
                await self.ble_advertisement.register(self.dbus_bus, adapter)
                self.get_logger().info("📡 Advertisement registered successfully")
            except Exception as adv_error:
                # Advertisement registration might have D-Bus warnings but still work
                self.get_logger().warn(f"Advertisement registration warning (service may still work): {adv_error}")
            
            # Restore normal logging
            logging.getLogger('root').setLevel(logging.WARNING)
            
            self.get_logger().info(f"✅ BLE server setup complete")
            self.get_logger().info(f"   📱 Device: {self.device_name}")
            self.get_logger().info(f"   🔵 Service: {RACING_SERVICE_UUID}")
            self.get_logger().info(f"   📝 Command: {COMMAND_CHAR_UUID}")
            self.get_logger().info(f"   📊 Sensors: {SENSOR_CHAR_UUID}")
            self.get_logger().info(f"   💡 Ready for iPhone AR app connections!")
            self.get_logger().info(f"   ℹ️  Note: TxPower D-Bus errors above can be safely ignored")
            
            # Mark as successfully connected
            self.bluetooth_connected = True
            
            # Keep service running and periodically send sensor updates
            while rclpy.ok():
                await asyncio.sleep(1.0)
                
                # Send sensor updates to connected iPhone clients
                if self.bluetooth_connected and self.racing_service:
                    self.racing_service.update_sensor_data()
                    
        except Exception as e:
            self.get_logger().error(f"BLE setup failed: {e}")
            import traceback
            self.get_logger().error(f"Error details: {traceback.format_exc()}")
            self.bluetooth_connected = False
            # Fallback to development mode
            while rclpy.ok():
                await asyncio.sleep(5.0)
                self.get_logger().debug("BLE failed - running in fallback mode")

            
    def _simulate_iphone_command(self):
        """Simulate receiving a command from iPhone for testing ROS2 integration."""
        test_command = {
            "msg_type": "robot_command",
            "data": {
                "movement": {
                    "linear": {"x": 0.3, "y": 0.0},
                    "angular": {"z": 0.5}
                },
                "game_effects": {
                    "power_up": None,
                    "collision": None
                }
            }
        }
        
        self.get_logger().info("🧪 SIMULATING iPhone command for testing")
        self._process_iphone_command(test_command)


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