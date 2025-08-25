#!/usr/bin/env python3

"""
Intelligent BLE Server for Yahboom Robot - Bidirectional Messaging
==================================================================

A complete BLE server with intelligent conversation capabilities and proper GATT structure.
This version supports bidirectional messaging with command processing and intelligent responses,
perfect for Unity mobile app integration and racing control applications.

Features:
    - Full BLE GATT structure (Service + Characteristics + Properties)
    - Intelligent command processing (ping, status, movement, sensors, etc.)
    - Conversation state management with round tracking
    - JSON-based bidirectional communication
    - Racing control commands (emergency_stop, speed control)
    - Comprehensive logging and error handling

Supported Commands:
    - "ping" -> "pong (round X)"
    - "status" -> detailed robot status
    - "move_forward" -> movement acknowledgment
    - "get_sensors" -> simulated sensor data
    - "hello" -> greeting response
    - "emergency_stop" -> critical stop acknowledgment
    - "speed 50" -> speed setting confirmation

Requirements:
    pip install -r bluetooth_requirements.txt

Usage:
    python3 ble_server.py

Author: Yahboom Robot Bluetooth Team
"""

import asyncio
import logging
import json
from datetime import datetime
from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('ble_server.log')
    ]
)
logger = logging.getLogger(__name__)

# Service and Characteristic UUIDs
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
STATUS_CHAR_UUID = "11111111-2222-3333-4444-555555555555"

class YahboomBLEServer:
    def __init__(self):
        self.server = BlessServer(name="YahboomRobot")
        self.message_count = 0
        self.start_time = datetime.now()
        
        # Conversation state management for bidirectional messaging
        self.conversation_state = {
            "round": 0,
            "last_command": None,
            "last_response": None,
            "pending_response": None,
            "conversation_history": [],
            "session_id": datetime.now().strftime("%Y%m%d_%H%M%S")
        }
        

    def read_request_callback(self, characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
        """Handle read requests from clients - now with intelligent conversation responses"""
        logger.info(f"📖 Read request for characteristic: {characteristic.uuid}")
        
        try:
            # Check if we have a pending response from a previous command
            if self.conversation_state["pending_response"]:
                response_data = self.conversation_state["pending_response"]
                self.conversation_state["last_response"] = response_data["content"]
                self.conversation_state["pending_response"] = None  # Clear after sending
                
                logger.info(f"📤 Sending command response: {response_data['type']} - {response_data['content']}")
            else:
                # Default status response when no specific command was received
                response_data = {
                    "type": "status",
                    "content": {
                        "server_name": "YahboomRobot",
                        "status": "running",
                        "uptime_seconds": int((datetime.now() - self.start_time).total_seconds()),
                        "session_id": self.conversation_state["session_id"]
                    },
                    "conversation_round": self.conversation_state["round"],
                    "timestamp": datetime.now().isoformat(),
                    "message_count": self.message_count
                }
                
                logger.info(f"📤 Sending default status data")
            
            # Convert to JSON and then to bytearray
            response_json = json.dumps(response_data, indent=2)
            response_bytes = bytearray(response_json.encode('utf-8'))
            
            # Update the characteristic value
            characteristic.value = response_bytes
            
            # Log conversation history
            self.conversation_state["conversation_history"].append({
                "direction": "outbound",
                "content": response_data,
                "timestamp": datetime.now().isoformat()
            })
            
            logger.info(f"📤 Response sent ({len(response_bytes)} bytes) - Round {self.conversation_state['round']}")
            
            return response_bytes
            
        except Exception as e:
            logger.error(f"❌ Read request failed: {e}")
            error_response = bytearray(json.dumps({
                "type": "error", 
                "content": str(e),
                "timestamp": datetime.now().isoformat()
            }).encode('utf-8'))
            return error_response
        
    def write_request_callback(self, characteristic: BlessGATTCharacteristic, value, **kwargs):
        """Handle write requests from clients - now with intelligent command processing"""
        logger.info(f"✍️  Write request for characteristic: {characteristic.uuid}")
        
        try:
            # Update the characteristic value
            characteristic.value = value
            
            # Decode the incoming data
            if isinstance(value, (bytes, bytearray)):
                message = value.decode('utf-8', errors='ignore')
            else:
                message = str(value)
            
            self.message_count += 1
            self.conversation_state["round"] += 1
            
            logger.info(f"📨 Received command #{self.message_count} (Round {self.conversation_state['round']}): '{message}'")
            
            # Log conversation history
            self.conversation_state["conversation_history"].append({
                "direction": "inbound",
                "content": message,
                "timestamp": datetime.now().isoformat(),
                "round": self.conversation_state["round"]
            })
            
            # Process command and prepare response
            response = self._process_command(message)
            self.conversation_state["last_command"] = message
            self.conversation_state["pending_response"] = response
            
            logger.info(f"🔄 Command processed - Response ready: {response['type']}")
            
            # Log to file as well
            try:
                with open("ble_messages.log", "a", encoding='utf-8') as f:
                    timestamp = datetime.now().isoformat()
                    f.write(f"[{timestamp}] Round {self.conversation_state['round']} - Command: {message} -> Response: {response['content']}\n")
                    f.flush()
            except Exception as log_error:
                logger.warning(f"⚠️  Failed to log conversation to file: {log_error}")
            
            logger.info("✅ Write request processed successfully - Response queued")
            
        except Exception as e:
            logger.error(f"❌ Write request failed: {e}")
            # Still queue an error response
            self.conversation_state["pending_response"] = {
                "type": "error",
                "content": f"Command processing failed: {str(e)}",
                "conversation_round": self.conversation_state["round"],
                "timestamp": datetime.now().isoformat()
            }
            raise
            
    def _process_command(self, command: str) -> dict:
        """Process incoming commands and generate appropriate responses"""
        command_lower = command.lower().strip()
        current_round = self.conversation_state["round"]
        
        logger.info(f"🧠 Processing command: '{command}' (Round {current_round})")
        
        # Command-Response Mapping for bidirectional testing
        if command_lower == "ping":
            response = {
                "type": "ping_response",
                "content": f"pong (round {current_round})",
                "conversation_round": current_round,
                "timestamp": datetime.now().isoformat()
            }
            
        elif command_lower == "status":
            response = {
                "type": "status_response", 
                "content": {
                    "server_name": "YahboomRobot",
                    "status": "running",
                    "uptime_seconds": int((datetime.now() - self.start_time).total_seconds()),
                    "total_messages": self.message_count,
                    "conversation_rounds": current_round,
                    "session_id": self.conversation_state["session_id"],
                    "last_command": self.conversation_state["last_command"]
                },
                "conversation_round": current_round,
                "timestamp": datetime.now().isoformat()
            }
            
        elif command_lower.startswith("move"):
            # Simulate robot movement command
            movement = command_lower.replace("move_", "").replace("move ", "")
            response = {
                "type": "movement_response",
                "content": f"movement_acknowledged: {movement}",
                "movement_type": movement,
                "conversation_round": current_round,
                "timestamp": datetime.now().isoformat()
            }
            
        elif command_lower == "get_sensors":
            # Simulate sensor data response
            response = {
                "type": "sensor_response",
                "content": {
                    "battery_level": 85,
                    "motor_temp": 42.3,
                    "wifi_strength": -45,
                    "gps_status": "connected",
                    "camera_status": "streaming"
                },
                "conversation_round": current_round,
                "timestamp": datetime.now().isoformat()
            }
            
        elif command_lower == "hello":
            response = {
                "type": "greeting_response",
                "content": f"hello_from_yahboom_robot (round {current_round})",
                "conversation_round": current_round,
                "timestamp": datetime.now().isoformat()
            }
            
        elif command_lower == "emergency_stop":
            # Simulate emergency stop for racing scenarios
            response = {
                "type": "emergency_response",
                "content": "emergency_stop_acknowledged - all_systems_halted",
                "priority": "critical",
                "conversation_round": current_round,
                "timestamp": datetime.now().isoformat()
            }
            
        elif command_lower.startswith("speed"):
            # Parse speed command like "speed 50" or "speed:75"
            try:
                speed_str = command_lower.replace("speed", "").replace(":", "").strip()
                speed_value = int(speed_str)
                response = {
                    "type": "speed_response",
                    "content": f"speed_set_to_{speed_value}_percent",
                    "speed_value": speed_value,
                    "conversation_round": current_round,
                    "timestamp": datetime.now().isoformat()
                }
            except ValueError:
                response = {
                    "type": "error_response",
                    "content": "invalid_speed_format - use 'speed 50' or 'speed:75'",
                    "conversation_round": current_round,
                    "timestamp": datetime.now().isoformat()
                }
                
        else:
            # Unknown command - echo back with suggestion
            response = {
                "type": "unknown_command",
                "content": f"unknown_command: '{command}' - try: ping, status, move_forward, get_sensors, hello",
                "received_command": command,
                "conversation_round": current_round,
                "timestamp": datetime.now().isoformat()
            }
            
        logger.info(f"✅ Response generated: {response['type']} - {response['content']}")
        return response
            
    async def setup_server(self):
        """Initialize the BLE server with service and characteristics"""
        logger.info("⚙️  Setting up BLE server...")
        
        try:
            # Add the main service
            await self.server.add_new_service(SERVICE_UUID)
            logger.info(f"✅ Service added: {SERVICE_UUID}")
            
            # Add the status characteristic with proper API usage
            char_properties = (
                GATTCharacteristicProperties.read
                | GATTCharacteristicProperties.write
                | GATTCharacteristicProperties.notify
            )
            char_permissions = (
                GATTAttributePermissions.readable 
                | GATTAttributePermissions.writeable
            )
            
            await self.server.add_new_characteristic(
                SERVICE_UUID,           # service_uuid
                STATUS_CHAR_UUID,       # char_uuid  
                char_properties,        # properties
                None,                  # initial_value (can be None)
                char_permissions       # permissions
            )
            logger.info(f"✅ Characteristic added: {STATUS_CHAR_UUID}")
            
            # Set up required callbacks
            self.server.read_request_func = self.read_request_callback
            self.server.write_request_func = self.write_request_callback
            logger.info("✅ Callbacks configured")
            
            logger.info("✅ BLE server setup complete!")
            return True
            
        except Exception as e:
            logger.error(f"❌ Server setup failed: {e}")
            return False
            
    async def start_server(self):
        """Start the BLE server and begin advertising"""
        logger.info("🚀 Starting Yahboom BLE Server...")
        
        # Setup first
        if not await self.setup_server():
            return False
            
        try:
            # Start the server
            await self.server.start()
            
            logger.info("🎉 BLE Server started successfully!")
            logger.info(f"📡 Advertising as: YahboomRobot")
            logger.info(f"🔍 Service UUID: {SERVICE_UUID}")
            logger.info("📱 Use a BLE scanner app to find and connect!")
            logger.info("⏳ Server is running...")
            
            # Keep the server running
            while True:
                await asyncio.sleep(30)  # Heartbeat every 30 seconds
                uptime = datetime.now() - self.start_time
                logger.info(f"💓 Heartbeat - Uptime: {uptime}, Messages: {self.message_count}")
                
        except KeyboardInterrupt:
            logger.info("⏹️  Server interrupted by user")
        except Exception as e:
            logger.error(f"❌ Server error: {e}")
            return False
        finally:
            await self.stop_server()
            
        return True
        
    async def stop_server(self):
        """Stop the server"""
        logger.info("🔄 Stopping server...")
        try:
            await self.server.stop()
            logger.info("✅ Server stopped successfully")
        except Exception as e:
            logger.error(f"❌ Error stopping server: {e}")


async def main():
    """Main function to run the BLE server"""
    print("=" * 70)
    print("    🤖 Yahboom Robot BLE Server - Intelligent Conversations")
    print("=" * 70)
    print()
    print("🌟 This server features:")
    print("  • Advertise as 'YahboomRobot'")
    print("  • Bidirectional intelligent messaging")
    print("  • Command processing with smart responses")
    print("  • Conversation state tracking")
    print("  • Unity mobile app ready")
    print()
    print("📱 How to test:")
    print("  • Use 'nRF Connect' app on mobile")
    print("  • Or run: python3 ble_client_test.py (automated)")
    print("  • Look for 'YahboomRobot' device")
    print()
    print("🎯 Service UUID: 12345678-1234-1234-1234-123456789abc")
    print("📊 Status Characteristic: 11111111-2222-3333-4444-555555555555")
    print()
    print("💬 Supported Commands:")
    print("  • ping → pong (round X)")
    print("  • status → detailed robot status")
    print("  • move_forward → movement acknowledgment")
    print("  • get_sensors → sensor data")
    print("  • hello → greeting response")
    print("  • emergency_stop → critical stop")
    print("  • speed 50 → speed confirmation")
    print()
    print("Press Ctrl+C to stop")
    print("=" * 70)
    print()
    
    server = YahboomBLEServer()
    
    try:
        success = await server.start_server()
        if not success:
            print("❌ Failed to start server")
    except Exception as e:
        logger.error(f"💥 Unexpected error: {e}")
        print(f"\n❌ Error: {e}")
        print("\n🔧 Troubleshooting:")
        print("1. Ensure Bluetooth is enabled on your system")
        print("2. Try running with sudo on Linux")
        print("3. Check that no other BLE services are running")
        print("4. Verify bless library is properly installed")
        
    print("\n👋 Goodbye!")


if __name__ == "__main__":
    asyncio.run(main())
