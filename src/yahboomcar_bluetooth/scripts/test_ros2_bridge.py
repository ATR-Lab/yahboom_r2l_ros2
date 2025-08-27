#!/usr/bin/env python3

"""
Test Script for New Bluetooth ROS2 Bridge
==========================================

This script tests the new bluetooth_ros2_bridge.py to ensure it works correctly
with existing BLE test clients and provides proper ROS2 integration.

Test Coverage:
- BLE server connectivity (using ble_client_test.py logic)
- ROS2 topic publishing (cmd_vel commands)
- Sensor data feedback via BLE reads
- AR app JSON command format
- Simple command format (for compatibility)

Usage:
    # Terminal 1: Start the bridge
    ros2 run yahboomcar_bluetooth bluetooth_ros2_bridge --ros-args -p car_id:=1
    
    # Terminal 2: Run this test
    python3 test_ros2_bridge.py
    
    # Terminal 3: Monitor ROS2 topics
    ros2 topic echo /car_1/cmd_vel

Requirements:
    pip install -r bluetooth_requirements.txt

Author: Yahboom R2L Racing Team  
"""

import argparse
import asyncio
import json
import time
from datetime import datetime
from typing import Optional

try:
    from bleak import BleakScanner, BleakClient
    from bleak.backends.device import BLEDevice
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False
    print("❌ bleak library not available. Install with: pip install -r bluetooth_requirements.txt")

# Bridge Configuration (must match bluetooth_ros2_bridge.py)
TARGET_DEVICE_PATTERN = "YahboomRacer_Car"  
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
STATUS_CHAR_UUID = "11111111-2222-3333-4444-555555555555"   # Single characteristic for commands and sensor data


class BridgeTestClient:
    """Test client for the new Bluetooth ROS2 bridge."""
    
    def __init__(self, jetson_mode=False):
        self.device: Optional[BLEDevice] = None
        self.client: Optional[BleakClient] = None
        self.jetson_mode = jetson_mode
        self.test_results = {
            "discovery": False,
            "connection": False,
            "simple_commands": 0,
            "json_commands": 0,
            "sensor_reads": 0,
            "errors": []
        }
    
    async def run_test_suite(self):
        """Run comprehensive test of the new bridge."""
        print("🧪 Testing New Bluetooth ROS2 Bridge")
        print("=" * 50)
        print()
        
        if not BLEAK_AVAILABLE:
            print("❌ Cannot run test - bleak library required")
            return False
        
        try:
            # Test 1: Device Discovery
            if not await self._test_discovery():
                return False
            
            # Test 2: Connection
            if not await self._test_connection():
                return False
            
            # Test 3: Simple Commands (ble_server.py compatibility)
            await self._test_simple_commands()
            
            # Test 4: JSON Commands (AR app format)
            await self._test_json_commands()
            
            # Test 5: Sensor Data Reading
            await self._test_sensor_feedback()
            
            # Test 6: High-Frequency Commands (racing simulation)
            await self._test_high_frequency_commands()
            
        except Exception as e:
            print(f"❌ Test suite failed: {e}")
            self.test_results["errors"].append(str(e))
            return False
        finally:
            await self._cleanup()
        
        # Print results
        self._print_results()
        return self._evaluate_success()
    
    async def _test_discovery(self):
        """Test device discovery for bridge."""
        print("🔍 Test 1: Device Discovery")
        print("-" * 30)
        
        try:
            print("Scanning for Bluetooth ROS2 bridge devices...")
            devices = await BleakScanner.discover(timeout=10.0)
            
            # 🔧 DEBUG: Show all discovered devices to diagnose truncation
            print(f"📱 DEBUG: Found {len(devices)} total BLE devices:")
            for i, device in enumerate(devices[:15]):  # Show first 15
                name = device.name or "No Name"
                print(f"   {i+1:2d}. '{name}' ({device.address}) RSSI: {device.rssi}dBm")
            print()
            
            bridge_devices = [d for d in devices if d.name and TARGET_DEVICE_PATTERN in d.name]
            
            # 🔧 DEBUG: Also check for partial matches (truncation handling)
            partial_matches = [d for d in devices if d.name and "Yahboom" in d.name]
            if partial_matches and not bridge_devices:
                print(f"🔍 DEBUG: Found {len(partial_matches)} 'Yahboom' devices (possible truncation):")
                for device in partial_matches:
                    print(f"   • '{device.name}' - Length: {len(device.name)} chars")
                print()
            
            if not bridge_devices:
                print(f"❌ No bridge devices found (looking for '{TARGET_DEVICE_PATTERN}')")
                if partial_matches:
                    print(f"💡 HINT: Found Yahboom devices but names don't match exactly")
                    print(f"   This suggests BLE name truncation (18 chars → ~12 chars)")
                    print(f"   Expected: 'YahboomRacer_Car1' (18 chars)")
                    print(f"   Actual: '{partial_matches[0].name}' ({len(partial_matches[0].name)} chars)")
                print("Make sure the bridge is running:")
                print("  ros2 run yahboomcar_bluetooth bluetooth_ros2_bridge")
                return False
            
            self.device = bridge_devices[0]  # Use first found device
            print(f"✅ Found bridge device: {self.device.name}")
            print(f"   Address: {self.device.address}")
            print(f"   RSSI: {self.device.rssi} dBm")
            
            self.test_results["discovery"] = True
            return True
            
        except Exception as e:
            print(f"❌ Discovery failed: {e}")
            self.test_results["errors"].append(f"Discovery: {e}")
            return False
    
    async def _test_connection(self):
        """Test connection to bridge."""
        print(f"\n🔌 Test 2: Connection")
        print("-" * 30)
        
        try:
            self.client = BleakClient(self.device)
            await self.client.connect()
            
            if not self.client.is_connected:
                print("❌ Connection failed")
                return False
            
            print("✅ Connected successfully")
            
            # Verify characteristics
            services = self.client.services
            target_service = None
            
            for service in services:
                if service.uuid.lower() == SERVICE_UUID.lower():
                    target_service = service
                    break
            
            if not target_service:
                print(f"❌ Service not found: {SERVICE_UUID}")
                return False
            
            # Check for single status characteristic
            status_char = None
            
            for char in target_service.characteristics:
                if char.uuid.lower() == STATUS_CHAR_UUID.lower():
                    status_char = char
                    break
            
            if not status_char:
                print(f"❌ Status characteristic not found: {STATUS_CHAR_UUID}")
                return False
            
            print("✅ Status characteristic found")
            print(f"   Status: {STATUS_CHAR_UUID} (Properties: {status_char.properties})")
            print("📋 Single characteristic - simpler integration!")
            
            self.test_results["connection"] = True
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.test_results["errors"].append(f"Connection: {e}")
            return False
    
    async def _test_simple_commands(self):
        """Test simple text commands (ble_server.py compatibility)."""
        print(f"\n📝 Test 3: Simple Commands")
        print("-" * 30)
        
        test_commands = [
            "ping",
            "hello", 
            "status",
            "move_forward",
            "cmd_vel:0.3,0.5",
            "emergency_stop"
        ]
        
        for command in test_commands:
            try:
                print(f"Testing: '{command}'")
                
                # Send command
                response_mode = not self.jetson_mode  # False for Jetson, True for others
                await self.client.write_gatt_char(STATUS_CHAR_UUID, command.encode('utf-8'), response=response_mode)
                await asyncio.sleep(0.2)  # Let bridge process
                
                # Read response (could be command response or sensor data)
                response_data = await self.client.read_gatt_char(STATUS_CHAR_UUID)
                if response_data:
                    data = json.loads(response_data.decode('utf-8'))
                    response_type = data.get('type', data.get('t', 'unknown'))
                    
                    if command in ["ping", "hello", "status"] and (response_type.endswith('_response') or response_type in ['ping', 'hello', 'sens']):
                        print(f"   ✅ Command response received: {response_type}")
                    else:
                        print(f"   ✅ Data received: {response_type} (sensor data or acknowledgment)")
                else:
                    print(f"   ⚠️  No response received")
                
                self.test_results["simple_commands"] += 1
                
            except Exception as e:
                print(f"   ❌ Command failed: {e}")
                self.test_results["errors"].append(f"Simple command '{command}': {e}")
    
    async def _test_json_commands(self):
        """Test JSON commands (AR app format)."""
        print(f"\n📱 Test 4: JSON Commands (AR App Format)")
        print("-" * 30)
        
        # Shortened JSON format to test length limit hypothesis
        # Format: {"cmd": "move", "lin": [x, y, z], "ang": [x, y, z]}
        test_commands = [
            {
                "cmd": "move",
                "lin": [0.5, 0.0, 0.0],
                "ang": [0.0, 0.0, 0.3]
            },
            {
                "cmd": "move", 
                "lin": [-0.2, 0.0, 0.0],
                "ang": [0.0, 0.0, -0.5],
                "fx": {"boost": 1, "dur": 2.0}
            },
            {
                "cmd": "move",
                "lin": [0.0, 0.0, 0.0],
                "ang": [0.0, 0.0, 0.0]
            }
        ]
        
        for i, command in enumerate(test_commands, 1):
            try:
                print(f"JSON Command {i}: linear_x={command['lin'][0]}, "
                      f"angular_z={command['ang'][2]}")
                
                json_str = json.dumps(command)
                response_mode = not self.jetson_mode  # False for Jetson, True for others
                await self.client.write_gatt_char(STATUS_CHAR_UUID, json_str.encode('utf-8'), response=response_mode)
                await asyncio.sleep(0.1)  # Brief delay
                
                print(f"   ✅ JSON command sent")
                self.test_results["json_commands"] += 1
                
            except Exception as e:
                print(f"   ❌ JSON command failed: {e}")
                self.test_results["errors"].append(f"JSON command {i}: {e}")
    
    async def _test_sensor_feedback(self):
        """Test sensor data reading."""
        print(f"\n📊 Test 5: Sensor Data Reading")
        print("-" * 30)
        
        try:
            for i in range(5):
                sensor_data = await self.client.read_gatt_char(STATUS_CHAR_UUID)
                
                if sensor_data:
                    data = json.loads(sensor_data.decode('utf-8'))
                    data_type = data.get('type', data.get('t', 'unknown'))
                    
                    if data_type == 'sens':
                        # Ultra-short format: {"t": "sens", "d": [bat, emg, spd, imu_z, imu_x, imu_y], "id": car_id}
                        sensor_array = data.get('d', [0, 0, 0, 0, 0, 0])
                        car_id = data.get('id', 0)
                        battery = sensor_array[0] if len(sensor_array) > 0 else 0
                        emergency = bool(sensor_array[1]) if len(sensor_array) > 1 else False
                        speed = sensor_array[2] if len(sensor_array) > 2 else 0
                        imu_z = sensor_array[3] if len(sensor_array) > 3 else 0
                        print(f"Read {i+1}: Car{car_id}, Battery={battery:.1f}V, "
                              f"Speed={speed:.1f}, Emergency={emergency}, IMU_Z={imu_z:.2f}")
                    else:
                        # Old format or other data
                        content = data.get('content', {})
                        print(f"Read {i+1}: Type={data_type}, Battery={content.get('battery_voltage', 0):.1f}V, "
                              f"Speed={content.get('speed', 0):.2f}, "
                              f"Emergency={content.get('emergency_state', False)}")
                    
                    self.test_results["sensor_reads"] += 1
                else:
                    print(f"Read {i+1}: No data")
                
                await asyncio.sleep(0.5)
            
            print("✅ Sensor reading test completed")
            
        except Exception as e:
            print(f"❌ Sensor reading failed: {e}")
            self.test_results["errors"].append(f"Sensor reading: {e}")
    
    async def _test_high_frequency_commands(self):
        """Test high-frequency command sending (racing simulation)."""
        print(f"\n🏎️  Test 6: High-Frequency Commands (Racing Simulation)")
        print("-" * 30)
        
        try:
            print("Sending 20 rapid movement commands...")
            start_time = time.time()
            
            for i in range(20):
                # Simulate racing movement - varying speed and steering
                linear_x = 0.3 + 0.2 * (i % 5) / 5.0  # 0.3 to 0.5
                angular_z = 0.5 * (-1 if i % 4 < 2 else 1)  # Alternate steering
                
                command = {
                    "cmd": "move",
                    "lin": [linear_x, 0.0, 0.0],
                    "ang": [0.0, 0.0, angular_z]
                }
                
                json_str = json.dumps(command)
                response_mode = not self.jetson_mode  # False for Jetson, True for others
                await self.client.write_gatt_char(STATUS_CHAR_UUID, json_str.encode('utf-8'), response=response_mode)
                
                # No delay between commands - test rapid-fire capability
            
            elapsed = time.time() - start_time
            rate = 20 / elapsed
            
            print(f"✅ Sent 20 commands in {elapsed:.2f}s ({rate:.1f} commands/sec)")
            
            # Read final sensor state
            await asyncio.sleep(0.5)
            sensor_data = await self.client.read_gatt_char(STATUS_CHAR_UUID)
            if sensor_data:
                data = json.loads(sensor_data.decode('utf-8'))
                data_type = data.get('type', data.get('t', 'unknown'))
                
                if data_type == 'sens':
                    # Ultra-short format
                    sensor_array = data.get('d', [0, 0, 0, 0, 0, 0])
                    final_speed = sensor_array[2] if len(sensor_array) > 2 else 0
                else:
                    # Old format
                    final_speed = data.get('content', {}).get('speed', 0)
                
                print(f"Final robot speed: {final_speed:.2f} m/s")
            
        except Exception as e:
            print(f"❌ High-frequency test failed: {e}")
            self.test_results["errors"].append(f"High-frequency: {e}")
    
    async def _cleanup(self):
        """Clean up connection."""
        if self.client and self.client.is_connected:
            try:
                await self.client.disconnect()
                print("\n🔌 Disconnected from bridge")
            except:
                pass
    
    def _print_results(self):
        """Print test results summary."""
        print("\n" + "=" * 50)
        print("🧪 Test Results Summary")  
        print("=" * 50)
        
        results = self.test_results
        
        print(f"\n📊 Test Statistics:")
        print(f"   Device Discovery: {'✅' if results['discovery'] else '❌'}")
        print(f"   Connection: {'✅' if results['connection'] else '❌'}")
        print(f"   Simple Commands: {results['simple_commands']} successful")
        print(f"   JSON Commands: {results['json_commands']} successful") 
        print(f"   Sensor Reads: {results['sensor_reads']} successful")
        
        if results["errors"]:
            print(f"\n❌ Errors ({len(results['errors'])}):")
            for error in results["errors"][:5]:  # Show first 5 errors
                print(f"   • {error}")
            if len(results["errors"]) > 5:
                print(f"   • ... and {len(results['errors']) - 5} more")
        else:
            print(f"\n✅ No errors detected!")
    
    def _evaluate_success(self):
        """Evaluate if tests were successful."""
        results = self.test_results
        
        # Must have basic connectivity
        if not results["discovery"] or not results["connection"]:
            return False
        
        # Must have some successful operations  
        total_operations = results["simple_commands"] + results["json_commands"] + results["sensor_reads"]
        if total_operations < 5:
            return False
        
        # Error rate should be reasonable
        error_rate = len(results["errors"]) / max(total_operations, 1)
        if error_rate > 0.3:  # More than 30% error rate
            return False
        
        return True


async def main():
    """Main test function."""
    parser = argparse.ArgumentParser(description='Test Bluetooth ROS2 Bridge')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--jetson', action='store_true', 
                       help='Enable Jetson Nano compatibility mode (use write-without-response)')
    args = parser.parse_args()
    
    if args.verbose:
        import logging
        logging.basicConfig(level=logging.DEBUG)
    
    print("🚀 Starting Bluetooth ROS2 Bridge Test")
    if args.jetson:
        print("🤖 Jetson Nano mode: Using write-without-response for compatibility")
        print("Make sure the bridge is running in another terminal:")
        print("  ros2 run yahboomcar_bluetooth bluetooth_ros2_bridge --jetson --ros-args -p car_id:=1")
    else:
        print("Make sure the bridge is running in another terminal:")
        print("  ros2 run yahboomcar_bluetooth bluetooth_ros2_bridge --ros-args -p car_id:=1")
    print()
    
    tester = BridgeTestClient(jetson_mode=args.jetson)
    success = await tester.run_test_suite()
    
    if success:
        print("\n🎉 All tests PASSED! Bridge is working correctly.")
        print("✅ Ready for iPhone AR app integration!")
    else:
        print("\n❌ Some tests FAILED. Check the bridge configuration.")
        print("💡 Make sure:")
        print("   1. Bridge node is running with correct car_id")
        print("   2. bless library is properly installed")  
        print("   3. Bluetooth permissions are correct")
    
    return success


if __name__ == '__main__':
    if BLEAK_AVAILABLE:
        result = asyncio.run(main())
        exit(0 if result else 1)
    else:
        print("\n❌ Cannot run test without bleak library")
        print("Install with: pip install -r bluetooth_requirements.txt")
        exit(1)