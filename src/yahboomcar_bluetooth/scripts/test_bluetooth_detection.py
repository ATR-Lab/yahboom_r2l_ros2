#!/usr/bin/env python3

"""
Standalone BLE Detection Test Script for YahboomRacer Bluetooth Server

This script tests whether the YahboomRacer BLE server is properly advertising
and accessible from client devices. It performs comprehensive BLE scanning,
service discovery, and connection testing.

Usage:
    python3 test_bluetooth_detection.py

Requirements:
    pip3 install bleak

Author: Yahboom R2L Multiplayer Racing Team
License: MIT
"""

import asyncio
import json
import time
from typing import List, Dict, Any, Optional

try:
    from bleak import BleakScanner, BleakClient
    from bleak.backends.device import BLEDevice
    from bleak.backends.scanner import AdvertisementData
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False
    print("❌ bleak library not available. Install with: pip3 install bleak")

# YahboomRacer BLE Configuration (from bluetooth_bridge_node.py)
RACING_SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
COMMAND_CHAR_UUID = "87654321-4321-4321-4321-cba987654321"
SENSOR_CHAR_UUID = "11111111-2222-3333-4444-555555555555"
DEVICE_NAME_PREFIX = "YahboomRacer"
EXPECTED_MAC = "14:4F:8A:CA:F2:37"  # From your system


class BLEDetectionTester:
    """Comprehensive BLE detection and testing tool."""
    
    def __init__(self):
        self.found_devices: List[BLEDevice] = []
        self.yahboom_devices: List[BLEDevice] = []
        self.test_results: Dict[str, Any] = {
            'scan_results': {},
            'service_discovery': {},
            'connection_tests': {},
            'summary': {}
        }
    
    async def run_full_test(self):
        """Run comprehensive BLE detection test."""
        print("🔵 YahboomRacer BLE Detection Test")
        print("=" * 50)
        
        if not BLEAK_AVAILABLE:
            print("❌ Cannot run test - bleak library required")
            return
        
        # Phase 1: Device Discovery
        await self._scan_for_devices()
        
        # Phase 2: Service Discovery
        await self._discover_services()
        
        # Phase 3: Connection Testing
        await self._test_connections()
        
        # Phase 4: Results Summary
        self._print_summary()
        
        return self.test_results
    
    async def _scan_for_devices(self):
        """Phase 1: Scan for BLE devices."""
        print("\n📡 Phase 1: BLE Device Scanning")
        print("-" * 30)
        
        try:
            # Scan for 15 seconds
            print("🔍 Scanning for BLE devices (15 seconds)...")
            devices = await BleakScanner.discover(timeout=15.0, return_adv=True)
            
            print(f"✅ Found {len(devices)} BLE devices total")
            
            # Process all discovered devices
            for device, adv_data in devices.items():
                self.found_devices.append(device)
                
                # Check if this is a YahboomRacer device
                if self._is_yahboom_device(device, adv_data):
                    self.yahboom_devices.append(device)
                    print(f"🎯 FOUND YahboomRacer device: {device.name} ({device.address})")
                
                # Log all devices for debugging
                self._log_device(device, adv_data)
            
            # Store scan results
            self.test_results['scan_results'] = {
                'total_devices': len(devices),
                'yahboom_devices_found': len(self.yahboom_devices),
                'all_devices': [
                    {
                        'name': d.name or "Unknown",
                        'address': d.address,
                        'rssi': d.rssi
                    } for d in self.found_devices
                ]
            }
            
        except Exception as e:
            print(f"❌ Scan failed: {e}")
            self.test_results['scan_results']['error'] = str(e)
    
    def _is_yahboom_device(self, device: BLEDevice, adv_data: AdvertisementData) -> bool:
        """Check if device matches YahboomRacer criteria."""
        
        # Check device name
        if device.name and DEVICE_NAME_PREFIX in device.name:
            return True
        
        # Check service UUIDs
        if adv_data.service_uuids:
            for uuid in adv_data.service_uuids:
                if uuid.lower() == RACING_SERVICE_UUID.lower():
                    return True
        
        # Check MAC address (if advertising from same adapter)
        if device.address.upper() == EXPECTED_MAC.upper():
            return True
        
        return False
    
    def _log_device(self, device: BLEDevice, adv_data: AdvertisementData):
        """Log device details for debugging."""
        print(f"  📱 {device.name or 'Unknown'}")
        print(f"     Address: {device.address}")
        print(f"     RSSI: {device.rssi} dBm")
        
        if adv_data.service_uuids:
            print(f"     Services: {list(adv_data.service_uuids)}")
        
        if adv_data.local_name:
            print(f"     Local Name: {adv_data.local_name}")
        
        print()
    
    async def _discover_services(self):
        """Phase 2: Discover services on found YahboomRacer devices."""
        print("\n🔍 Phase 2: Service Discovery")
        print("-" * 30)
        
        if not self.yahboom_devices:
            print("⚠️  No YahboomRacer devices found - skipping service discovery")
            return
        
        for device in self.yahboom_devices:
            print(f"🔗 Connecting to {device.name} ({device.address})")
            
            try:
                async with BleakClient(device) as client:
                    print(f"✅ Connected to {device.name}")
                    
                    # Get all services
                    services = client.services
                    print(f"📋 Found {len(services)} services:")
                    
                    device_services = []
                    found_racing_service = False
                    
                    for service in services:
                        service_info = {
                            'uuid': service.uuid,
                            'description': service.description,
                            'characteristics': []
                        }
                        
                        print(f"  🔧 Service: {service.uuid}")
                        if service.description:
                            print(f"      Description: {service.description}")
                        
                        # Check if this is our racing service
                        if service.uuid.lower() == RACING_SERVICE_UUID.lower():
                            found_racing_service = True
                            print("      🎯 THIS IS THE RACING SERVICE!")
                        
                        # Get characteristics
                        for char in service.characteristics:
                            char_info = {
                                'uuid': char.uuid,
                                'properties': char.properties,
                                'description': char.description
                            }
                            service_info['characteristics'].append(char_info)
                            
                            print(f"      📝 Characteristic: {char.uuid}")
                            print(f"          Properties: {char.properties}")
                            
                            # Check for our specific characteristics
                            if char.uuid.lower() == COMMAND_CHAR_UUID.lower():
                                print("          🎯 THIS IS THE COMMAND CHARACTERISTIC!")
                            elif char.uuid.lower() == SENSOR_CHAR_UUID.lower():
                                print("          🎯 THIS IS THE SENSOR CHARACTERISTIC!")
                        
                        device_services.append(service_info)
                        print()
                    
                    # Store service discovery results
                    self.test_results['service_discovery'][device.address] = {
                        'connected': True,
                        'racing_service_found': found_racing_service,
                        'services': device_services
                    }
                    
            except Exception as e:
                print(f"❌ Failed to connect to {device.name}: {e}")
                self.test_results['service_discovery'][device.address] = {
                    'connected': False,
                    'error': str(e)
                }
    
    async def _test_connections(self):
        """Phase 3: Test actual communication with YahboomRacer devices."""
        print("\n🔗 Phase 3: Communication Testing")
        print("-" * 30)
        
        if not self.yahboom_devices:
            print("⚠️  No YahboomRacer devices found - skipping communication tests")
            return
        
        for device in self.yahboom_devices:
            print(f"📡 Testing communication with {device.name}")
            
            try:
                async with BleakClient(device) as client:
                    # Test sensor characteristic read
                    await self._test_sensor_read(client, device)
                    
                    # Test command characteristic write
                    await self._test_command_write(client, device)
                    
            except Exception as e:
                print(f"❌ Communication test failed for {device.name}: {e}")
                self.test_results['connection_tests'][device.address] = {
                    'error': str(e)
                }
    
    async def _test_sensor_read(self, client: BleakClient, device: BLEDevice):
        """Test reading from sensor characteristic."""
        try:
            print("  📊 Testing sensor data read...")
            data = await client.read_gatt_char(SENSOR_CHAR_UUID)
            sensor_json = data.decode('utf-8')
            sensor_data = json.loads(sensor_json)
            
            print(f"  ✅ Sensor read successful: {sensor_data}")
            
            if device.address not in self.test_results['connection_tests']:
                self.test_results['connection_tests'][device.address] = {}
            self.test_results['connection_tests'][device.address]['sensor_read'] = {
                'success': True,
                'data': sensor_data
            }
            
        except Exception as e:
            print(f"  ❌ Sensor read failed: {e}")
            if device.address not in self.test_results['connection_tests']:
                self.test_results['connection_tests'][device.address] = {}
            self.test_results['connection_tests'][device.address]['sensor_read'] = {
                'success': False,
                'error': str(e)
            }
    
    async def _test_command_write(self, client: BleakClient, device: BLEDevice):
        """Test writing to command characteristic."""
        try:
            print("  📝 Testing command write...")
            
            # Create test command
            test_command = {
                "msg_type": "robot_command",
                "data": {
                    "movement": {
                        "linear": {"x": 0.1, "y": 0.0},
                        "angular": {"z": 0.0}
                    },
                    "game_effects": {
                        "power_up": None
                    }
                }
            }
            
            command_json = json.dumps(test_command)
            command_data = command_json.encode('utf-8')
            
            await client.write_gatt_char(COMMAND_CHAR_UUID, command_data, response=False)
            print(f"  ✅ Command write successful: {test_command}")
            
            if device.address not in self.test_results['connection_tests']:
                self.test_results['connection_tests'][device.address] = {}
            self.test_results['connection_tests'][device.address]['command_write'] = {
                'success': True,
                'command_sent': test_command
            }
            
        except Exception as e:
            print(f"  ❌ Command write failed: {e}")
            if device.address not in self.test_results['connection_tests']:
                self.test_results['connection_tests'][device.address] = {}
            self.test_results['connection_tests'][device.address]['command_write'] = {
                'success': False,
                'error': str(e)
            }
    
    def _print_summary(self):
        """Print test results summary."""
        print("\n📋 Test Results Summary")
        print("=" * 50)
        
        # Device discovery summary
        total_devices = self.test_results['scan_results'].get('total_devices', 0)
        yahboom_found = self.test_results['scan_results'].get('yahboom_devices_found', 0)
        
        print(f"📡 BLE Scan Results:")
        print(f"   Total devices found: {total_devices}")
        print(f"   YahboomRacer devices: {yahboom_found}")
        
        if yahboom_found == 0:
            print("   ❌ NO YAHBOOM DEVICES DETECTED")
            print("   🔍 Possible issues:")
            print("      - Server not actually advertising")
            print("      - Different device name than expected")
            print("      - BLE advertisement not visible externally")
            print("      - Scanning from wrong location/permissions")
        else:
            print(f"   ✅ YahboomRacer devices detected successfully!")
        
        # Service discovery summary
        print(f"\n🔍 Service Discovery:")
        if not self.test_results['service_discovery']:
            print("   ⚠️  No service discovery performed")
        else:
            for addr, results in self.test_results['service_discovery'].items():
                if results['connected']:
                    racing_service = "✅" if results['racing_service_found'] else "❌"
                    print(f"   Device {addr}: Connected {racing_service} Racing Service")
                else:
                    print(f"   Device {addr}: ❌ Connection failed")
        
        # Communication testing summary
        print(f"\n🔗 Communication Tests:")
        if not self.test_results['connection_tests']:
            print("   ⚠️  No communication tests performed")
        else:
            for addr, results in self.test_results['connection_tests'].items():
                sensor_ok = results.get('sensor_read', {}).get('success', False)
                command_ok = results.get('command_write', {}).get('success', False)
                sensor_status = "✅" if sensor_ok else "❌"
                command_status = "✅" if command_ok else "❌"
                print(f"   Device {addr}: Sensor {sensor_status} Command {command_status}")
        
        print(f"\n🎯 Overall Status:")
        if yahboom_found > 0:
            print("   ✅ BLE server is advertising and detectable!")
            print("   💡 Your Bluetooth server appears to be working correctly.")
        else:
            print("   ❌ BLE server not detected")
            print("   💡 Check if the ROS2 node is actually running and advertising.")


async def main():
    """Main test function."""
    print("Starting YahboomRacer BLE Detection Test...")
    print("Make sure the bluetooth_bridge_node is running in another terminal!\n")
    
    tester = BLEDetectionTester()
    results = await tester.run_full_test()
    
    # Optionally save results to file
    try:
        with open('ble_test_results.json', 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\n💾 Detailed results saved to: ble_test_results.json")
    except Exception as e:
        print(f"⚠️  Could not save results file: {e}")


if __name__ == '__main__':
    if BLEAK_AVAILABLE:
        asyncio.run(main())
    else:
        print("\n❌ Cannot run test without bleak library")
        print("Install with: pip3 install bleak")
        print("Then run: python3 test_bluetooth_detection.py")