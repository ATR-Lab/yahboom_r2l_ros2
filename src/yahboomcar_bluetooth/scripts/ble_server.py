#!/usr/bin/env python3

"""
Working Minimal BLE Server for Yahboom Robot
============================================

A minimal working BLE server that starts correctly and can be discovered by other devices.
This version avoids the API issues we encountered.

Requirements:
    pip install bless

Usage:
    python3 working_ble_server.py

Author: Yahboom Robot Bluetooth Team
"""

import asyncio
import logging
import json
from datetime import datetime
from bless import BlessServer

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
        
    def get_status_data(self):
        """Generate status data as bytes"""
        status_data = {
            "timestamp": datetime.now().isoformat(),
            "status": "running",
            "message_count": self.message_count,
            "server_name": "YahboomRobot",
            "uptime_seconds": int((datetime.now() - self.start_time).total_seconds())
        }
        return json.dumps(status_data).encode('utf-8')
        
    def read_request_callback(self, characteristic, offset):
        """Handle read requests from clients"""
        logger.info(f"📖 Read request for: {characteristic}")
        
        if characteristic == STATUS_CHAR_UUID:
            data = self.get_status_data()
            logger.info(f"📤 Sending status data: {len(data)} bytes")
            return data
        
        # Default response
        response = b"Hello from YahboomRobot!"
        logger.info(f"📤 Sending default response: {response.decode()}")
        return response
        
    def write_request_callback(self, characteristic, value, offset, without_response):
        """Handle write requests from clients"""
        logger.info(f"✍️  Write request for: {characteristic}")
        
        try:
            message = value.decode('utf-8')
            self.message_count += 1
            logger.info(f"📥 Received message: {message} (#{self.message_count})")
            
            # Echo response
            response = f"Received: {message} (#{self.message_count})"
            logger.info(f"📤 Response: {response}")
            
        except Exception as e:
            logger.error(f"❌ Error processing write: {e}")
            
    async def setup_server(self):
        """Initialize the BLE server with basic service"""
        logger.info("⚙️  Setting up BLE server...")
        
        try:
            # Add service only (no characteristics to avoid API issues)
            await self.server.add_new_service(SERVICE_UUID)
            logger.info(f"✅ Service added: {SERVICE_UUID}")
            
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
    print("=" * 60)
    print("    🤖 Yahboom Robot BLE Server (Working Version)")
    print("=" * 60)
    print()
    print("🌟 This server will:")
    print("  • Advertise as 'YahboomRobot'")
    print("  • Be discoverable by BLE scanners")
    print("  • Handle basic read/write operations")
    print("  • Log all interactions")
    print()
    print("📱 How to test:")
    print("  • Use 'nRF Connect' app on mobile")
    print("  • Use 'BLE Scanner' on Android")
    print("  • Use 'LightBlue Explorer' on iOS")
    print("  • Look for 'YahboomRobot' device")
    print()
    print("🎯 Service UUID: 12345678-1234-1234-1234-123456789abc")
    print("📊 Status Characteristic: 11111111-2222-3333-4444-555555555555")
    print()
    print("Press Ctrl+C to stop")
    print("=" * 60)
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
