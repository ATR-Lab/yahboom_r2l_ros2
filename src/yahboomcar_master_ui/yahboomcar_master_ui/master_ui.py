#!/usr/bin/env python3
"""
Master UI Node - Main entry point for the robot racing master control center.
Integrates ROS2 with PyQT for comprehensive monitoring and control.
"""

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from .main_window import MasterControlWindow
from .ros_data_manager import RosDataManager


class MasterUINode(Node):
    """ROS2 node that manages the master control UI application."""
    
    def __init__(self):
        super().__init__('master_ui_node')
        self.get_logger().info('Starting Master Control Center UI...')
        
        # Initialize ROS data manager
        self.data_manager = RosDataManager(self)
        
        # Initialize PyQT application
        self.app = QApplication(sys.argv)
        self.main_window = MasterControlWindow(self.data_manager)
        
        # Set up timer for ROS spinning (integrate with PyQT event loop)
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_once)
        self.timer.start(10)  # 100Hz update rate
        
        self.get_logger().info('Master Control Center UI initialized successfully')
    
    def spin_once(self):
        """Process ROS callbacks - called by PyQT timer."""
        rclpy.spin_once(self, timeout_sec=0)
    
    def run(self):
        """Start the application."""
        self.main_window.show()
        return self.app.exec_()


def main(args=None):
    """Main entry point for the master UI application."""
    rclpy.init(args=args)
    
    try:
        master_ui = MasterUINode()
        exit_code = master_ui.run()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()