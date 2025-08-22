#!/usr/bin/env python3
"""
System Status Widget - Overall system health monitoring.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QProgressBar, QHBoxLayout
from PyQt5.QtCore import QTimer
import time


class SystemStatusWidget(QWidget):
    """Widget for displaying overall system status."""
    
    def __init__(self, data_manager):
        super().__init__()
        self.data_manager = data_manager
        self.setMinimumHeight(110)
        self.setMaximumHeight(140)
        self._init_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data)
        self.update_timer.start(1000)  # 1Hz updates
    
    def _init_ui(self):
        """Initialize the system status UI."""
        self.setStyleSheet("""
            QWidget {
                background-color: #2b2b2b;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 5px;
            }
        """)
        
        layout = QVBoxLayout(self)
        
        # Cars online status
        self.cars_status_label = QLabel("Cars Online: 0/4")
        self.cars_status_label.setStyleSheet("font-weight: bold; color: white;")
        layout.addWidget(self.cars_status_label)
        
        # System uptime
        self.uptime_label = QLabel("Uptime: 00:00:00")
        layout.addWidget(self.uptime_label)
        
        # Network status
        self.network_label = QLabel("Network: 🟢 Connected")
        layout.addWidget(self.network_label)
        
        # Joystick status
        self.joystick_label = QLabel("Joystick: 🔴 Disconnected")
        layout.addWidget(self.joystick_label)
        
        # Add stretch to use available space efficiently
        layout.addStretch()
    
    def _update_data(self):
        """Update system status display."""
        # Count online cars
        online_count = sum(1 for car in self.data_manager.get_all_cars().values() 
                          if car.ros2_connected)
        self.cars_status_label.setText(f"Cars Online: {online_count}/4")
        
        # Update uptime
        uptime_seconds = int(self.data_manager.get_system_uptime())
        hours = uptime_seconds // 3600
        minutes = (uptime_seconds % 3600) // 60
        seconds = uptime_seconds % 60
        self.uptime_label.setText(f"Uptime: {hours:02d}:{minutes:02d}:{seconds:02d}")
        
        # Update joystick status
        if self.data_manager.is_joystick_connected():
            self.joystick_label.setText("Joystick: 🟢 Connected")
        else:
            self.joystick_label.setText("Joystick: 🔴 Disconnected")