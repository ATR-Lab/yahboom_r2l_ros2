#!/usr/bin/env python3
"""
Event Log Widget - Real-time system event logging.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTextEdit, QHBoxLayout, QPushButton
from PyQt5.QtCore import QTimer
import time


class EventLogWidget(QWidget):
    """Widget for displaying real-time event logs."""
    
    def __init__(self, data_manager):
        super().__init__()
        self.data_manager = data_manager
        self.log_entries = []
        self._init_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_log)
        self.update_timer.start(1000)  # 1Hz updates
        
        # Add initial log entry
        self._add_log_entry("INFO", "Master Control Center started")
    
    def _init_ui(self):
        """Initialize the event log UI."""
        self.setStyleSheet("""
            QWidget {
                background-color: #2b2b2b;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 8px;
            }
        """)
        
        layout = QVBoxLayout(self)
        
        # Header with controls
        header_layout = QHBoxLayout()
        
        title_label = QLabel("📜 REAL-TIME EVENT LOG")
        title_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        header_layout.addWidget(title_label)
        
        header_layout.addStretch()
        
        # Clear button
        clear_btn = QPushButton("🧹 CLEAR")
        clear_btn.setFixedSize(60, 25)
        clear_btn.clicked.connect(self._clear_log)
        header_layout.addWidget(clear_btn)
        
        layout.addLayout(header_layout)
        
        # Log display
        self.log_display = QTextEdit()
        self.log_display.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                border: 1px solid #444;
                color: #ffffff;
                font-family: 'Courier New', monospace;
                font-size: 10px;
            }
        """)
        self.log_display.setReadOnly(True)
        layout.addWidget(self.log_display)
    
    def _add_log_entry(self, level, message):
        """Add a new log entry."""
        timestamp = time.strftime("%H:%M:%S")
        entry = f"[{timestamp}] {level:4} {message}"
        
        self.log_entries.append(entry)
        
        # Keep only last 100 entries
        if len(self.log_entries) > 100:
            self.log_entries.pop(0)
        
        # Update display
        self._refresh_display()
    
    def _refresh_display(self):
        """Refresh the log display."""
        self.log_display.clear()
        
        for entry in self.log_entries:
            # Color code by log level
            if "ERROR" in entry:
                color = "#ff5555"
            elif "WARN" in entry:
                color = "#ffaa00"
            elif "INFO" in entry:
                color = "#50fa7b"
            else:
                color = "#ffffff"
            
            self.log_display.append(f'<span style="color: {color};">{entry}</span>')
        
        # Scroll to bottom
        self.log_display.moveCursor(self.log_display.textCursor().End)
    
    def _clear_log(self):
        """Clear the log display."""
        self.log_entries.clear()
        self.log_display.clear()
        self._add_log_entry("INFO", "Log cleared")
    
    def _update_log(self):
        """Check for new events to log."""
        # Monitor car status changes and log events
        current_time = time.time()
        
        for car_id, car_data in self.data_manager.get_all_cars().items():
            # Check for connection issues
            if current_time - car_data.last_update_time > 5.0 and car_data.ros2_connected:
                car_data.ros2_connected = False
                self._add_log_entry("WARN", f"Car #{car_id} connection lost")
            
            # Check for low battery
            if car_data.battery_percentage < 20 and car_data.battery_percentage > 0:
                # Only log once when it goes below 20%
                pass  # Would need state tracking to avoid spam
            
            # Check for emergency stops
            if car_data.emergency_stopped:
                # Would need state tracking to log only state changes
                pass