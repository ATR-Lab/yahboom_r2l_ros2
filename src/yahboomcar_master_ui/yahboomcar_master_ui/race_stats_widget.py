#!/usr/bin/env python3
"""
Race Statistics Widget - Performance metrics and analytics.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer


class RaceStatsWidget(QWidget):
    """Widget for displaying race statistics."""
    
    def __init__(self, data_manager):
        super().__init__()
        self.data_manager = data_manager
        self._init_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data)
        self.update_timer.start(5000)  # 5 second updates
    
    def _init_ui(self):
        """Initialize the race stats UI."""
        self.setStyleSheet("""
            QWidget {
                background-color: #2b2b2b;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 8px;
            }
        """)
        
        layout = QVBoxLayout(self)
        
        title_label = QLabel("📊 RACE STATISTICS")
        title_label.setStyleSheet("font-weight: bold; font-size: 14px; margin-bottom: 5px;")
        layout.addWidget(title_label)
        
        # Performance metrics
        self.avg_speed_label = QLabel("📈 Avg Speed: 2.1 m/s")
        layout.addWidget(self.avg_speed_label)
        
        self.avg_battery_label = QLabel("🔋 Avg Battery: 83%")
        layout.addWidget(self.avg_battery_label)
        
        # Safety events - now prominently displayed
        safety_label = QLabel("🚨 SAFETY EVENTS")
        safety_label.setStyleSheet("font-weight: bold; font-size: 13px; margin-top: 10px; color: #ff9800;")
        layout.addWidget(safety_label)
        
        self.emergency_stops_label = QLabel("🚨 Emergency Stops: 0")
        self.emergency_stops_label.setStyleSheet("font-size: 11px;")
        layout.addWidget(self.emergency_stops_label)
        
        self.warnings_label = QLabel("⚠️ Warnings: 0") 
        self.warnings_label.setStyleSheet("font-size: 11px;")
        layout.addWidget(self.warnings_label)
        
        layout.addStretch()
    
    def _update_data(self):
        """Update race statistics."""
        # Calculate stats from car data
        cars = self.data_manager.get_all_cars()
        if cars:
            total_speed = sum(car.speed_linear for car in cars.values())
            avg_speed = total_speed / len(cars)
            
            total_battery = sum(car.battery_percentage for car in cars.values())
            avg_battery = total_battery / len(cars)
            
            emergency_count = sum(1 for car in cars.values() if car.emergency_stopped)
            
            self.avg_speed_label.setText(f"📈 Avg Speed: {avg_speed:.1f} m/s")
            self.avg_battery_label.setText(f"🔋 Avg Battery: {avg_battery:.0f}%")
            self.emergency_stops_label.setText(f"🚨 Emergency Stops: {emergency_count}")