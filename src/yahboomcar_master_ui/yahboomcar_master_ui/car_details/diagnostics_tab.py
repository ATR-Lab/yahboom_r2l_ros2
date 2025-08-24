#!/usr/bin/env python3
"""
Diagnostics Tab - Expanded sensor data and performance metrics for individual cars.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QProgressBar, QFrame, QGridLayout, QGroupBox, QTextEdit
)
from PyQt5.QtCore import Qt


class DiagnosticsTab(QWidget):
    """Tab widget for car diagnostics and sensor data."""
    
    def __init__(self, car_id, data_manager):
        super().__init__()
        self.car_id = car_id
        self.data_manager = data_manager
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the diagnostics tab UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # Top row: Battery and Performance
        top_layout = QHBoxLayout()
        
        # Battery Details Group
        battery_group = self._create_battery_group()
        top_layout.addWidget(battery_group)
        
        # Performance Metrics Group  
        performance_group = self._create_performance_group()
        top_layout.addWidget(performance_group)
        
        layout.addLayout(top_layout)
        
        # Middle row: Sensor Data
        sensor_group = self._create_sensor_group()
        layout.addWidget(sensor_group)
        
        # Bottom row: System Health and Logs
        bottom_layout = QHBoxLayout()
        
        health_group = self._create_health_group()
        bottom_layout.addWidget(health_group)
        
        logs_group = self._create_logs_group()
        bottom_layout.addWidget(logs_group)
        
        layout.addLayout(bottom_layout)
    
    def _create_battery_group(self):
        """Create battery details group."""
        group = QGroupBox("🔋 Battery Details")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        # Voltage and percentage
        self.voltage_label = QLabel("Voltage: 12.3V")
        self.voltage_label.setStyleSheet("font-size: 14px;")
        layout.addWidget(self.voltage_label)
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setFixedHeight(20)
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #555;
                border-radius: 3px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #4caf50;
                border-radius: 2px;
            }
        """)
        layout.addWidget(self.battery_bar)
        
        # Additional battery metrics
        self.current_label = QLabel("Current Draw: 2.1A")
        self.temperature_label = QLabel("Temperature: 28°C")
        self.runtime_label = QLabel("Est. Runtime: 45 min")
        
        for label in [self.current_label, self.temperature_label, self.runtime_label]:
            label.setStyleSheet("color: #ccc; font-size: 11px;")
            layout.addWidget(label)
        
        return group
    
    def _create_performance_group(self):
        """Create performance metrics group."""
        group = QGroupBox("🏎️ Performance Metrics")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        # Current speed and position
        self.speed_label = QLabel("Speed: 1.2 m/s")
        self.position_label = QLabel("Position: (2.5, -1.1)")
        self.heading_label = QLabel("Heading: 45°")
        
        # Performance metrics
        self.uptime_label = QLabel("Uptime: 00:15:32")
        self.commands_label = QLabel("Commands/sec: 12")
        self.latency_label = QLabel("Avg Latency: 15ms")
        
        for label in [self.speed_label, self.position_label, self.heading_label,
                     self.uptime_label, self.commands_label, self.latency_label]:
            label.setStyleSheet("font-size: 12px; margin: 2px;")
            layout.addWidget(label)
        
        return group
    
    def _create_sensor_group(self):
        """Create sensor data group."""
        group = QGroupBox("📡 Sensor Data")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        
        layout = QGridLayout(group)
        
        # IMU Data
        imu_frame = QFrame()
        imu_frame.setStyleSheet("border: 1px solid #444; border-radius: 3px; padding: 5px;")
        imu_layout = QVBoxLayout(imu_frame)
        
        imu_title = QLabel("🧭 IMU (Accelerometer/Gyroscope)")
        imu_title.setStyleSheet("font-weight: bold; font-size: 11px;")
        imu_layout.addWidget(imu_title)
        
        self.imu_accel_label = QLabel("Accel: (0.1, 0.0, 9.8) m/s²")
        self.imu_gyro_label = QLabel("Gyro: (0.0, 0.0, 0.2) rad/s")
        
        for label in [self.imu_accel_label, self.imu_gyro_label]:
            label.setStyleSheet("color: #ccc; font-size: 10px; font-family: monospace;")
            imu_layout.addWidget(label)
        
        layout.addWidget(imu_frame, 0, 0)
        
        # Magnetometer
        mag_frame = QFrame()
        mag_frame.setStyleSheet("border: 1px solid #444; border-radius: 3px; padding: 5px;")
        mag_layout = QVBoxLayout(mag_frame)
        
        mag_title = QLabel("🧲 Magnetometer")
        mag_title.setStyleSheet("font-weight: bold; font-size: 11px;")
        mag_layout.addWidget(mag_title)
        
        self.mag_field_label = QLabel("Field: (0.2, -0.1, 0.5) mT")
        self.mag_heading_label = QLabel("Magnetic Heading: 47°")
        
        for label in [self.mag_field_label, self.mag_heading_label]:
            label.setStyleSheet("color: #ccc; font-size: 10px; font-family: monospace;")
            mag_layout.addWidget(label)
        
        layout.addWidget(mag_frame, 0, 1)
        
        # Joint States
        joints_frame = QFrame()
        joints_frame.setStyleSheet("border: 1px solid #444; border-radius: 3px; padding: 5px;")
        joints_layout = QVBoxLayout(joints_frame)
        
        joints_title = QLabel("⚙️ Motor States")
        joints_title.setStyleSheet("font-weight: bold; font-size: 11px;")
        joints_layout.addWidget(joints_title)
        
        self.left_motor_label = QLabel("Left Motor: 45% | 1200 RPM")
        self.right_motor_label = QLabel("Right Motor: 42% | 1150 RPM")
        
        for label in [self.left_motor_label, self.right_motor_label]:
            label.setStyleSheet("color: #ccc; font-size: 10px; font-family: monospace;")
            joints_layout.addWidget(label)
        
        layout.addWidget(joints_frame, 0, 2)
        
        return group
    
    def _create_health_group(self):
        """Create system health group."""
        group = QGroupBox("🏥 System Health")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        self.health_status = QLabel("Status: ✅ All Systems Operational")
        self.health_status.setStyleSheet("font-size: 12px; color: #4caf50;")
        layout.addWidget(self.health_status)
        
        # Health metrics
        self.cpu_usage = QLabel("CPU Usage: 15%")
        self.memory_usage = QLabel("Memory: 234 MB")
        self.temperature = QLabel("System Temp: 42°C")
        
        for label in [self.cpu_usage, self.memory_usage, self.temperature]:
            label.setStyleSheet("color: #ccc; font-size: 11px;")
            layout.addWidget(label)
        
        # Run diagnostics button
        diag_btn = QPushButton("🔍 Run Full Diagnostics")
        diag_btn.setStyleSheet("""
            QPushButton {
                background-color: #0066cc;
                color: white;
                padding: 8px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #0080ff;
            }
            QPushButton:pressed {
                background-color: #004499;
            }
        """)
        diag_btn.clicked.connect(self._run_diagnostics)
        layout.addWidget(diag_btn)
        
        return group
    
    def _create_logs_group(self):
        """Create car-specific logs group."""
        group = QGroupBox("📋 Car Logs")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        self.log_display = QTextEdit()
        self.log_display.setFixedHeight(120)
        self.log_display.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                border: 1px solid #444;
                color: #ffffff;
                font-family: 'Courier New', monospace;
                font-size: 9px;
            }
        """)
        self.log_display.setReadOnly(True)
        layout.addWidget(self.log_display)
        
        return group
    
    def _run_diagnostics(self):
        """Trigger comprehensive diagnostics."""
        self.data_manager.call_car_diagnostics(self.car_id)
        self.data_manager.log_user_action(f"Full diagnostics requested", f"Car #{self.car_id}")
    
    def update_data(self):
        """Update diagnostics tab with latest data."""
        car_data = self.data_manager.get_car_data(self.car_id)
        if not car_data:
            return
        
        # Update battery details
        self.voltage_label.setText(f"Voltage: {car_data.battery_voltage:.1f}V")
        self.battery_bar.setValue(car_data.battery_percentage)
        
        # Update performance metrics
        self.speed_label.setText(f"Speed: {car_data.speed_linear:.1f} m/s")
        self.position_label.setText(f"Position: ({car_data.position_x:.1f}, {car_data.position_y:.1f})")
        self.heading_label.setText(f"Heading: {car_data.heading:.0f}°")
        
        # Update health status
        if car_data.emergency_stopped:
            self.health_status.setText("Status: 🚨 Emergency Stopped")
            self.health_status.setStyleSheet("font-size: 12px; color: #f44336;")
        elif car_data.system_health == "ERROR":
            self.health_status.setText("Status: ⚠️ System Errors Detected")
            self.health_status.setStyleSheet("font-size: 12px; color: #ff9800;")
        else:
            self.health_status.setText("Status: ✅ All Systems Operational")
            self.health_status.setStyleSheet("font-size: 12px; color: #4caf50;")
        
        # Update sensor data (placeholder - would be real IMU/sensor data)
        if car_data.imu_data:
            # Would extract real IMU data here
            pass
        
        # Update car-specific logs
        # TODO: Implement car-specific log filtering from event log