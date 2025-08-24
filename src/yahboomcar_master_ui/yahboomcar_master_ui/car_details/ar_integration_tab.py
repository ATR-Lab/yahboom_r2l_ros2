#!/usr/bin/env python3
"""
AR Integration Tab - Bluetooth and AR app connectivity management.
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QProgressBar, QFrame, QGroupBox, QTextEdit, QListWidget, QCheckBox, QFormLayout
)
from PyQt5.QtCore import Qt


class ARIntegrationTab(QWidget):
    """Tab widget for AR app and Bluetooth integration."""
    
    def __init__(self, car_id, data_manager):
        super().__init__()
        self.car_id = car_id
        self.data_manager = data_manager
        self._init_ui()
    
    def _init_ui(self):
        """Initialize the AR integration tab UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Top row: Connection Status
        connection_group = self._create_connection_group()
        layout.addWidget(connection_group)
        
        # Middle row: AR App Communication
        communication_layout = QHBoxLayout()
        
        ar_status_group = self._create_ar_status_group()
        communication_layout.addWidget(ar_status_group)
        
        message_logs_group = self._create_message_logs_group()
        communication_layout.addWidget(message_logs_group)
        
        layout.addLayout(communication_layout)
        
        # Bottom row: Controls
        controls_group = self._create_controls_group()
        layout.addWidget(controls_group)
    
    def _create_connection_group(self):
        """Create Bluetooth connection status group."""
        group = QGroupBox("📡 Bluetooth Connection Status")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        # Connection info
        info_layout = QHBoxLayout()
        
        # Status indicator
        self.connection_status = QLabel("🟢 Connected to iPhone_AR_Driver")
        self.connection_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #4caf50;")
        info_layout.addWidget(self.connection_status)
        
        info_layout.addStretch()
        
        # Signal strength
        self.signal_strength = QLabel("Signal: -45 dBm")
        self.signal_strength.setStyleSheet("color: #ccc;")
        info_layout.addWidget(self.signal_strength)
        
        layout.addLayout(info_layout)
        
        # Connection details
        details_layout = QHBoxLayout()
        
        self.device_info = QLabel("Device: iPhone 14 Pro | iOS 17.1")
        self.device_info.setStyleSheet("color: #ccc; font-size: 11px;")
        details_layout.addWidget(self.device_info)
        
        details_layout.addStretch()
        
        self.connection_time = QLabel("Connected: 00:12:34")
        self.connection_time.setStyleSheet("color: #ccc; font-size: 11px;")
        details_layout.addWidget(self.connection_time)
        
        layout.addLayout(details_layout)
        
        return group
    
    def _create_ar_status_group(self):
        """Create AR app communication status."""
        group = QGroupBox("📱 AR App Communication")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        # Protocol info
        self.protocol_label = QLabel("Protocol: Mario Kart Live v2.1")
        self.protocol_label.setStyleSheet("font-size: 12px;")
        layout.addWidget(self.protocol_label)
        
        # Message statistics
        stats_layout = QFormLayout()
        
        self.messages_sent = QLabel("1,247")
        self.messages_received = QLabel("1,251") 
        self.error_rate = QLabel("0.3%")
        self.latency = QLabel("18ms avg")
        
        stats_layout.addRow("Messages Sent:", self.messages_sent)
        stats_layout.addRow("Messages Received:", self.messages_received)
        stats_layout.addRow("Error Rate:", self.error_rate)
        stats_layout.addRow("Avg Latency:", self.latency)
        
        for i in range(stats_layout.rowCount()):
            label = stats_layout.itemAt(i, QFormLayout.LabelRole).widget()
            if label:
                label.setStyleSheet("color: #ccc; font-size: 11px;")
            field = stats_layout.itemAt(i, QFormLayout.FieldRole).widget()
            if field:
                field.setStyleSheet("color: white; font-size: 11px; font-weight: bold;")
        
        layout.addLayout(stats_layout)
        
        return group
    
    def _create_message_logs_group(self):
        """Create message logs display."""
        group = QGroupBox("📨 Message Logs")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QVBoxLayout(group)
        
        # Message type filter
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("Show:"))
        
        # TODO: Add filter checkboxes for different message types
        self.show_commands = QCheckBox("Commands")
        self.show_feedback = QCheckBox("Feedback") 
        self.show_errors = QCheckBox("Errors")
        
        self.show_commands.setChecked(True)
        self.show_feedback.setChecked(True)
        self.show_errors.setChecked(True)
        
        for checkbox in [self.show_commands, self.show_feedback, self.show_errors]:
            checkbox.setStyleSheet("color: #ccc;")
            filter_layout.addWidget(checkbox)
        
        filter_layout.addStretch()
        layout.addLayout(filter_layout)
        
        # Message log display
        self.message_log = QTextEdit()
        self.message_log.setFixedHeight(150)
        self.message_log.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                border: 1px solid #444;
                color: #ffffff;
                font-family: 'Courier New', monospace;
                font-size: 9px;
            }
        """)
        self.message_log.setReadOnly(True)
        layout.addWidget(self.message_log)
        
        return group
    
    def _create_controls_group(self):
        """Create Bluetooth and AR control buttons."""
        group = QGroupBox("🔧 Connection Controls")
        group.setStyleSheet("QGroupBox { font-weight: bold; }")
        layout = QHBoxLayout(group)
        
        # Reconnect Bluetooth
        reconnect_btn = QPushButton("🔄 Reconnect Bluetooth")
        reconnect_btn.setStyleSheet("""
            QPushButton {
                background-color: #0066cc;
                color: white;
                padding: 10px 15px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #0080ff;
            }
            QPushButton:pressed {
                background-color: #004499;
            }
        """)
        reconnect_btn.clicked.connect(self._reconnect_bluetooth)
        layout.addWidget(reconnect_btn)
        
        # Re-pair device
        pair_btn = QPushButton("📱 Re-pair AR Device")
        pair_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff9800;
                color: white;
                padding: 10px 15px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #ffb74d;
            }
            QPushButton:pressed {
                background-color: #f57c00;
            }
        """)
        pair_btn.clicked.connect(self._repair_device)
        layout.addWidget(pair_btn)
        
        # Reset AR communication
        reset_comm_btn = QPushButton("🔁 Reset AR Communication")
        reset_comm_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                padding: 10px 15px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #ef5350;
            }
            QPushButton:pressed {
                background-color: #d32f2f;
            }
        """)
        reset_comm_btn.clicked.connect(self._reset_communication)
        layout.addWidget(reset_comm_btn)
        
        layout.addStretch()
        
        return group
    
    def _reconnect_bluetooth(self):
        """Attempt to reconnect Bluetooth."""
        self.data_manager.log_user_action(f"Bluetooth reconnect requested", f"Car #{self.car_id}")
        # TODO: Implement actual Bluetooth reconnection
        
    def _repair_device(self):
        """Start AR device re-pairing process."""
        self.data_manager.log_user_action(f"AR device re-pairing initiated", f"Car #{self.car_id}")
        # TODO: Implement device pairing workflow
        
    def _reset_communication(self):
        """Reset AR communication protocols."""
        self.data_manager.log_user_action(f"AR communication reset", f"Car #{self.car_id}")
        # TODO: Implement communication protocol reset
    
    def has_pending_changes(self):
        """Check if there are unsaved changes."""
        return False  # AR tab typically doesn't have pending config changes
    
    def apply_changes(self):
        """Apply any configuration changes."""
        pass  # AR tab handles changes immediately via buttons
    
    def update_data(self):
        """Update AR integration tab with latest data."""
        car_data = self.data_manager.get_car_data(self.car_id)
        if not car_data:
            return
        
        # Update connection status
        if car_data.bluetooth_connected:
            self.connection_status.setText("🟢 Connected to AR Device")
            self.connection_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #4caf50;")
        else:
            self.connection_status.setText("🔴 AR Device Disconnected")
            self.connection_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #f44336;")
        
        # TODO: Update message statistics, logs, and device information
        # This would pull real communication data from the robot