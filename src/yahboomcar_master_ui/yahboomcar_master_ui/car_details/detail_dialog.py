#!/usr/bin/env python3
"""
Car Detail Dialog - Main popup window for individual car management.
"""

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, 
    QTabWidget, QFrame, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

from .diagnostics_tab import DiagnosticsTab
from .config_tab import ConfigTab
from .ar_integration_tab import ARIntegrationTab
from .advanced_controls_tab import AdvancedControlsTab


class CarDetailDialog(QDialog):
    """Detailed car management popup dialog."""
    
    def __init__(self, car_id, data_manager, parent=None):
        super().__init__(parent)
        self.car_id = car_id
        self.data_manager = data_manager
        self.car_data = data_manager.get_car_data(car_id)
        
        # Get car configuration for color and name
        car_configs = data_manager.get_car_configuration()
        self.car_config = next((config for config in car_configs if config["id"] == car_id), None)
        if not self.car_config:
            self.car_config = {"id": car_id, "name": f"Car {car_id}", "color": "#666666", "active": True}
        
        self.setModal(True)
        self.setWindowTitle(f"🚗 Car #{car_id} Details - {self.car_config['name']}")
        self.setFixedSize(800, 600)
        self.setStyleSheet(self._get_dialog_style())
        
        self._init_ui()
        
        # Update timer for real-time data
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data)
        self.update_timer.start(200)  # 5Hz updates
    
    def _init_ui(self):
        """Initialize the dialog UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Header with car info
        header_frame = self._create_header()
        layout.addWidget(header_frame)
        
        # Main tabbed content
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #555;
                background-color: #2b2b2b;
                border-radius: 5px;
            }
            QTabBar::tab {
                background-color: #404040;
                color: white;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 3px;
                border-top-right-radius: 3px;
            }
            QTabBar::tab:selected {
                background-color: #0066cc;
                color: white;
            }
            QTabBar::tab:hover {
                background-color: #505050;
            }
        """)
        
        # Create tabs
        self.diagnostics_tab = DiagnosticsTab(self.car_id, self.data_manager)
        self.config_tab = ConfigTab(self.car_id, self.data_manager)
        self.ar_tab = ARIntegrationTab(self.car_id, self.data_manager)
        self.advanced_tab = AdvancedControlsTab(self.car_id, self.data_manager)
        
        self.tab_widget.addTab(self.diagnostics_tab, "📊 Diagnostics")
        self.tab_widget.addTab(self.config_tab, "⚙️ Configuration")
        self.tab_widget.addTab(self.ar_tab, "📱 AR Integration")
        self.tab_widget.addTab(self.advanced_tab, "🔧 Advanced")
        
        layout.addWidget(self.tab_widget)
        
        # Bottom buttons
        button_layout = self._create_buttons()
        layout.addWidget(button_layout)
    
    def _create_header(self):
        """Create header with car identification and status."""
        header_frame = QFrame()
        header_frame.setFixedHeight(60)
        header_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {self.car_config['color']};
                border-radius: 5px;
                border: 2px solid #555;
            }}
        """)
        
        layout = QHBoxLayout(header_frame)
        layout.setContentsMargins(15, 10, 15, 10)
        
        # Car identification
        car_info_layout = QVBoxLayout()
        
        # Car name and ID
        title_label = QLabel(f"🚗 CAR #{self.car_id} - \"{self.car_config['name']}\"")
        title_label.setStyleSheet("color: black; font-weight: bold; font-size: 16px;")
        car_info_layout.addWidget(title_label)
        
        # Quick status summary
        self.status_summary = QLabel("Loading status...")
        self.status_summary.setStyleSheet("color: #222; font-size: 12px;")
        car_info_layout.addWidget(self.status_summary)
        
        layout.addLayout(car_info_layout)
        layout.addStretch()
        
        # Emergency stop button in header
        emergency_btn = QPushButton("🚨 EMERGENCY STOP")
        emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                font-weight: bold;
                padding: 10px 15px;
                border-radius: 5px;
                border: 2px solid #ff0000;
            }
            QPushButton:hover {
                background-color: #ff0000;
            }
            QPushButton:pressed {
                background-color: #aa0000;
            }
        """)
        emergency_btn.clicked.connect(self._emergency_stop)
        layout.addWidget(emergency_btn)
        
        return header_frame
    
    def _create_buttons(self):
        """Create bottom action buttons."""
        button_frame = QFrame()
        button_frame.setFixedHeight(50)
        
        layout = QHBoxLayout(button_frame)
        layout.addStretch()
        
        # Apply button for settings changes
        apply_btn = QPushButton("✅ Apply Changes")
        apply_btn.setFixedSize(120, 35)
        apply_btn.setStyleSheet("""
            QPushButton {
                background-color: #009900;
                color: white;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #00cc00;
            }
            QPushButton:pressed {
                background-color: #007700;
            }
        """)
        apply_btn.clicked.connect(self._apply_changes)
        layout.addWidget(apply_btn)
        
        # Close button
        close_btn = QPushButton("❌ Close")
        close_btn.setFixedSize(80, 35)
        close_btn.setStyleSheet("""
            QPushButton {
                background-color: #666;
                color: white;
                font-weight: bold;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #777;
            }
            QPushButton:pressed {
                background-color: #555;
            }
        """)
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)
        
        return button_frame
    
    def _update_data(self):
        """Update dialog with latest car data."""
        # Refresh car data
        self.car_data = self.data_manager.get_car_data(self.car_id)
        
        # Update header status
        if self.car_data.emergency_stopped:
            status_text = "🚨 EMERGENCY STOPPED"
        elif self.car_data.control_mode == "MANUAL":
            status_text = "🎮 Manual Control Active"
        else:
            status_text = f"🏁 Racing Mode | Battery: {self.car_data.battery_percentage}%"
        
        self.status_summary.setText(status_text)
        
        # Update all tabs
        self.diagnostics_tab.update_data()
        self.config_tab.update_data()
        self.ar_tab.update_data()
        self.advanced_tab.update_data()
    
    def _emergency_stop(self):
        """Emergency stop this car from the dialog."""
        self.data_manager.emergency_stop(self.car_id)
        self.data_manager.log_user_action(f"Emergency stop from detail dialog", f"Car #{self.car_id}")
    
    def _apply_changes(self):
        """Apply configuration changes from all tabs."""
        # Collect changes from all tabs
        changes_applied = False
        
        # Apply config tab changes
        if self.config_tab.has_pending_changes():
            self.config_tab.apply_changes()
            changes_applied = True
        
        # Apply advanced tab changes
        if self.advanced_tab.has_pending_changes():
            self.advanced_tab.apply_changes()
            changes_applied = True
        
        if changes_applied:
            self.data_manager.log_user_action(f"Applied configuration changes", f"Car #{self.car_id}")
            
            # Show brief confirmation (could enhance with status bar later)
            self.status_summary.setText("✅ Changes applied successfully")
    
    def _get_dialog_style(self):
        """Return dialog-specific dark theme stylesheet."""
        return """
        QDialog {
            background-color: #1e1e1e;
            color: #ffffff;
        }
        QWidget {
            background-color: #1e1e1e;
            color: #ffffff;
            font-family: 'Segoe UI', Arial, sans-serif;
        }
        QFrame {
            border: 1px solid #555;
            border-radius: 5px;
            background-color: #2b2b2b;
        }
        QLabel {
            color: #ffffff;
        }
        """
    
    def closeEvent(self, event):
        """Handle dialog close event."""
        # Stop the update timer
        if hasattr(self, 'update_timer'):
            self.update_timer.stop()
        
        # Check for unsaved changes
        if (self.config_tab.has_pending_changes() or 
            self.advanced_tab.has_pending_changes()):
            # Could add unsaved changes warning here
            pass
        
        event.accept()