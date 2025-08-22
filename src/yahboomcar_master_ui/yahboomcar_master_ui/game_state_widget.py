#!/usr/bin/env python3
"""
Game State Widget - Race progress and game element monitoring.
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5.QtCore import QTimer


class GameStateWidget(QWidget):
    """Widget for displaying game state and race progress."""
    
    def __init__(self, data_manager):
        super().__init__()
        self.data_manager = data_manager
        self.setFixedHeight(180)
        self._init_ui()
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_data)
        self.update_timer.start(1000)  # 1Hz updates
    
    def _init_ui(self):
        """Initialize the game state UI."""
        self.setStyleSheet("""
            QWidget {
                background-color: #2b2b2b;
                border: 1px solid #555;
                border-radius: 5px;
                padding: 5px;
            }
        """)
        
        layout = QVBoxLayout(self)
        
        # Race status
        race_label = QLabel("🏁 RACE STATUS")
        race_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(race_label)
        
        self.race_time_label = QLabel("⏱️ Race Time: 00:00")
        layout.addWidget(self.race_time_label)
        
        self.lap_label = QLabel("🏁 Lap: 1/5")
        layout.addWidget(self.lap_label)
        
        # Current leader
        self.leader_label = QLabel("🥇 Leader: Car #1")
        self.leader_label.setStyleSheet("color: #ffd700;")
        layout.addWidget(self.leader_label)
        
        # Track conditions
        conditions_label = QLabel("🌤️ TRACK CONDITIONS")
        conditions_label.setStyleSheet("font-weight: bold; font-size: 12px; margin-top: 10px;")
        layout.addWidget(conditions_label)
        
        self.weather_label = QLabel("☀️ Clear, Dry")
        layout.addWidget(self.weather_label)
        
        self.temperature_label = QLabel("🌡️ 22°C Optimal")
        layout.addWidget(self.temperature_label)
    
    def _update_data(self):
        """Update game state display."""
        # This is placeholder data - in Phase 3, this will connect to real game state
        pass