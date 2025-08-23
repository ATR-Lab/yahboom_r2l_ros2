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
        self.setMinimumHeight(180)
        self.setMaximumHeight(300)
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
        
        layout = QHBoxLayout(self)
        layout.setSpacing(15)  # Add spacing between the two columns
        
        # Left column: Race Status
        race_column = QVBoxLayout()
        race_column.setSpacing(5)
        
        race_label = QLabel("🏁 RACE STATUS")
        race_label.setStyleSheet("font-weight: bold; font-size: 14px; margin-bottom: 5px;")
        race_column.addWidget(race_label)
        
        self.race_time_label = QLabel("⏱️ Race Time: 00:00")
        race_column.addWidget(self.race_time_label)
        
        self.lap_label = QLabel("🏁 Lap: 1/5")
        race_column.addWidget(self.lap_label)
        
        # Current leader
        self.leader_label = QLabel("🥇 Leader: Car #1")
        self.leader_label.setStyleSheet("color: #ffd700;")
        race_column.addWidget(self.leader_label)
        
        race_column.addStretch()
        layout.addLayout(race_column)
        
        # Right column: Track Conditions
        track_column = QVBoxLayout()
        track_column.setSpacing(5)
        
        conditions_label = QLabel("🌤️ TRACK CONDITIONS")
        conditions_label.setStyleSheet("font-weight: bold; font-size: 14px; margin-bottom: 5px;")
        track_column.addWidget(conditions_label)
        
        self.weather_label = QLabel("☀️ Clear, Dry")
        track_column.addWidget(self.weather_label)
        
        self.temperature_label = QLabel("🌡️ 22°C Optimal")
        track_column.addWidget(self.temperature_label)
        
        track_column.addStretch()
        layout.addLayout(track_column)
    
    def _update_data(self):
        """Update game state display with dummy data functions."""
        # TODO: Replace these with actual ROS2 topic subscriptions
        
        # Update race status from dummy data
        race_data = self.data_manager.get_race_data()
        self.race_time_label.setText(f"⏱️ Race Time: {race_data['race_time']}")
        self.lap_label.setText(f"🏁 Lap: {race_data['current_lap']}/{race_data['total_laps']}")
        
        # Update leader from dummy data  
        leader_data = self.data_manager.get_leaderboard_data()
        self.leader_label.setText(f"🥇 Leader: {leader_data['leader_car_name']}")
        
        # Update environmental data from dummy functions
        env_data = self.data_manager.get_environment_data()
        self.weather_label.setText(f"☀️ {env_data['weather']}")
        self.temperature_label.setText(f"🌡️ {env_data['temperature']}")