
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os

# Libs
from PySide6.QtWidgets import QVBoxLayout, QWidget

# Own modules
from .ui_loader import uic

##################################################################################################################


class AgentOverviewWidget(QWidget):
    def __init__(self):
        super().__init__()

        # ----------------------------------- Load GUI
        # -> Load ui singleton
        self.ui = uic.loadUi(os.getcwd() + "/src/gcs_mission_interface/gcs_mission_interface/resources/uis/robot_overview.ui")

        # -> Setup layout
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.ui)
        self.setLayout(self.layout)
