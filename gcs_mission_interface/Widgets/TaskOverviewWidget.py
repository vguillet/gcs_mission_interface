
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os

# Libs
from pandas import to_datetime
from PySide6.QtWidgets import QVBoxLayout, QWidget

# Own modules
from ..ui_loader import uic
from ..ui_singleton import NucleusSingleton

##################################################################################################################


class TaskOverviewWidget(QWidget):
    def __init__(self, task):
        super().__init__()

        self.task = task

        # ----------------------------------- Load GUI
        # -> Load main singleton
        self.nucleus = NucleusSingleton()

        # -> Load ui singleton
        self.ui = uic.loadUi(os.getcwd() + "/src/gcs_mission_interface/gcs_mission_interface/resources/uis/task_overview.ui")

        # > Update header
        self.__update_header()
        self.update()

        # -> Setup layout
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.ui)
        self.setLayout(self.layout)

    @property
    def current_interface_view(self) -> dict:
        current_interface_view = {}

        return current_interface_view

    @current_interface_view.setter
    def current_interface_view(self, current_interface_view) -> None:
        pass

    def __update_header(self):
        # -> Update agent details
        # self.ui.label_task_name.setText(self.task.name)
        self.ui.label_task_id.setText(str(self.task.id))
        self.ui.label_task_type.setText(str(self.task.type))

        # > Metadata
        self.ui.label_task_creator.setText(self.task.creator)

        creation_timestamp = to_datetime(self.task.creation_timestamp, unit="s")
        creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

        self.ui.label_task_creation_timestamp.setText(str(creation_timestamp))

        if self.task.termination_timestamp is not None:
            termination_timestamp = to_datetime(self.task.termination_timestamp, unit="s")
            termination_timestamp = termination_timestamp.replace(microsecond=0, nanosecond=0)

            self.ui.label_task_termination_timestamp.setText(str(termination_timestamp))
        else:
            self.ui.label_task_termination_timestamp.setText("N/A")

    def refresh(self):
        # -> Update agent state
        # > Last update timestamp
        # TODO: Finish fixing this
        # time_elapsed_since_last_update = self.ros_node.current_timestamp - self.task.creation_timestamp

        # creation_timestamp = to_datetime(self.task.creation_timestamp, unit="s")
        # creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

        # self.ui.label_last_update_timestamp.setText(str(creation_timestamp))

        # > Status
        self.ui.label_task_status.setText(self.task.status.capitalize())

        # make italics
        if self.task.status == "pending":
            self.ui.label_task_status.setStyleSheet("color: orange")
        elif self.task.status == "completed":
            self.ui.label_task_status.setStyleSheet("color: green")
        elif self.task.status == "cancelled":
            self.ui.label_task_status.setStyleSheet("color: red")
