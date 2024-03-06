
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os

# Libs
from pandas import to_datetime
from PySide6.QtWidgets import QVBoxLayout, QWidget
from PySide6.QtGui import QColor, QPainter, QBrush

# Own modules
from ..ui_loader import uic

##################################################################################################################


class BaseTaskWidget(QWidget):
    def __init__(self, task, ros_node):
        super().__init__()

        self.task = task
        self.ros_node = ros_node

        # ----------------------------------- Load GUI
        # -> Load ui singleton
        self.ui = uic.loadUi(os.getcwd() + "/src/gcs_mission_interface/gcs_mission_interface/resources/uis/BaseTaskWidget.ui")

        # > Update header
        self.__update_header()
        self.refresh()

        # -> Setup layout
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.ui)
        self.setLayout(self.layout)

        # -> Add visible edge to widget
        self.setStyleSheet("border: 1px solid black;")  # Set border style

        # -> Make widget selectable
        self.selected = False

        self.setMinimumHeight(20)  # Set the minimum height for the widget

    def mousePressEvent(self, event):
        # -> Deselect all other widgets
        for widget in self.ui.tasks_base_widgets:
            widget.selected = False
            widget.update()

        # -> Select this widget
        self.selected = True
        self.update()

    def mouseReleaseEvent(self, event):
        self.selected = False
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.selected:
            painter.setBrush(QColor(255, 165, 0))  # Orange color when selected
        else:
            painter.setBrush(QColor(255, 255, 255))  # Normal color
        painter.drawRect(self.rect())

    def __update_header(self):
        # -> Update agent details
        # self.ui.label_task_name.setText(self.task.name)
        self.ui.label_task_id.setText(str(self.task.id))
        self.ui.label_task_type.setText(str(self.task.type))
        self.ui.label_task_priority.setText(str(self.task.priority))

        # ->> Metadata
        self.ui.label_creation_source_id.setText(self.task.creator)

        # > Creation timestamp
        creation_timestamp = to_datetime(self.task.creation_timestamp, unit="s")
        creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

        self.ui.label_creation_timestamp.setText(str(creation_timestamp))

        # > Instructions
        for instruction in self.task.instructions:
            self.ui.listWidget_tasks_instructions.addItem(instruction)

        # > Affilitations
        for affilitation in self.task.affiliations:
            self.ui.listWidget_tasks_affilitations.addItem(affilitation)

    def refresh(self):
        # -> Update agent state
        # > Last update timestamp
        # TODO: Finish fixing this
        # time_elapsed_since_last_update = self.ros_node.current_timestamp - self.task.creation_timestamp

        # creation_timestamp = to_datetime(self.task.creation_timestamp, unit="s")
        # creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

        # self.ui.label_last_update_timestamp.setText(str(creation_timestamp))

        # > Termination timestamp
        if self.task.termination_timestamp is not None:
            termination_timestamp = to_datetime(self.task.termination_timestamp, unit="s")
            termination_timestamp = termination_timestamp.replace(microsecond=0, nanosecond=0)

            self.ui.label_termination_timestamp.setText(str(termination_timestamp))
        else:
            self.ui.label_termination_timestamp.setText("N/A")

        # > Status
        self.ui.label_task_status.setText(self.task.status.capitalize())

        # make italics
        if self.task.status == "pending":
            self.ui.label_task_status.setStyleSheet("color: orange")
        elif self.task.status == "completed":
            self.ui.label_task_status.setStyleSheet("color: green")
        elif self.task.status == "cancelled":
            self.ui.label_task_status.setStyleSheet("color: red")


