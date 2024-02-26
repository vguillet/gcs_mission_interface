

##################################################################################################################
"""
"""

import sys
import os
from typing import List, Optional
from json import loads, dumps
from copy import deepcopy
import numpy as np
import pandas as pd

# Libs
import PySide6
from PySide6.QtCore import *
import sys
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel
from PySide6.QtGui import QIcon

# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import JointState, LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

# Local Imports
from .gcs_maaf_node import MAAFNode
from .ui_singleton import UiSingleton

from .AgentOverviewWidget import AgentOverviewWidget

##################################################################################################################


class gcs_mission_interface(MAAFNode):
    def __init__(self):
        # ----------------------------------- Load GUI
        app = QtWidgets.QApplication([])

        # -> Load ui singleton
        self.ui = UiSingleton().interface

        root_path = str(os.getcwd())
        self.ui.setWindowIcon(QIcon(root_path + "/ros2_ws/src/gcs_mission_interface/gcs_mission_interface/resources/gcs_mission_interface_logo.jpeg"))
        self.ui.setWindowTitle("GCS Mission Interface")

        # ----------------------------------- Create Node
        # ---- Init parent class
        MAAFNode.__init__(self)

        # ---- Extend singleton
        self.ui.agents_overviews = {}
        self.ui.tasks_overviews = {}

        # ---- Connect listeners
        # - Fleet listeners
        self.fleet.add_on_add_item_listener(self.__update_agent_overview)
        self.fleet.add_on_update_item_listener(self.__update_agent_overview)
        self.fleet.add_on_remove_item_listener(self.__update_agent_overview)

        # - Task log listeners
        self.task_log.add_on_add_item_listener(self.__update_tasks)
        self.task_log.add_on_update_item_listener(self.__update_tasks)
        self.task_log.add_on_remove_item_listener(self.__update_tasks)

        # -> QT timer based spin
        self.spin_timer = QtCore.QTimer()
        self.spin_timer.timeout.connect(self.__node_spinner)
        self.spin_timer.setInterval(1)
        self.spin_timer.start(10)

        # ---- Connect signals
        # -> Agent selection
        self.ui.tableWidget_tasks_select.itemSelectionChanged.connect(self.__select_agent)

        # -> Task selection
        self.ui.tableWidget_tasks_select.itemSelectionChanged.connect(self.__select_task)

        # ----------------------------------- Final setup
        # -> Display windows
        self.ui.show()

        # -> Spin once
        # rclpy.spin_once(self.node)

        sys.exit(app.exec())

    # ============================================================== METHODS
    # ------------------------------ Update
    def __update_interface(self) -> None:
        """
        Update the interface
        """
        self.__update_mission_state_overview()

    def __update_mission_state_overview(self) -> None:
        """
        Update the mission state overview widgets
        """

        # ---- Fleet
        self.ui.lcdNumber_agent_count_total.display(len(self.fleet))
        self.ui.lcdNumber_agents_count_active.display(len(self.fleet.ids_active))
        self.ui.lcdNumber_agents_count_inactive.display(len(self.fleet.ids_inactive))

        # TODO: Implement agents free/busy

        # ---- Comms. state
        # TODO: Implement comms. state tracking
        self.ui.lcdNumber_agent_count_total_2.display(len(self.fleet))

        # ---- Tasks
        self.ui.lcdNumber_tasks_count_total.display(len(self.task_log))
        self.ui.lcdNumber_tasks_count_pending.display(len(self.task_log.ids_pending))
        self.ui.lcdNumber_tasks_count_completed.display(len(self.task_log.ids_completed))
        self.ui.lcdNumber_tasks_count_canceled.display(len(self.task_log.ids_cancelled))

        # ---- Mission progress
        if len(self.task_log) == 0:
            mission_progress = 0
        else:
            mission_progress = (1 - len(self.task_log.ids_pending) / len(self.task_log)) * 100

        self.ui.progressBar_mission_progress.setValue(mission_progress)

    def __update_agent_overview(self, *args, **kwargs) -> None:
        """
        Update the agent overview widgets
        """

        print("Updating agent overview")

        # -> Construct table to display
        agent_ids = []
        agent_data = []
        for agent in self.fleet:
            agent_ids.append(agent.id)
            agent_data.append([agent.name, agent.agent_class])

        # -> Update table
        # > Set the column headers
        headers = ["Name", "Class"]
        self.ui.tableWidget_agents_select.setColumnCount(len(headers))
        self.ui.tableWidget_agents_select.setHorizontalHeaderLabels(headers)

        # -> Set the ids as the row headers
        self.ui.tableWidget_agents_select.setRowCount(len(agent_data))
        self.ui.tableWidget_agents_select.setVerticalHeaderLabels([str(id) for id in agent_ids])

        # > Enable word wrap for the row headers
        self.ui.tableWidget_agents_select.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

        # > Set the data
        for i, row in enumerate(agent_data):
            for j, cell in enumerate(row):
                self.ui.tableWidget_agents_select.setItem(i, j, QtWidgets.QTableWidgetItem(str(cell)))

        # > Resize columns
        self.ui.tableWidget_tasks_select.resizeColumnsToContents()

    def __update_tasks(self, *args, **kwargs) -> None:
        """
        Update the task overview widgets
        """

        print("Updating task overview")

        # ---- Update task log overview
        # -> Construct table to display
        task_ids = []
        task_data = []

        for task in self.task_log:
            task_ids.append(task.id)

            # -> Convert timestamp to human readable format
            creation_timestamp = pd.to_datetime(task.creation_timestamp, unit="s")
            creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

            task_data.append([task.type, task.priority, creation_timestamp, task.creator, task.affiliations, task.status])

        # -> Update table
        # > Set the column headers
        headers = ["Type", "Priority", "Creation Timestamp", "Creator", "Affiliations", "Status"]
        self.ui.tableWidget_task_log_overview.setColumnCount(len(headers))
        self.ui.tableWidget_task_log_overview.setHorizontalHeaderLabels(headers)

        # > Set the ids as the row headers
        self.ui.tableWidget_task_log_overview.setRowCount(len(task_data))
        self.ui.tableWidget_task_log_overview.setVerticalHeaderLabels([str(id) for id in task_ids])

        # > Enable word wrap for the row headers
        self.ui.tableWidget_task_log_overview.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

        # > Set the data
        for i, row in enumerate(task_data):
            for j, cell in enumerate(row):
                self.ui.tableWidget_task_log_overview.setItem(i, j, QtWidgets.QTableWidgetItem(str(cell)))

        # > Resize columns
        self.ui.tableWidget_task_log_overview.resizeColumnsToContents()

        # ---- Update task overview
        # -> Construct table to display
        task_ids = []
        task_data = []
        for task in self.task_log:
            task_ids.append(task.id)

            # -> Convert timestamp to human readable format
            creation_timestamp = pd.to_datetime(task.creation_timestamp, unit="s")
            creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

            task_data.append([task.type, task.priority, creation_timestamp])

        # -> Update table
        self.ui.tableWidget_tasks_select.setRowCount(len(task_data))
        self.ui.tableWidget_tasks_select.setColumnCount(len(task_data[0]))

        # > Set the column headers
        self.ui.tableWidget_tasks_select.setHorizontalHeaderLabels(["Type", "Priority", "Creation Timestamp"])

        # > Set the ids as the row headers
        self.ui.tableWidget_tasks_select.setVerticalHeaderLabels([str(id) for id in task_ids])

        # > Enable word wrap for the row headers
        self.ui.tableWidget_tasks_select.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

        # > Set the data
        for i, row in enumerate(task_data):
            for j, cell in enumerate(row):
                self.ui.tableWidget_tasks_select.setItem(i, j, QtWidgets.QTableWidgetItem(str(cell)))

        # > Resize columns
        self.ui.tableWidget_tasks_select.resizeColumnsToContents()

    # ------------------------------ Select
    def __select_agent(self) -> None:
        """
        Select an agent from the agent overview table
        """
        return

        # -> Get the selected agent id
        selected_agent_id = self.ui.tableWidget_agents_select.currentItem().row()

        # -> Get the agent object
        selected_agent = self.fleet[selected_agent_id]

        # -> Get the agent overview widget
        if selected_agent_id not in self.ui.agents_overviews:
            # -> Create widget
            widget = AgentOverviewWidget()

            # -> Add to the agent overview dictionary
            self.ui.agents_overviews[selected_agent_id] = widget

            # -> Add to the stacked widget
            self.ui.stackedWidget.addWidget(widget)

        else:
            widget = self.ui.agents_overviews[selected_agent_id]

        # -> Switch stacked widget to the agent overview
        self.ui.stackedWidget.setCurrentWidget(widget)

    def __select_task(self) -> None:
        """
        Select a task from the task overview table
        """

        pass

    # ================================================= Custom ROS2 integration
    def __node_spinner(self):
        # -> Spin once
        rclpy.spin_once(self)

        # -> Update mission state overview
        # self.__update_interface()


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    # app = QApplication(sys.argv)

    app = gcs_mission_interface()


if __name__ == '__main__':
    main()