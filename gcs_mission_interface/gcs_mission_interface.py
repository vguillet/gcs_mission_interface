

##################################################################################################################
"""
"""

import os
import sys
from typing import List, Optional
from json import loads, dumps
from copy import deepcopy
from functools import partial
from threading import Thread

# Libs
import numpy as np
import pandas as pd
import PySide6
from PySide6.QtCore import *
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel
from PySide6.QtGui import QIcon

# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import JointState, LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Local Imports
from .gcs_maaf_node import MAAFNode
from .ui_singleton import UiSingleton

from .AgentOverviewWidget import AgentOverviewWidget
from .TaskOverviewWidget import TaskOverviewWidget

##################################################################################################################


class gcs_mission_interface:
    def __init__(self):
        # ----------------------------------- Load GUI
        app = QtWidgets.QApplication(sys.argv)

        # -> Load ui singleton
        self.ui = UiSingleton().interface

        root_path = str(os.getcwd())
        self.ui.setWindowIcon(QIcon(root_path + "/ros2_ws/src/gcs_mission_interface/gcs_mission_interface/resources/gcs_mission_interface_logo.jpeg"))
        self.ui.setWindowTitle("GCS Mission Interface")

        # ----------------------------------- Create Node
        # ---- Init parent class
        # MAAFNode.__init__(self)

        self.ros_node = MAAFNode()

        # -> Initial reset to initialize the node
        self.__reset_local_allocation_node()

        # ---- Connect listeners
        # - Fleet listeners
        self.ros_node.fleet.add_on_add_item_listener(self.__update_agents)
        self.ros_node.fleet.add_on_update_item_listener(self.__update_agents)
        self.ros_node.fleet.add_on_remove_item_listener(self.__update_agents)

        # - Task log listeners
        self.ros_node.task_log.add_on_add_item_listener(self.__update_tasks)
        self.ros_node.task_log.add_on_update_item_listener(self.__update_tasks)
        self.ros_node.task_log.add_on_remove_item_listener(self.__update_tasks)

        # -> QT timer based spin
        self.spin_timer = QtCore.QTimer()
        self.spin_timer.timeout.connect(self.__node_spinner)
        self.spin_timer.setInterval(100)
        self.spin_timer.start()

        # ---- Connect signals
        self.ui.pushButton_reset_local_allocation_node.clicked.connect(self.__reset_local_allocation_node)

        # -> Agent selection
        self.ui.tableWidget_agents_select.itemSelectionChanged.connect(
            partial(self.__select_agent, source=self.ui.tableWidget_tasks_select)
        )      # SIDE VIEW
        self.ui.tableWidget_fleet_overview.itemSelectionChanged.connect(
            partial(self.__select_agent, source=self.ui.tableWidget_fleet_overview)
        )    # MAIN VIEW

        # -> Task selection
        self.ui.tableWidget_tasks_select.itemSelectionChanged.connect(
            partial(self.__select_task, source=self.ui.tableWidget_tasks_select)
        )   # SIDE VIEW
        self.ui.tableWidget_task_log_overview.itemSelectionChanged.connect(
            partial(self.__select_task, source=self.ui.tableWidget_task_log_overview)
        )   # MAIN VIEW

        # ----------------------------------- Final setup
        # -> Display windows
        self.ui.show()

        # ----------------------------------- Terminate ROS2 node on exit
        # self.ros_node.destroy_node()
        sys.exit(app.exec())

    # ============================================================== METHODS
    def __reset_local_allocation_node(self):
        # -> Reset allocation states
        self.ros_node.reset_allocation_states()

        # -> Setup interface
        self.ui.agents_overviews = {}
        self.ui.tasks_overviews = {}

        # -> Setup trackers
        self.selected_agent_id = None
        self.selected_task_id = None

        self.current_agent_interface_view = None
        self.current_task_interface_view = None

    # ------------------------------ Create
    def __create_agent(self, agent_id: dict) -> AgentOverviewWidget:
        # -> Create widget
        widget = AgentOverviewWidget(agent_id=agent_id, ros_node=self.ros_node)

        # -> Add to the agent overview dictionary
        self.ui.agents_overviews[agent_id] = widget

        # -> Add to the stacked widget
        self.ui.stackedWidget_agents_overviews.addWidget(widget)

        # -> Connect listeners
        self.ros_node.add_team_msg_subscriber_callback_listener(widget.update)
        self.ros_node.add_team_msg_subscriber_callback_listener(widget.add_to_logs)

        return widget

    def __create_task(self, task_id: dict) -> TaskOverviewWidget:
        # -> Create widget
        widget = TaskOverviewWidget(task=self.ros_node.task_log[task_id], ros_node=self.ros_node)

        # -> Add to the task overview dictionary
        self.ui.tasks_overviews[task_id] = widget

        # -> Add to the stacked widget
        self.ui.stackedWidget_tasks_overviews.addWidget(widget)

        return widget

    # ------------------------------ Track
    def __track_current_agent_interface_view(self, current_interface_view: dict) -> None:
        self.current_agent_interface_view = current_interface_view

    def __track_current_task_interface_view(self, current_interface_view: dict) -> None:
        self.current_task_interface_view = current_interface_view

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
        self.ui.lcdNumber_agent_count_total.display(len(self.ros_node.fleet))
        self.ui.lcdNumber_agents_count_active.display(len(self.ros_node.fleet.ids_active))
        self.ui.lcdNumber_agents_count_inactive.display(len(self.ros_node.fleet.ids_inactive))

        # TODO: Implement agents free/busy

        # ---- Comms. state
        # TODO: Implement comms. state tracking
        self.ui.lcdNumber_agent_count_total_2.display(len(self.ros_node.fleet))

        # ---- Tasks
        self.ui.lcdNumber_tasks_count_total.display(len(self.ros_node.task_log))
        self.ui.lcdNumber_tasks_count_pending.display(len(self.ros_node.task_log.ids_pending))
        self.ui.lcdNumber_tasks_count_completed.display(len(self.ros_node.task_log.ids_completed))
        self.ui.lcdNumber_tasks_count_canceled.display(len(self.ros_node.task_log.ids_cancelled))

        # ---- Mission progress
        if len(self.ros_node.task_log) == 0:
            mission_progress = 0
        else:
            mission_progress = (1 - len(self.ros_node.task_log.ids_pending) / len(self.ros_node.task_log)) * 100

        self.ui.progressBar_mission_progress.setValue(mission_progress)

    def __update_agents(self, *args, **kwargs) -> None:
        """
        Update the agent overview widgets
        """

        # ---- Update mission state overview
        self.__update_mission_state_overview()

        # -> Sort the fleet
        self.ros_node.fleet.sort(key=lambda agent: agent.id)

        # ---- Update agent MAIN VIEW
        # -> Construct table to display
        agent_ids = []
        agent_data = []

        for agent in self.ros_node.fleet:
            if agent is self.ros_node.agent:
                continue

            # -> Convert timestamp to human readable format
            state_timestamp = pd.to_datetime(agent.state.timestamp, unit="s")
            state_timestamp = state_timestamp.replace(microsecond=0, nanosecond=0)

            agent_ids.append(agent.id)
            agent_data.append(
                [
                    agent.name,
                    agent.agent_class,
                    agent.hierarchy_level,
                    agent.affiliations,
                    agent.specs,
                    agent.skillset,
                    (agent.state.x, agent.state.y, agent.state.z),
                    (agent.state.u, agent.state.v, agent.state.w),
                    agent.state.battery_level,
                    agent.state.stuck,
                    state_timestamp
                ])

        # -> If there are no agents, return
        if len(agent_data) == 0:
            # -> Clear tables
            self.ui.tableWidget_fleet_overview.clear()
            self.ui.tableWidget_agents_select.clear()

            # -> Remove all rows
            self.ui.tableWidget_fleet_overview.setRowCount(0)
            self.ui.tableWidget_agents_select.setRowCount(0)

            # -> Clear agent overviews
            self.ui.agents_overviews = {}
            return

        # -> Update table
        # > Set the column headers
        headers = ["Name", "Class", "Rank", "Affiliations", "Specs", "Skillset", "Position", "Orientation", "Battery Level", "Stuck", "State Timestamp"]
        self.ui.tableWidget_fleet_overview.setColumnCount(len(headers))
        self.ui.tableWidget_fleet_overview.setHorizontalHeaderLabels(headers)

        # > Set the ids as the row headers
        self.ui.tableWidget_fleet_overview.setRowCount(len(agent_data))
        self.ui.tableWidget_fleet_overview.setVerticalHeaderLabels([str(id) for id in agent_ids])

        # > Enable word wrap for the row headers
        self.ui.tableWidget_fleet_overview.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

        # > Set the data
        for i, row in enumerate(agent_data):
            for j, cell in enumerate(row):
                self.ui.tableWidget_fleet_overview.setItem(i, j, QtWidgets.QTableWidgetItem(str(cell)))

        # > Resize columns
        self.ui.tableWidget_fleet_overview.resizeColumnsToContents()

        # > Sort the table by row header
        self.ui.tableWidget_fleet_overview.sortItems(1, QtCore.Qt.AscendingOrder)

        # ---- Update fleet SIDE VIEW
        # -> Construct table to display
        agent_ids = []
        agent_data = []
        for agent in self.ros_node.fleet:
            if agent is self.ros_node.agent:
                continue

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

        # > Sort the table by row header
        self.ui.tableWidget_agents_select.sortItems(1, QtCore.Qt.AscendingOrder)

        # ---- Ensure all agents widgets are created
        for agent_id in agent_ids:
            if agent_id not in self.ui.agents_overviews:
                self.__create_agent(agent_id=agent_id)

        # -> Select first agent if no agent is selected
        if not self.selected_agent_id:
            self.ui.tableWidget_agents_select.selectRow(0)
            self.__select_agent(source=self.ui.tableWidget_agents_select)

    def __update_tasks(self, *args, **kwargs) -> None:
        """
        Update the task overview widgets
        """

        # ---- Update mission state overview
        self.__update_mission_state_overview()

        self.ros_node.task_log.sort(key=lambda task: task.creation_timestamp, reverse=True)

        # ---- Update task log MAIN VIEW
        # -> Construct table to display
        task_ids = []
        task_data = []

        for task in self.ros_node.task_log:
            task_ids.append(task.id)

            # -> Convert timestamp to human readable format
            creation_timestamp = pd.to_datetime(task.creation_timestamp, unit="s")
            creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

            task_data.append([task.type, task.priority, creation_timestamp, task.creator, task.affiliations, task.status.capitalize(), task.termination_source_id])

        # -> If there are no agents, return
        if len(task_data) == 0:
            # -> Clear tables rows
            self.ui.tableWidget_task_log_overview.clear()
            self.ui.tableWidget_tasks_select.clear()

            # -> Remove all rows
            self.ui.tableWidget_task_log_overview.setRowCount(0)
            self.ui.tableWidget_tasks_select.setRowCount(0)

            # -> Clear agent overviews
            self.ui.tasks_overviews = {}
            return

        # -> Update table
        # > Set the column headers
        headers = ["Type", "Priority", "Creation Timestamp", "Creator", "Affiliations", "Status", "Termination Source"]
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

        # > Sort the table by row header
        self.ui.tableWidget_task_log_overview.sortItems(1, QtCore.Qt.AscendingOrder)

        # ---- Update task log SIDE VIEW
        # -> Construct table to display
        task_ids = []
        task_data = []

        for task in self.ros_node.task_log:
            task_ids.append(task.id)

            # -> Convert timestamp to human readable format
            creation_timestamp = pd.to_datetime(task.creation_timestamp, unit="s")
            creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

            task_data.append([task.type, task.priority, creation_timestamp, task.status.capitalize()])

        # -> Update table
        self.ui.tableWidget_tasks_select.setRowCount(len(task_data))
        self.ui.tableWidget_tasks_select.setColumnCount(len(task_data[0]))

        # > Set the column headers
        self.ui.tableWidget_tasks_select.setHorizontalHeaderLabels(["Type", "Priority", "Creation Timestamp", "Status"])

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

        # > Sort the table by row header
        self.ui.tableWidget_tasks_select.sortItems(1, QtCore.Qt.AscendingOrder)

        # ---- Ensure all task widgets are created
        for task_id in task_ids:
            if task_id not in self.ui.tasks_overviews:
                self.__create_task(task_id=task_id)

        # -> Select first task if no task is selected
        if not self.selected_task_id:
            self.ui.tableWidget_tasks_select.selectRow(0)
            self.__select_task(source=self.ui.tableWidget_tasks_select)

    # ------------------------------ Select
    def __select_agent(self, source) -> None:
        """
        Select an agent from the agent overview table
        """

        # -> Get selected agent id
        if source == self.ui.tableWidget_fleet_overview:
            # > Get selected row id
            selected_agent_row = self.ui.tableWidget_fleet_overview.currentIndex().row()
            selected_agent_id = self.ui.tableWidget_fleet_overview.verticalHeaderItem(selected_agent_row).text()

            # > Select the corresponding agent in the side view
            self.ui.tableWidget_agents_select.selectRow(selected_agent_row)

        else:
            # -> Get the selected agent id
            selected_agent_row = self.ui.tableWidget_agents_select.currentIndex().row()
            selected_agent_id = self.ui.tableWidget_agents_select.verticalHeaderItem(selected_agent_row).text()

            # > Select the corresponding agent in the main view
            self.ui.tableWidget_fleet_overview.selectRow(selected_agent_row)

        # -> Get the agent overview widget
        widget = self.ui.agents_overviews[selected_agent_id]

        # -> Get current agent interface view
        if self.selected_agent_id is not None:
            self.current_agent_interface_view = self.ui.agents_overviews[self.selected_agent_id].current_interface_view

            # -> Set current agent interface view
            widget.current_interface_view = self.current_agent_interface_view

        # -> Switch stacked widget to the agent overview
        self.ui.stackedWidget_agents_overviews.setCurrentWidget(widget)

        # -> Set the selected agent id
        self.selected_agent_id = selected_agent_id

    def __select_task(self, source) -> None:
        """
        Select a task from the task overview table
        """

        # -> Get selected task id
        if source == self.ui.tableWidget_task_log_overview:
            # > Get selected row id
            selected_task_row = self.ui.tableWidget_task_log_overview.currentIndex().row()
            selected_task_id = self.ui.tableWidget_task_log_overview.verticalHeaderItem(selected_task_row).text()

            # > Select the corresponding task in the side view
            self.ui.tableWidget_tasks_select.selectRow(selected_task_row)

        else:
            # -> Get the selected task id
            selected_task_row = self.ui.tableWidget_tasks_select.currentIndex().row()
            selected_task_id = self.ui.tableWidget_tasks_select.verticalHeaderItem(selected_task_row).text()

            # > Select the corresponding task in the main view
            self.ui.tableWidget_task_log_overview.selectRow(selected_task_row)

        # -> Get the task overview widget
        widget = self.ui.tasks_overviews[selected_task_id]

        # -> Switch stacked widget to the task overview
        self.ui.stackedWidget_tasks_overviews.setCurrentWidget(widget)

        # -> Set the selected task id
        self.selected_task_id = selected_task_id

    # ================================================= Custom ROS2 integration
    def __node_spinner(self):
        # -> Spin once
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        # rclpy.spin(self.ros_node)



def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    app = gcs_mission_interface()

    rclpy.shutdown()

# def main(args=None):
#     # `rclpy` library is initialized
#     rclpy.init(args=args)
#
#     ros_node = MAAFNode()
#
#     app = QtWidgets.QApplication(sys.argv)
#     gui = gcs_mission_interface(ros_node=ros_node)
#
#     executor = MultiThreadedExecutor()
#     executor.add_node(ros_node)
#
#     # -> Start the ROS2 node on a separate thread
#     thread = Thread(target=executor.spin)
#     thread.start()
#
#     # -> Start the GUI on the main thread
#     try:
#         gui.ui.show()
#         sys.exit(app.exec())
#
#     finally:
#         # `rclpy` library is shutdown
#         ros_node.destroy_node()
#         rclpy.shutdown()


if __name__ == '__main__':
    main()