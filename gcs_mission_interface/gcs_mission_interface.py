

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
from pprint import pprint, pformat

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
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from maaf_allocation_node.fleet_dataclasses import Agent, Fleet
from maaf_allocation_node.task_dataclasses import Task, Task_log
from maaf_allocation_node.tools import *

from .gcs_maaf_node import MAAFNode
from .ui_singleton import UiSingleton

from .Widgets.AgentOverviewWidget import AgentOverviewWidget
from .Widgets.TaskOverviewWidget import TaskOverviewWidget
from .Widgets.BaseTaskWidget import BaseTaskWidget
from .Widgets.GraphEnvView import GraphEnvView

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
        self.ros_node = MAAFNode()

        # -> Extend the ui singleton
        self.selected_agent_id = None
        self.selected_task_id = None

        self.current_agent_interface_view = None
        self.current_task_interface_view = None

        self.ui.env_widget = None

        # -> Add env widget update listener to env update listeners
        self.ros_node.add_on_env_update_listener(self.__update_env_view)

        # ---- Connect listeners
        # - Fleet listeners
        self.ros_node.fleet.add_on_add_item_listener(self.__create_agent)
        self.ros_node.fleet.add_on_edit_list_listener(self.__update_agents)
        self.ros_node.fleet.add_on_state_change_listener(self.__update_env_view)

        # - Task log listeners
        self.ros_node.task_log.add_on_add_item_listener(self.__create_task)
        self.ros_node.task_log.add_on_edit_list_listener(self.__update_tasks)
        self.ros_node.task_log.add_on_status_change_listener(self.__update_env_view)

        # -> QT timer based spin
        self.spin_timer = QtCore.QTimer()
        self.spin_timer.timeout.connect(self.__node_spinner)
        self.spin_timer.setInterval(10)
        self.spin_timer.start()

        # ---- Connect signals
        self.ui.pushButton_reset_local_allocation_node.clicked.connect(self.__reset_local_allocation_node)
        self.ui.pushButton_task_overview_view_switcher.clicked.connect(self.__switch_task_overview_view_mode)

        # Toggle switch once to set the initial view and button text
        self.__switch_task_overview_view_mode()

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
    def __reset_local_allocation_node(self, *args, **kwargs):
        # -> Reset allocation states
        self.ros_node.reset_allocation_states()

        # -> Setup interface
        for agent in self.ros_node.fleet:
            agent.local["overview_widget"] = None

        for task in self.ros_node.task_log:
            task.local["overview_widget"] = None
            task.local["base_widget"] = None

        # -> Setup trackers
        self.selected_agent_id = None
        self.selected_task_id = None

        self.current_agent_interface_view = None
        self.current_task_interface_view = None

    # ------------------------------ Create
    def __create_agent(self, agent: Agent, *args, **kwargs) -> None:
        """
        Create all the widgets and subscribers for the agent
        """
        # ----- Overview widget
        # -> Create widget
        widget = AgentOverviewWidget(agent_id=agent.id, ros_node=self.ros_node)

        # -> Add to the agent local
        agent.local["overview_widget"] = widget

        # -> Add to the stacked widget
        self.ui.stackedWidget_agents_overviews.addWidget(widget)

        # -> Connect widget listeners
        self.ros_node.add_team_msg_subscriber_callback_listener(widget.update)
        self.ros_node.add_team_msg_subscriber_callback_listener(widget.add_to_logs)

        # ----- Pose subscriber
        # -> Create subscriber
        pose_subscriber = self.ros_node.create_subscription(
            msg_type=PoseStamped,
            topic=f"/{agent.id}{topic_pose}",
            callback=partial(self.pose_subscriber_callback, agent=agent),
            qos_profile=qos_pose
        )

        # -> Add to the agent local
        agent.local["pose_subscriber"] = pose_subscriber

    def __create_task(self, task: Task, *args, **kwargs) -> None:
        # -> Create overview widget
        overview_widget = TaskOverviewWidget(task=task, ros_node=self.ros_node)

        # > Add to the task local
        task.local["overview_widget"] = overview_widget

        # > Add to the stacked widget
        self.ui.stackedWidget_tasks_overviews.addWidget(overview_widget)

        # -> Create base widget
        base_widget = BaseTaskWidget(task=task, ros_node=self.ros_node)

        # > Add to the task local
        task.local["base_widget"] = base_widget

        # -> Add widget to layout
        self.ui.verticalLayout_tasks_overview_base_widgets.addWidget(base_widget)

    # ------------------------------ Track
    def __track_current_agent_interface_view(self, current_interface_view: dict) -> None:
        self.current_agent_interface_view = current_interface_view

    def __track_current_task_interface_view(self, current_interface_view: dict) -> None:
        self.current_task_interface_view = current_interface_view

    # ------------------------------ Update
    def __switch_task_overview_view_mode(self, *args, **kwargs) -> None:
        # -> If the stacked widget current view is the raw view
        if self.ui.stackedWidget_task_overview_views.currentWidget() == self.ui.page_raw_view:
            # > Switch to the processed view
            self.ui.stackedWidget_task_overview_views.setCurrentWidget(self.ui.page_rendered_view)

            # -> Update button text
            self.ui.pushButton_task_overview_view_switcher.setText("Switch to Raw View")

        # -> Else, if the stacked widget current view is the processed view
        else:
            # > Switch to the raw view
            self.ui.stackedWidget_task_overview_views.setCurrentWidget(self.ui.page_raw_view)

            # -> Update button text
            self.ui.pushButton_task_overview_view_switcher.setText("Switch to Rendered View")

    def __update_env_view(self, *args, **kwargs) -> None:
        """
        Update the interface
        """

        # -> If no environment widget exists, create one
        if self.ui.env_widget is None:
            if self.ros_node.env["type"] == "graph":
                self.ui.env_widget = GraphEnvView(agent_id=self.ros_node.agent.id, ros_node=self.ros_node)
                self.ui.verticalLayout_env_view.addWidget(self.ui.env_widget)

            # TODO: Add support for more environment types

            # -> Add env widget update listener to env update listeners
            self.ros_node.add_on_env_update_listener(self.ui.env_widget.update)

        # -> If the wrong environment widget exists, remove it and create the correct one
        elif self.ros_node.env["type"] != self.ui.env_widget.env_type:
            self.ui.verticalLayout_env_view.removeWidget(self.ui.env_widget)
            self.ui.env_widget = None
            self.__update_env_view()

        else:
            self.ui.env_widget.update()

    def __update_mission_state_overview(self, *args, **kwargs) -> None:
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
            # -> Skip self
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

        # ---- Update task log MAIN VIEW Rendered
        # -> Update base widgets
        for task in self.ros_node.task_log:
            task.local["base_widget"].refresh()

        # ---- Update task log MAIN VIEW Raw
        # -> Construct table to display
        task_ids = []
        task_data = []

        for task in self.ros_node.task_log:
            task_ids.append(task.id)

            # -> Convert timestamp to human readable format
            creation_timestamp = pd.to_datetime(task.creation_timestamp, unit="s")
            creation_timestamp = creation_timestamp.replace(microsecond=0, nanosecond=0)

            # -> If termination timestamp is not None, convert it to human readable format
            if task.termination_timestamp is not None:
                termination_timestamp = pd.to_datetime(task.termination_timestamp, unit="s")
                termination_timestamp = termination_timestamp.replace(microsecond=0, nanosecond=0)
            else:
                termination_timestamp = "N/A"

            # -> If agent is None, record N.A.
            if task.termination_source_id is None:
                termination_source_id = "N/A"
            else:
                termination_source_id = task.termination_source_id

            task_data.append([
                task.type,
                task.priority,
                task.affiliations,
                task.creator,
                creation_timestamp,
                termination_timestamp,
                task.status.capitalize(),
                termination_source_id
            ])

        # -> If there are no agents, return
        if len(task_data) == 0:
            # -> Clear tables rows
            self.ui.tableWidget_task_log_overview.clear()
            self.ui.tableWidget_tasks_select.clear()

            # -> Remove all rows
            self.ui.tableWidget_task_log_overview.setRowCount(0)
            self.ui.tableWidget_tasks_select.setRowCount(0)

            return

        # -> Update table
        # > Set the column headers
        headers = [
            "Type",
            "Priority",
            "Affiliations",
            "Creator",
            "Creation Timestamp",
            "Termination Timestamp",
            "Status",
            "Termination Source"
        ]

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

        # -> Select first task if no task is selected
        if not self.selected_task_id:
            self.ui.tableWidget_tasks_select.selectRow(0)
            self.__select_task(source=self.ui.tableWidget_tasks_select)

    # ------------------------------ Select
    def __select_agent(self, source, *args, **kwargs) -> None:
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
        self.ros_node.get_logger().info(f"Selected agent id: {selected_agent_id}")
        widget = self.ros_node.fleet[selected_agent_id].local["overview_widget"]

        if self.selected_agent_id is not None:
            # -> Get current agent interface view
            self.current_agent_interface_view = self.ros_node.fleet[self.selected_agent_id].local["overview_widget"].current_interface_view

            # -> Set current agent interface view
            widget.current_interface_view = self.current_agent_interface_view

        # -> Switch stacked widget to the agent overview
        self.ui.stackedWidget_agents_overviews.setCurrentWidget(widget)

        # -> Set the selected agent id
        self.selected_agent_id = selected_agent_id

    def __select_task(self, source, *args, **kwargs) -> None:
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
        widget = self.ros_node.task_log[selected_task_id].local["overview_widget"]

        # -> Switch stacked widget to the task overview
        self.ui.stackedWidget_tasks_overviews.setCurrentWidget(widget)

        # -> Set the selected task id
        self.selected_task_id = selected_task_id

    # ================================================= Custom ROS2 integration
    def pose_subscriber_callback(self, pose_msg, agent) -> None:
        """
        Callback for the pose subscriber

        :param pose_msg: PoseStamped message
        """
        # -> Convert quaternion to euler
        u, v, w = euler_from_quaternion(quat=pose_msg.pose.orientation)

        # -> Update state
        # > Pose
        agent.state.x = pose_msg.pose.position.x
        agent.state.y = pose_msg.pose.position.y
        agent.state.z = pose_msg.pose.position.z
        agent.state.u = u
        agent.state.v = v
        agent.state.w = w

        # > Timestamp   # TODO: Review
        # agent.state.timestamp = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9

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