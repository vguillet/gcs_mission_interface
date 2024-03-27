

##################################################################################################################
"""
"""

import os
import random
import sys
import time
from typing import List, Optional
from json import loads, dumps
from functools import partial
from threading import Thread
from pprint import pprint, pformat
from random import randint
from copy import copy, deepcopy

# Libs
import numpy as np
import pandas as pd
import PySide6
from PySide6.QtCore import *
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel
from PySide6.QtGui import QIcon
from json import loads, dumps

# ROS2
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import JointState, LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Local Imports
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE

from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.AgentState import AgentState
from maaf_tools.datastructures.agent.Plan import Plan

from maaf_tools.tools import *
from maaf_tools.Event_bus import EventBus

from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

from .gcs_maaf_node import MAAFNode
from .GCSNode import GCSNode
from .ui_singleton import UiSingleton, NucleusSingleton

from .Widgets.AgentOverviewWidget import AgentOverviewWidget
from .Widgets.TaskOverviewWidget import TaskOverviewWidget
from .Widgets.BaseTaskWidget import BaseTaskWidget
from .Widgets.GraphEnvView import GraphEnvView


##################################################################################################################


class gcs_mission_interface:
    def __init__(self, interface_id: str):
        # ----------------------------------- Load GUI
        app = QtWidgets.QApplication(sys.argv)

        # -> Load ui singleton
        self.ui = UiSingleton().ui

        root_path = str(os.getcwd())
        self.ui.setWindowIcon(QIcon(root_path + "/ros2_ws/src/gcs_mission_interface/gcs_mission_interface/resources/gcs_mission_interface_logo.jpeg"))
        self.ui.setWindowTitle("GCS Mission Interface")

        self.interface_widgets = {
            "agents": {},
            "tasks": {}
        }

        self.eventbus = EventBus()
        self.nucleus = NucleusSingleton()

        # > Create shallow copy to avoid concurrent access
        # self.env = copy(self.nucleus.env)
        self.fleet = copy(self.nucleus.fleet)
        self.task_log = copy(self.nucleus.task_log)

        # ----------------------------------- Add all update methods reference to the nucleus
        # self.nucleus.update_all = self.__update_all
        # self.nucleus.update_env_view = self.__update_env_view
        # self.nucleus.update_agents = self.__update_agents
        # self.nucleus.update_tasks = self.__update_tasks

        # ----------------------------------- Create Node
        self.nucleus.id = interface_id

        # -> Extend the ui singleton
        self.selected_agent_id = None
        self.selected_task_id = None

        self.current_agent_interface_view = None
        self.current_task_interface_view = None

        self.ui.env_widget = None

        # ---- Init ros node
        # -> Init the gui event bus connections to the ros2 node
        self.init_event_bus_connections()

        # self.ros_node = GCSNode(interface_id=interface_id)
        # self.single_threaded_executor = SingleThreadedExecutor()
        # self.single_threaded_executor.add_node(self.ros_node)
        #
        # # -> QT timer based spin
        # self.spin_timer = QtCore.QTimer()
        # self.spin_timer.timeout.connect(self.__node_spinner)
        # self.spin_timer.setInterval(1)
        # self.spin_timer.start(0.1)

        self.ros_thread = ROS_Thread(ros_node=GCSNode(interface_id=interface_id))
        self.ros_thread.start()

        # -> QT timer based refresh
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.__update_all)
        # self.plot_timer.setInterval(16.666666666666668)  # 60 Hz
        self.plot_timer.setInterval(32)  # 30 Hz
        # self.plot_timer.setInterval(128)  # 30 Hz
        self.plot_timer.start()

        # ---- Connect local signals
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

        # self.ros_node.get_logger().info(f"GCS Mission Interface ({interface_id}) initialized")

        # ----------------------------------- Final setup
        # -> Display windows
        self.ui.show()

        # ----------------------------------- Terminate ROS2 node on exit

        sys.exit(app.exec())
        self.ros_node.destroy_node()

    def init_event_bus_connections(self):
        # -> Add env widget update listener to env update listeners
        self.eventbus.subscribe(
            topic=["ros_node", "env_update"],
            callbacks=self.__update_env_view,
            subscriber_id="env_update"
        )

        # ---- Connect listeners
        # - Fleet listeners
        # > Fleet add item
        self.eventbus.subscribe(
            topic=["ros_node", "fleet", "add_item"],
            callbacks=self.__create_agent,
            subscriber_id="add_agent"
        )

        # > Fleet remove item
        self.eventbus.subscribe(
            topic=["ros_node", "fleet", "remove_item"],
            callbacks=self.__update_agents,     # TODO: Fix once remove agent is implemented
            subscriber_id="remove_agent"
        )

        # > Fleet state change
        self.eventbus.subscribe(
            topic=["ros_node", "fleet", "state_change"],
            callbacks=[
                self.__update_agents,
                # self.__update_env_view
            ],
            subscriber_id="agent_state_change"
        )

        # - Task log listeners
        # > Task log add item
        self.eventbus.subscribe(
            topic=["ros_node", "task_log", "add_item"],
            callbacks=self.__create_task,
            subscriber_id="add_task"
        )

        # > Task log remove item
        self.eventbus.subscribe(
            topic=["ros_node", "task_log", "remove_item"],
            callbacks=self.__update_tasks,      # TODO: Fix once remove task is implemented
            subscriber_id="remove_task"
        )

        # > Task log status change
        self.eventbus.subscribe(
            topic=["ros_node", "task_log", "status_change"],
            callbacks=[
                self.__update_tasks,
                # self.__update_env_view
            ],
            subscriber_id="task_status_change"
        )

        print("Event bus connections initialized")
        pprint(self.eventbus.tree, width=1, indent=1, compact=True, depth=2)

    # ============================================================== METHODS
    def __reset_local_allocation_node(self, *args, **kwargs):
        return

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
        widget = AgentOverviewWidget(agent_id=agent.id)

        # -> Add to the interface widgets
        if agent.id not in self.interface_widgets["agents"].keys():
            self.interface_widgets["agents"][agent.id] = {
                "overview_widget": widget
            }
        else:
            self.interface_widgets["agents"][agent.id]["overview_widget"] = widget

        # -> Add to the stacked widget
        self.ui.stackedWidget_agents_overviews.addWidget(widget)

        # -> Connect widget listeners
        self.eventbus.subscribe(
            topic=["ros_node", "team_msg"],
            callbacks=[
                widget.update,
                # widget.add_to_logs
            ],
            subscriber_id=f"agent_{agent.id}_update"
        )

        # ----- Pose subscriber
        # -> Create subscriber
        # pose_subscriber = self.ros_node.create_subscription(
        #     msg_type=PoseStamped,
        #     topic=f"/{agent.id}{topic_pose}",
        #     callback=partial(self.pose_subscriber_callback, agent=agent),
        #     qos_profile=qos_pose
        # )
        #
        # # -> Add to the agent local
        # agent.local["pose_subscriber"] = pose_subscriber

    def __create_task(self, task: Task, *args, **kwargs) -> None:
        # -> Create overview widget
        overview_widget = TaskOverviewWidget(task=task)

        # -> Add to the interface widgets
        if task.id not in self.interface_widgets["tasks"].keys():
            self.interface_widgets["tasks"][task.id] = {
                "overview_widget": overview_widget
            }
        else:
            self.interface_widgets["tasks"][task.id]["overview_widget"] = overview_widget

        # > Add to the stacked widget
        self.ui.stackedWidget_tasks_overviews.addWidget(overview_widget)

        # -> Create base widget
        base_widget = BaseTaskWidget(task=task)

        # > Add to the task local
        self.interface_widgets["tasks"][task.id]["base_widget"] = base_widget

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

    def __update_all(self, *args, **kwargs) -> None:
        # return
        self.__update_agents(agent=None)
        self.__update_tasks(task=None)
        self.__update_env_view(env=None)

    def __update_env_view(self, env, *args, **kwargs) -> None:
        """
        Update the interface
        """

        if env is not None:
            self.nucleus.env = env

        elif not self.nucleus.env:
            return

        env = copy(self.nucleus.env)

        # -> If no environment widget exists, create one
        if self.ui.env_widget is None:
            if env["env_type"] == "graph":
                self.ui.env_widget = GraphEnvView()
                self.ui.verticalLayout_env_view.addWidget(self.ui.env_widget)
                self.ui.env_widget.update_env()

            else:
                # TODO: Add support for more environment types
                raise NotImplementedError("Environment type not supported")

            # # -> Add env widget update listener to env update listeners
            # self.eventbus.subscribe(
            #     topic=["ros_node", "env_update"],
            #     callbacks=self.ui.env_widget.update_plot,
            #     subscriber_id="gui_env_update"
            # )

        # -> If the wrong environment widget exists, remove it and create the correct one
        elif env["env_type"] != self.ui.env_widget.env_type:
            self.ui.verticalLayout_env_view.removeWidget(self.ui.env_widget)
            self.ui.env_widget = None
            self.__update_env_view(env=env["env_type"])

        else:
            self.ui.env_widget.update_plot()

    def __update_mission_state_overview(self, *args, **kwargs) -> None:
        """
        Update the mission state overview widgets
        """

        # -> Clone the fleet and task log to avoid concurrent access
        fleet = self.fleet.clone()
        task_log = self.task_log.clone()

        # ---- Fleet
        self.ui.lcdNumber_agent_count_total.display(len(fleet))
        self.ui.lcdNumber_agents_count_active.display(len(fleet.ids_active))
        self.ui.lcdNumber_agents_count_inactive.display(len(fleet.ids_inactive))

        # TODO: Implement agents free/busy

        # ---- Comms. state
        # TODO: Implement comms. state tracking
        self.ui.lcdNumber_agent_count_total_2.display(len(fleet))

        # ---- Tasks
        self.ui.lcdNumber_tasks_count_total.display(len(task_log))
        self.ui.lcdNumber_tasks_count_pending.display(len(task_log.ids_pending))
        self.ui.lcdNumber_tasks_count_completed.display(len(task_log.ids_completed))
        self.ui.lcdNumber_tasks_count_canceled.display(len(task_log.ids_cancelled))

        # ---- Mission progress
        if len(task_log) == 0:
            mission_progress = 0
        else:
            mission_progress = (1 - len(task_log.ids_pending) / len(task_log)) * 100

        self.ui.progressBar_mission_progress.setValue(mission_progress)

    def __update_agents(self, agent: Agent, *args, **kwargs) -> None:
        """
        Update the agent overview widgets
        """

        fleet = self.fleet.clone()   # > Clone to avoid concurrent access

        # ---- Update mission state overview
        self.__update_mission_state_overview()

        # ---- Update agent view OVERVIEW
        for agent in fleet:
            if "overview_widget" not in agent.local:
                self.__create_agent(agent)
            else:
                self.interface_widgets["agents"][agent.id]["overview_widget"].refresh()

        # ---- Update agent MAIN VIEW
        # -> Sort the fleet
        fleet.sort(key=lambda agent: agent.id)

        # -> Construct table to display
        agent_ids = []
        agent_data = []

        for agent in fleet:
            # -> Skip GCS Mission Interfaces
            if agent.name == "GCS Mission Interface":
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
        for agent in fleet:
            # -> Skip GCS Mission Interfaces
            if agent.name == "GCS Mission Interface":
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

    def __update_tasks(self, task: Task, *args, **kwargs) -> None:
        """
        Update the task overview widgets
        """

        task_log = self.task_log.clone()  # > Clone to avoid concurrent access

        # ---- Update mission state overview
        self.__update_mission_state_overview()

        # ---- Update task view OVERVIEW
        for task in task_log:
            if "overview_widget" not in task.local:
                self.__create_task(task)
            else:
                self.interface_widgets["tasks"][task.id]["overview_widget"].refresh()

        # ---- Update task log MAIN VIEW Rendered
        # -> Update base widgets
        # for task in self.task_log:
        #     if "base_widget" in task.local:
        #         task.local["base_widget"].refresh()
        #     else:
        #         self.__create_task(task)

        # ---- Update task log MAIN VIEW Raw
        # -> Sort the task log
        task_log.sort(key=lambda task: task.creation_timestamp, reverse=True)

        # -> Construct table to display
        task_ids = []
        task_data = []

        for task in task_log:
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

        for task in task_log:
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

        fleet = self.fleet.clone()  # > Clone to avoid concurrent access

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
        if "overview_widget" not in self.interface_widgets["agents"][selected_agent_id].keys():
            self.__create_agent(fleet[selected_agent_id])

        widget = self.interface_widgets["agents"][selected_agent_id]["overview_widget"]

        # -> Get current agent interface view
        if self.selected_agent_id is None:
            self.selected_agent_id = selected_agent_id

        self.current_agent_interface_view = self.interface_widgets["agents"][self.selected_agent_id]["overview_widget"].current_interface_view

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

        task_log = self.task_log.clone()  # > Clone to avoid concurrent access

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
        if "overview_widget" not in self.interface_widgets["tasks"][selected_task_id].keys():
            self.__create_task(task_log[selected_task_id])

        widget = self.interface_widgets["tasks"][selected_task_id]["overview_widget"]

        # -> Get current task interface view
        if self.selected_task_id is None:
            self.selected_task_id = selected_task_id

        self.current_task_interface_view = self.interface_widgets["tasks"][self.selected_task_id]["overview_widget"].current_interface_view

        # -> Set current task interface view
        widget.current_interface_view = self.current_task_interface_view

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

    # def __node_spinner(self):
    #     # -> Spin once
    #     self.single_threaded_executor.spin_once(timeout_sec=0)
    #
    #     # rclpy.spin_once(self.ros_node, timeout_sec=0)
    #     # self.__update_env_view()


class ROS_Thread(QThread):
    def __init__(self, ros_node):
        super().__init__()

        # -> Store node
        self.node = ros_node

        # -> Create executor
        self.executor = SingleThreadedExecutor()

        # -> Add node to executor
        self.executor.add_node(self.node)

    def run(self):
        # ----- QT timer based spin
        # self.spin_timer = QtCore.QTimer()
        # self.spin_timer.timeout.connect(self.node_spinner)
        # self.spin_timer.setInterval(1)
        # self.spin_timer.start(0.1)

        while not self.executor._is_shutdown:
            self.executor.spin_once(timeout_sec=0)
            # print("ROS Thread spinning", random.randint(0, 1000))
            time.sleep(0.01)


def main(args=None):
    interface_id = f"gcs_mission_interface_{randint(0, 1000)}"

    # `rclpy` library is initialized
    rclpy.init(args=args)

    # -> Create backend node
    maaf_node = MAAFNode(node_id=interface_id)

    executor = MultiThreadedExecutor()
    executor.add_node(maaf_node)
    thread = Thread(target=executor.spin)
    thread.start()

    app = gcs_mission_interface(interface_id=interface_id)

    # `rclpy` library is shutdown
    maaf_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()