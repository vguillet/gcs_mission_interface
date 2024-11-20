

##################################################################################################################
"""
"""

import os
import sys
from functools import partial
from copy import deepcopy
import time

# Libs
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Signal, QObject, QAbstractTableModel, Qt, QItemSelection, QItemSelectionModel
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QHeaderView, QTableView

# ROS2
import rclpy
from geometry_msgs.msg import Twist, PoseStamped, Point

# Local Imports

try:
    from orchestra_config.orchestra_config import *  # KEEP THIS LINE, DO NOT REMOVE
    from orchestra_config.sim_config import *

    from maaf_tools.datastructures.task.Task import Task
    from maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.datastructures.agent.AgentState import AgentState
    from maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.tools import *

except ImportError:
    from orchestra_config.orchestra_config import *  # KEEP THIS LINE, DO NOT REMOVE
    from orchestra_config.orchestra_config.sim_config import *

    from maaf_tools.maaf_tools.datastructures.task.Task import Task
    from maaf_tools.maaf_tools.datastructures.task.TaskLog import TaskLog

    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.agent.Fleet import Fleet
    from maaf_tools.maaf_tools.datastructures.agent.AgentState import AgentState
    from maaf_tools.maaf_tools.datastructures.agent.Plan import Plan

    from maaf_tools.maaf_tools.tools import *

from .gcs_maaf_node import MAAFNode
from .ui_singleton import UiSingleton

from .Widgets.AgentOverviewWidget import AgentOverviewWidget
from .Widgets.TaskOverviewWidget import TaskOverviewWidget
from .Widgets.BaseTaskWidget import BaseTaskWidget
from .Widgets.GraphEnvView import GraphEnvView

##################################################################################################################


class RefreshTimerEmitter(QObject):
    signal = Signal()

    def __init__(self):
        super().__init__()
        self.signal_timer = QtCore.QTimer()
        self.signal_timer.setInterval(100)
        self.signal_timer.timeout.connect(self.signal.emit)

    def start(self):
        self.signal_timer.start()


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

        self.current_agent_id_selection = None
        self.current_task_id_selection = None

        self.ui.env_widget = None

        # ----- Connect refresh listeners
        self.refresh_flags = {
            "env": False,
            "fleet": False,
            "tasks": False,
        }

        self.refresh_signal_emitter = RefreshTimerEmitter()

        # - Environment listeners
        # Event based
        # self.ros_node.add_on_env_update_listener(partial(self.stage_for_refresh, target="env"))

        # Signal based
        # self.refresh_signal_emitter.signal.connect(self.__update_env_view)

        # - Mission state listeners
        # Event based

        # Signal based
        self.refresh_signal_emitter.signal.connect(self.__update_mission_state_overview)

        # - Fleet listeners
        # Event based
        # self.ros_node.fleet.add_on_add_item_listener(self.__create_agent)
        # self.ros_node.fleet.add_on_edit_list_listener(self.__update_agents)
        self.ros_node.fleet.add_on_edit_list_listener(partial(self.stage_for_refresh, target="fleet"))

        self.refresh_signal_emitter.signal.connect(partial(self.refreshUI, target="fleet"))

        # Signal based
        # self.refresh_signal_emitter.signal.connect(self.__create_agents)
        # self.refresh_signal_emitter.signal.connect(self.__update_agents)

        # - Task log listeners
        # Event based
        # self.ros_node.tasklog.add_on_add_item_listener(self.__create_task)
        # self.ros_node.tasklog.add_on_edit_list_listener(self.update_task_tables)
        self.ros_node.tasklog.add_on_edit_list_listener(partial(self.stage_for_refresh, target="tasks"))

        self.refresh_signal_emitter.signal.connect(partial(self.refreshUI, target="tasks"))

        # Signal based
        # self.refresh_signal_emitter.signal.connect(self.__update_tasks)

        # -> Refresh signal generator
        self.refresh_signal_emitter.start()

        # ----- Connect other signals
        self.ui.pushButton_reset_local_allocation_node.clicked.connect(self.__reset_local_allocation_node)
        self.ui.pushButton_task_overview_view_switcher.clicked.connect(self.__switch_task_overview_view_mode)

        # Toggle switch once to set the initial view and button text
        self.__switch_task_overview_view_mode()

        # # -> Agent selection
        # self.ui.tableView_agents_select.itemSelectionChanged.connect(
        #     partial(self.__select_agent, source=self.ui.tableView_tasks_select)
        # )    # SIDE VIEW
        # self.ui.tableView_agents_overview.itemSelectionChanged.connect(
        #     partial(self.__select_agent, source=self.ui.tableView_agents_overview)
        # )    # MAIN VIEW
        #
        # # -> Task selection
        # # SIDE VIEW
        # self.ui.tableView_tasks_select.setModel(TaskTableModel([], []))
        # self.tableView_tasks_selection_model = self.ui.tableView_tasks_select.selectionModel()
        # # selection_model.selectionChanged.connect(partial(self.__select_task, source=self.ui.tableView_tasks_select))
        # self.tableView_tasks_selection_model.selectionChanged.connect(self.__select_task)
        #
        # # MAIN VIEW
        # self.ui.tableView_tasks_overview.setModel(TaskTableModel([], []))
        # selection_model = self.ui.tableView_tasks_overview.selectionModel()
        # # selection_model.selectionChanged.connect(partial(self.__select_task, source=self.ui.tableView_tasks_overview))
        # selection_model.selectionChanged.connect(self.__select_task)

        # ----------------------------------- Final setup
        # -> QT timer based spin
        self.spin_timer = QtCore.QTimer()
        self.spin_timer.timeout.connect(self.__node_spinner)
        self.spin_timer.setInterval(10)
        self.spin_timer.start()

        # -> Display windows
        self.ui.show()

        # ----------------------------------- Terminate ROS2 node on exit
        # self.ros_node.destroy_node()
        sys.exit(app.exec())

    # ============================================================== REFRESH EVENT LOGIC HANDLER
    def refreshUI(self, target):
        if self.refresh_flags[target] is True:
            if target == "env":
                self.__update_env_view()

            elif target == "fleet":
                self.__create_agents()
                self.__update_agents()

            elif target == "tasks":
                self.__create_tasks()
                self.__update_tasks()

            self.refresh_flags[target] = False

    def stage_for_refresh(self, _, target, *args, **kwargs):
        self.refresh_flags[target] = True

    # ============================================================== METHODS
    # ------------------------------ Format
    def __configure_table_view(self, table_view):
        header = table_view.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setStretchLastSection(True)
        table_view.verticalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        table_view.setSelectionBehavior(QTableView.SelectRows)
        table_view.setSelectionMode(QTableView.SingleSelection)

    # ------------------------------ Setup
    def __reset_local_allocation_node(self, *args, **kwargs):
        # -> Reset allocation states
        self.ros_node.reset_allocation_states()

        # -> Setup interface
        for agent in self.ros_node.fleet:
            agent.local["overview_widget"] = None

        for task in self.ros_node.tasklog:
            task.local["overview_widget"] = None
            task.local["base_widget"] = None

        # -> Setup trackers
        self.selected_agent_id = None
        self.selected_task_id = None

        self.current_agent_id_selection = None
        self.current_task_id_selection = None

    # ------------------------------ Create
    def __create_agents(self, *args, **kwargs) -> None:
        """
        Create all the widgets and subscribers for the agents
        """
        # -> Check that all agents in fleet have a widget. If not, create agent widget
        for agent in self.ros_node.fleet:
            if "overview_widget" not in agent.local:
                self.__create_agent(agent)

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
        self.ros_node.add_team_msg_subscriber_callback_listener(widget.add_to_logs)
        self.refresh_signal_emitter.signal.connect(widget.update)

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

    def __create_tasks(self, *args, **kwargs) -> None:
        """
        Create all the widgets and subscribers for the tasks
        """

        # -> Check that all task in the task log have a widget. If not, create task widget
        for task in self.ros_node.tasklog:
            if "overview_widget" not in task.local:
                self.__create_task(task)

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

        # -> Connect widget listeners
        self.refresh_signal_emitter.signal.connect(overview_widget.update)
        self.refresh_signal_emitter.signal.connect(base_widget.update)

    # ------------------------------ Track
    def __track_current_agent_interface_view(self, current_interface_view: dict) -> None:
        self.current_agent_id_selection = current_interface_view

    def __track_current_task_interface_view(self, current_interface_view: dict) -> None:
        self.current_task_id_selection = current_interface_view

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

        if self.ros_node.environment is None:
            return

        # -> If no environment widget exists, create one
        if self.ui.env_widget is None:
            if self.ros_node.environment["env_type"] == "graph":
                self.ui.env_widget = GraphEnvView(agent_id=self.ros_node.agent.id, ros_node=self.ros_node)
                self.ui.verticalLayout_env_view.addWidget(self.ui.env_widget)

            # TODO: Add support for more environment types

            # -> Add environment widget update listener to environment update listeners
            self.ros_node.add_on_env_update_listener(self.ui.env_widget.update)

        # -> If the wrong environment widget exists, remove it and create the correct one
        elif self.ros_node.environment["env_type"] != self.ui.env_widget.env_type:
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
        self.ui.lcdNumber_tasks_count_total.display(len(self.ros_node.tasklog))
        self.ui.lcdNumber_tasks_count_pending.display(len(self.ros_node.tasklog.ids_pending))
        self.ui.lcdNumber_tasks_count_completed.display(len(self.ros_node.tasklog.ids_completed))
        self.ui.lcdNumber_tasks_count_canceled.display(len(self.ros_node.tasklog.ids_cancelled))

        # ---- Mission progress
        if len(self.ros_node.tasklog) == 0:
            mission_progress = 0
        else:
            mission_progress = (1 - len(self.ros_node.tasklog.ids_pending) / len(self.ros_node.tasklog)) * 100

        self.ui.progressBar_mission_progress.setValue(mission_progress)

    def __update_agents(self, *args, **kwargs) -> None:
        """
        Update the agent overview widgets
        """

        # -> Define column headers
        overview_columns = ["ID", "Name", "Class", "Rank", "Affiliations", "Specs", "Skillset", "Position", "Orientation", "Battery Level", "Stuck", "State Timestamp"]
        select_columns = ["ID", "Name", "Class"]

        # -> Check if fleet is empty
        if not self.ros_node.fleet:
            # -> Create empty models with headers only
            overview_model = AgentTableModel(fleet=None, columns=overview_columns, excluded_agent_id=None)
            select_model = AgentTableModel(fleet=None, columns=select_columns, excluded_agent_id=None)

        else:
            # -> Create data models
            overview_model = AgentTableModel(fleet=self.ros_node.fleet, columns=overview_columns, excluded_agent_id=self.ros_node.id)
            select_model = AgentTableModel(fleet=self.ros_node.fleet, columns=select_columns, excluded_agent_id=self.ros_node.id)

        # -> Set models to table views
        self.ui.tableView_agents_overview.setModel(overview_model)
        self.ui.tableView_agents_select.setModel(select_model)

        # -> Configure table views for better appearance
        self.__configure_table_view(self.ui.tableView_agents_overview)
        self.__configure_table_view(self.ui.tableView_agents_select)

        # -> Reconnect selection models
        self.ui.tableView_agents_overview.selectionModel().selectionChanged.connect(
            partial(self.__select_agent, source="tableView_agents_overview")
        )
        self.ui.tableView_agents_select.selectionModel().selectionChanged.connect(
            partial(self.__select_agent, source="tableView_agents_select")
        )

        # -> Reapply the shared selection based on the ID
        if self.current_agent_id_selection is not None:
            row_index = overview_model.get_row_by_agent_id(self.current_agent_id_selection)
            self.ui.tableView_agents_overview.selectRow(row_index)

            row_index = select_model.get_row_by_agent_id(self.current_agent_id_selection)
            self.ui.tableView_agents_select.selectRow(row_index)

        else:
            # -> Select first agent if no agent is selected
            self.ui.tableView_agents_select.selectRow(0)

    def __update_tasks(self, *args, **kwargs) -> None:
        """
        Update the task overview widgets
        """

        start_time = time.time()

        # -> Define column headers
        overview_columns = ["ID", "Type", "Priority", "Affiliations", "Creator", "Creation Timestamp", "Termination Timestamp", "Status", "Termination Source"]
        select_columns = ["ID", "Type", "Priority", "Status"]

        # -> Check if task log is empty
        if not self.ros_node.tasklog:
            # -> Create empty models with headers only
            overview_model = TaskTableModel(tasklog=None, columns=overview_columns)
            select_model = TaskTableModel(tasklog=None, columns=select_columns)

        else:
            # -> Create data models
            overview_model = TaskTableModel(tasklog=self.ros_node.tasklog, columns=overview_columns)
            select_model = TaskTableModel(tasklog=self.ros_node.tasklog, columns=select_columns)

        print(f"Model creation time: {time.time() - start_time}")
        time_1 = time.time()

        # -> Set models to table views
        self.ui.tableView_tasks_overview.setModel(overview_model)
        self.ui.tableView_tasks_select.setModel(select_model)

        print(f"Model set time: {time.time() - time_1}")
        time_2 = time.time()

        # -> Configure table views for better appearance
        self.__configure_table_view(self.ui.tableView_tasks_overview)
        self.__configure_table_view(self.ui.tableView_tasks_select)

        # -> Reconnect selection models
        self.ui.tableView_tasks_overview.selectionModel().selectionChanged.connect(
            partial(self.__select_task, source="tableView_tasks_overview")
        )
        self.ui.tableView_tasks_select.selectionModel().selectionChanged.connect(
            partial(self.__select_task, source="tableView_tasks_select")
        )

        print(f"Table configuration time: {time.time() - time_2}")
        time_3 = time.time()

        # -> Reapply the shared selection based on the ID
        if self.current_task_id_selection is not None:
            row_index = overview_model.get_row_by_task_id(self.current_task_id_selection)

            time_3_2 = time.time()
            # Only select if it's not already selected
            self.ui.tableView_tasks_overview.selectRow(row_index)
            print(f"    Row selection time: {time.time() - time_3_2}")

            row_index = select_model.get_row_by_task_id(self.current_task_id_selection)
            self.ui.tableView_tasks_select.selectRow(row_index)

        else:
            # -> Select first task if no task is selected
            self.ui.tableView_tasks_select.setCurrentIndex(0)

        print(f"Task selection time: {time.time() - time_3}")

        print(f"Task update time: {time.time() - start_time}\n")

    # ------------------------------ Select
    def __select_agent(self, selected, deselected, agent_id=None, source=None) -> None:
        """
        Select an agent from the agent overview table
        """

        if source is None:
            pass

        if selected.indexes():
            row_index = selected.indexes()[0].row()
            column_index = selected.indexes()[0].column()

            # ---- Get ID from selected
            if source == "tableView_agents_overview":
                # -> Get new selection
                self.current_agent_id_selection = (
                    self.ui.tableView_agents_overview.model().data(self.ui.tableView_agents_overview.model().index(row_index, 0)))

            elif source == "tableView_agents_select":
                # -> Get new selection
                self.current_agent_id_selection = (
                    self.ui.tableView_agents_select.model().data(self.ui.tableView_agents_select.model().index(row_index, 0)))

                # -> Apply selection to the main view
                self.ui.tableView_agents_overview.selectRow(row_index)

            # -> Get the agent overview widget
            widget = self.ros_node.fleet[self.current_agent_id_selection].local["overview_widget"]

            # -> Switch stacked widget to the agent overview
            self.ui.stackedWidget_agents_overviews.setCurrentWidget(widget)

    def __select_task(self, selected, deselected, task_id=None, source=None):

        if source is None:
            pass

        if selected.indexes():
            row_index = selected.indexes()[0].row()
            column_index = selected.indexes()[0].column()

            # ---- Get ID from selected
            if source == "tableView_tasks_overview":
                # -> Get new selection
                self.current_task_id_selection = (
                    self.ui.tableView_tasks_overview.model().data(self.ui.tableView_tasks_overview.model().index(row_index, 0)))

                # -> Apply selection to the side view
                self.ui.tableView_tasks_select.selectRow(row_index)

            elif source == "tableView_tasks_select":
                # -> Get new selection
                self.current_task_id_selection = (
                    self.ui.tableView_tasks_select.model().data(self.ui.tableView_tasks_select.model().index(row_index, 0)))

                # -> Apply selection to the main view
                self.ui.tableView_tasks_overview.selectRow(row_index)

            # -> Get the task overview widget
            widget = self.ros_node.tasklog[self.current_task_id_selection].local["overview_widget"]

            # -> Switch stacked widget to the task overview
            self.ui.stackedWidget_tasks_overviews.setCurrentWidget(widget)

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


class AgentTableModel(QAbstractTableModel):
    def __init__(self, fleet=None, columns=None, excluded_agent_id=None):
        super().__init__()
        self.fleet = fleet if fleet is not None else []
        self.columns = columns if columns is not None else []
        self.excluded_agent_id = excluded_agent_id  # ID of the agent to exclude (can be None)

        # Create a dictionary that maps task IDs to row indices for efficient lookup
        self.agent_id_index_map = {agent.id: index for index, agent in enumerate(self.fleet)}

    def rowCount(self, parent=None):
        # Count rows excluding the agent with the specified ID, only if excluded_agent_id is provided
        if self.excluded_agent_id is not None:
            return len([agent for agent in self.fleet if agent.id != self.excluded_agent_id])
        return len(self.fleet)  # No exclusion if excluded_agent_id is None

    def columnCount(self, parent=None):
        return len(self.columns)

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid() or role != Qt.DisplayRole:
            return None

        # Find the agent to display (skipping the excluded agent, only if excluded_agent_id is provided)
        if self.excluded_agent_id is not None:
            filtered_agents = [agent for agent in self.fleet if agent.id != self.excluded_agent_id]
        else:
            filtered_agents = self.fleet

        if index.row() >= len(filtered_agents) or index.column() >= len(self.columns):
            return None  # Handle out-of-range access safely

        agent = filtered_agents[index.row()]
        column = self.columns[index.column()]

        if agent is None:
            return None

        # Map column names to agent attributes or derived values
        if column == "ID":
            return agent.id
        elif column == "Name":
            return agent.name
        elif column == "Class":
            return agent.agent_class
        elif column == "Rank":
            return agent.hierarchy_level
        elif column == "Affiliations":
            return ", ".join(agent.affiliations) if agent.affiliations else "N/A"
        elif column == "Specs":
            return ", ".join(agent.specs) if agent.specs else "N/A"
        elif column == "Skillset":
            return ", ".join(agent.skillset) if agent.skillset else "N/A"
        elif column == "Position":
            return f"({agent.state.x}, {agent.state.y}, {agent.state.z})"
        elif column == "Orientation":
            return f"({agent.state.u}, {agent.state.v}, {agent.state.w})"
        elif column == "Battery Level":
            return agent.state.battery_level
        elif column == "Stuck":
            return agent.state.stuck
        elif column == "State Timestamp":
            return pd.to_datetime(agent.state.timestamp, unit="s").replace(microsecond=0, nanosecond=0).strftime('%Y-%m-%d %H:%M:%S')
        return None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role != Qt.DisplayRole:
            return None

        if orientation == Qt.Horizontal:
            return self.columns[section] if section < len(self.columns) else None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role != Qt.DisplayRole:
            return None

        if orientation == Qt.Horizontal:
            return self.columns[section] if section < len(self.columns) else None

    def get_row_by_agent_id(self, agent_id):
        """Retrieve the row index based on agent ID."""
        return self.agent_id_index_map.get(agent_id, -1)


class TaskTableModel(QAbstractTableModel):
    def __init__(self, tasklog=None, columns=None):
        super().__init__()
        self.tasklog = tasklog if tasklog is not None else []
        self.columns = columns if columns is not None else []

        # Create a dictionary that maps task IDs to row indices for efficient lookup
        self.task_id_index_map = {task.id: index for index, task in enumerate(self.tasklog)}

    def rowCount(self, parent=None):
        return len(self.tasklog)

    def columnCount(self, parent=None):
        return len(self.columns)

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid() or role != Qt.DisplayRole:
            return None

        if index.row() >= len(self.tasklog) or index.column() >= len(self.columns):
            return None  # Handle out-of-range access safely

        task = self.tasklog.get_by_index(index=index.row())
        column = self.columns[index.column()]

        if task is None:
            return None

        # Map column names to task attributes or derived values
        if column == "ID":
            return task.id
        elif column == "Type":
            return task.type
        elif column == "Priority":
            return task.priority
        elif column == "Affiliations":
            return ", ".join(task.affiliations) if task.affiliations else "N/A"
        elif column == "Creator":
            return task.creator
        elif column == "Creation Timestamp":
            return pd.to_datetime(task.creation_timestamp, unit="s").replace(microsecond=0, nanosecond=0).strftime(
                '%Y-%m-%d %H:%M:%S')
        elif column == "Termination Timestamp":
            if task.termination_timestamp is not None:
                return pd.to_datetime(task.termination_timestamp, unit="s").replace(microsecond=0,
                                                                                    nanosecond=0).strftime(
                    '%Y-%m-%d %H:%M:%S')
            return "N/A"
        elif column == "Status":
            return task.status.capitalize()
        elif column == "Termination Source":
            return task.termination_source_id if task.termination_source_id is not None else "N/A"
        return None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role != Qt.DisplayRole:
            return None

        if orientation == Qt.Horizontal:
            return self.columns[section] if section < len(self.columns) else None

        elif orientation == Qt.Vertical:
            # Default to 1-based indexing for row headers
            return str(section + 1)

        return None

    def get_row_by_task_id(self, task_id):
        """Retrieve the row index based on task ID."""
        # row_index = self.task_id_index_map.get(task_id, -1)
        # if row_index == -1:
        #     return None  # Return None if no task ID is found, indicating invalid index
        # return self.model().index(row_index, 0)  # Create a QModelIndex (use column 0 for now)

        return self.task_id_index_map.get(task_id, -1)



def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    app = gcs_mission_interface()

    rclpy.shutdown()


if __name__ == '__main__':
    main()