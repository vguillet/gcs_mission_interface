

##################################################################################################################
"""
"""

import sys
import os
from typing import List, Optional, Tuple
from json import loads, dumps
from copy import deepcopy
from datetime import datetime
from pprint import pprint

# Libs
import numpy as np
import pandas as pd
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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np

# Local Imports
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from maaf_allocation_node.maaf_agent import MAAFAgent
from maaf_allocation_node.node_config import *
from maaf_allocation_node.task_dataclasses import Task, Task_log
from maaf_allocation_node.fleet_dataclasses import Agent, Fleet
from maaf_allocation_node.state_dataclasses import Agent_state
from maaf_msgs.msg import TeamCommStamped, Bid, Allocation

from .ui_singleton import UiSingleton

##################################################################################################################


class MAAFNode(MAAFAgent):
    def __init__(self):
        # ---- Init parent class
        MAAFAgent.__init__(
            self,
            node_name="gcs_mission_interface",
            id="gcs_mission_interface",
            name="GCS Mission Interface",
            skillset=["INTERFACE"],
            bid_estimator=None
        )

        # -> Override node fleets_msgs subscriber with one with a best effort qos
        qos_override = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3
        )

        if RUN_MODE == OPERATIONAL:
            # ---------- fleet_msgs
            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=topic_fleet_msgs,
                callback=self.team_msg_subscriber_callback,
                qos_profile=qos_override
            )

        elif RUN_MODE == SIM:
            # ---------- fleet_msgs_filtered
            self.fleet_msgs_sub = self.create_subscription(
                msg_type=TeamCommStamped,
                topic=topic_fleet_msgs_filtered,
                callback=self.team_msg_subscriber_callback,
                qos_profile=qos_override
            )

        # -----------------------------------  Agent allocation states
        # -> Setup additional CBAA-specific allocation states
        self.__setup_allocation_additional_states()

        # -> Setup team message subscriber callback listeners
        self.__team_msg_subscriber_callback_listeners = []

        # -> Initialise previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

# ============================================================== INIT
    def __setup_allocation_additional_states(self) -> None:
        """
        Setup additional method-dependent allocation states for the agents
        """
        pass

    # ============================================================== PROPERTIES
    # ---------------- Self state
    @property
    def local_allocation_state(self) -> dict:
        """
        Get the local allocation state dictionary
        """
        return {}

    # ============================================================== METHODS
    # ---------------- Callbacks
    # >>>> Base
    def task_msg_subscriber_callback(self, task_msg):
        """
        Callback for task messages: create new task, add to local tasks and update local states

        :param task_msg: TaskMsgStamped message
        """

        # -> Check if the message is for the agent
        msg_target = task_msg.target

        # -> If the message is not for the agent...
        if msg_target != self.id and msg_target != "all":
            return

        # -> Create new task
        if task_msg.meta_action == "add":
            # -> Unpack msg
            task_dict = loads(task_msg.memo)  # Task from task factory

            # -> Ensure id is a string
            task_dict["id"] = str(task_dict["id"])

            # TODO: Remove this line once task creation handles stamp creation
            task_dict["creation_timestamp"] = self.current_timestamp

            # -> Create task object
            task = Task.from_dict(task_dict)

            # -> Update state awareness
            self.__update_situation_awareness(task_list=[task], fleet=None)

        # -> Update previous state hash
        self.prev_allocation_state_hash_dict = deepcopy(self.allocation_state_hash_dict)

        # -> Publish allocation state to the fleet to share the new task
        self.publish_allocation_state_msg()

    def team_msg_subscriber_callback(self, team_msg):
        """
        Callback for team messages: consensus phase of the CBAA algorithm

        :param team_msg: TeamComm message
        """
        # ---- Unpack msg
        # -> Ignore self messages
        if team_msg.source == self.id:
            return

        # -> Check if the message is for the agent
        msg_target = team_msg.target

        # -> If the message is not for the agent...
        if msg_target != self.id and msg_target != "all":
            # -> Check if the agent should rebroadcast the message
            msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)
            return

        # -> Deserialise allocation state
        received_allocation_state = self.deserialise(state=team_msg.memo)

        # -> Update local situation awareness
        # > Convert task list to Task objects
        received_task_log = [Task.from_dict(task) for task in received_allocation_state["tasks"]]

        # > Convert fleet to Agent objects
        received_fleet = [Agent.from_dict(agent) for agent in received_allocation_state["fleet"]]

        self.__update_situation_awareness(
            task_list=received_task_log,
            fleet=received_fleet,
            received_allocation_state=received_allocation_state
        )

        # > Find source agent in received fleet
        source_agent = [agent for agent in received_fleet if agent.id == team_msg.source][0]

        self.__log_received_allocation_states(
            agent=source_agent,
            received_allocation_state=received_allocation_state,
        )

        # -> Update shared states
        self.__update_shared_states(
            received_shared_bids_b=received_allocation_state["shared_bids_b"],
            received_shared_bids_priority_beta=received_allocation_state["shared_bids_priority_beta"],
            received_shared_allocations_a=received_allocation_state["shared_allocations_a"],
            received_shared_allocations_priority_alpha=received_allocation_state["shared_allocations_priority_alpha"]
        )

        # -> If state has changed, update local states (only publish when necessary)
        if self.allocation_state_change:
            self.check_publish_state_change()

        else:
            # -> Check if the agent should rebroadcast the message
            msg, rebroadcast = self.rebroadcast(msg=team_msg, publisher=self.fleet_msgs_pub)

        # -> Call listeners
        # > Convert ros time stamp to human readable format
        timestamp = team_msg.stamp.nanosec / 1e9 + team_msg.stamp.sec
        timestamp = datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")

        msg = {
            # Get msg stamp
            "timestamp": timestamp,
            "source": team_msg.source,
            "target": team_msg.target,
            "meta_action": team_msg.meta_action,
            "memo": {
                "tasks": received_task_log,
                "fleet": received_fleet,
                "allocation_state": received_allocation_state
            }
        }
        for listener in self.__team_msg_subscriber_callback_listeners:
            listener(msg)

    def add_team_msg_subscriber_callback_listener(self, listener):
        self.__team_msg_subscriber_callback_listeners.append(listener)

    # >>>> GCS mission interface
    def __update_situation_awareness(
            self,
            task_list: Optional[List[Task]],
            fleet: Optional[List[Agent]],
            received_allocation_state: dict
        ) -> None:
        """
        Update local states with new tasks and fleet dicts. Add new tasks and agents to local states and extend
        local states with new rows and columns for new tasks and agents

        :param task_list: Tasks dict
        :param fleet: Fleet dict
        """

        def add_agent(agent: Agent) -> None:
            """
            Add new agent to local fleet and extend local states with new columns for new agent
            """
            # -> Add new agent to local fleet
            self.fleet.add_agent(agent=agent)

            # -> Add a column to all relevant allocation lists and matrices with new agent
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove winning_bids_y from the state (not needed for this operation)
            state.pop("winning_bids_y")

            for matrix in state.values():
                matrix[agent.id] = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)

        def remove_agent(agent: Agent) -> None:
            """
            Remove agent from local fleet and remove all relevant allocation lists and matrices columns
            """
            # -> Remove agent from local fleet
            # > Flag agent is inactive
            self.fleet.set_agent_state(agent=agent, state=agent.state)

            # TODO: Figure out how to deal with removing bids from the winning bid matrix (full reset vs bid owner tracking). For now, reset winning bids matrix (full reset)
            self.winning_bids_y = pd.Series(np.zeros(self.Task_count_N_t), index=self.task_log.ids_pending)

            # -> Remove agent from all allocation lists and matrices
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            for matrix in state.values():
                try:
                    matrix.drop(columns=agent.id, inplace=True)
                except KeyError:
                    pass

        def add_task(task: Task) -> None:
            """
            Add new task to local tasks and extend local states with new rows for new task
            """
            # -> Add new task to local tasks
            self.task_log.add_task(task=task)

            # -> Add a row to all allocation lists and matrices with new task
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            # > Remove winning_bids_y from the state (operation performed separately)
            state.pop("winning_bids_y")
            self.winning_bids_y.loc[task.id] = 0

            for matrix in state.values():
                matrix.loc[task.id] = pd.Series(np.zeros(self.Agent_count_N_u), index=self.fleet.ids_active)

        def terminate_task(task: Task) -> None:
            """
            Flag task as terminated in task log and remove all relevant allocation lists and matrices rows
            """
            # -> Update task log
            if task.status == "completed":
                self.task_log.flag_task_completed(
                    task=task,
                    termination_source_id=task.termination_source_id,
                    termination_timestamp=task.termination_timestamp
                )

            elif task.status == "cancelled":
                self.task_log.flag_task_cancelled(
                    task=task,
                    termination_source_id=task.termination_source_id,
                    termination_timestamp=task.termination_timestamp
                )

            # -> Remove task from all allocation lists and matrices
            state = self.get_state(
                state_awareness=False,
                local_allocation_state=True,
                shared_allocation_state=True,
                serialised=False
            )

            for matrix in state.values():
                try:
                    matrix.drop(index=task.id, inplace=True)
                except KeyError:
                    pass

        # ---- Add new tasks and agents
        # -> Update local fleet
        if fleet is not None:
            for agent in fleet:
                # -> If the agent is not in the local fleet ...
                if agent.id not in self.fleet.ids:
                    # -> If the agent is active, add to fleet and extend local states with new columns
                    if agent.state.status == "active":
                        add_agent(agent=agent)

                    # -> If the agent is inactive, only add to the local fleet
                    else:
                        self.fleet.add_agent(agent=agent)

                # -> Else if the new agent state is more recent than the local agent state, update
                elif agent.state.timestamp > self.fleet[agent.id].state.timestamp:
                    # -> If the agent is active, update the agent state in the fleet to the latest state
                    if agent.state.status == "active":
                        self.fleet.set_agent_state(agent=agent, state=agent.state)

                    # -> If the agent is inactive, update state and remove agent from local states
                    else:
                        remove_agent(agent=agent)

        # -> Update task log and local states
        if task_list is not None:
            for task in task_list:
                # -> If the task is not in the task log ...
                if task.id not in self.task_log.ids:
                    # -> If the task is pending, add to task log and extend local states with new rows
                    if task.status == "pending":
                        add_task(task=task)

                    # -> If the task is completed, only add to the task log
                    else:
                        self.task_log.add_task(task=task)

                # -> Else if the task is in the task log and is not pending, flag as terminated in task log and remove rows from the local states
                elif task.status != "pending" and self.task_log[task.id].status == "pending":
                    terminate_task(task=task)

    def __update_shared_states(
            self,
            received_shared_bids_b,
            received_shared_bids_priority_beta,
            received_shared_allocations_a,
            received_shared_allocations_priority_alpha
    ):
        """
        Update local states with received states from the fleet

        :param received_shared_bids_b: Task bids matrix b received from the fleet
        :param received_shared_bids_priority_beta: Task bids priority matrix beta received from the fleet
        :param received_shared_allocations_a: Task allocations matrix a received from the fleet
        :param received_shared_allocations_priority_alpha: Task allocations priority matrix alpha received from the fleet
        """

        received_tasks_ids = list(received_shared_bids_b.index)
        received_agent_ids = list(received_shared_bids_b.columns)

        # -> For each task ...
        for task_id in received_tasks_ids:
            if task_id not in self.shared_bids_b.index:
                continue

            # -> for each agent ...
            for agent_id in received_agent_ids:
                if agent_id not in self.shared_bids_b.columns:
                    continue

                # -> Priority merge with reset received current bids b into local current bids b
                # > Determine correct matrix values
                shared_bids_b_ij_updated, shared_bids_priority_beta_ij_updated = (
                    self.CBAA_priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.shared_bids_b.loc[task_id, agent_id],
                        matrix_source_ij=received_shared_bids_b.loc[task_id, agent_id],
                        priority_updated_ij=self.shared_bids_priority_beta.loc[task_id, agent_id],
                        priority_source_ij=received_shared_bids_priority_beta.loc[task_id, agent_id]
                    )
                )

                # > Update local states
                self.shared_bids_b.loc[task_id, agent_id] = shared_bids_b_ij_updated
                self.shared_bids_priority_beta.loc[task_id, agent_id] = shared_bids_priority_beta_ij_updated

                # -> Priority merge received current allocations a into local current allocations a
                # > Determine correct matrix values
                shared_allocations_a_ij_updated, shared_allocations_priority_alpha_ij_updated = (
                    self.CBAA_priority_merge(
                        # Logging
                        task_id=task_id,
                        agent_id=agent_id,

                        # Merging
                        matrix_updated_ij=self.shared_allocations_a.loc[task_id, agent_id],
                        matrix_source_ij=received_shared_allocations_a.loc[task_id, agent_id],
                        priority_updated_ij=self.shared_allocations_priority_alpha.loc[task_id, agent_id],
                        priority_source_ij=received_shared_allocations_priority_alpha.loc[task_id, agent_id]
                    )
                )

                # > Update local states
                self.shared_allocations_a.loc[task_id, agent_id] = shared_allocations_a_ij_updated
                self.shared_allocations_priority_alpha.loc[task_id, agent_id] = shared_allocations_priority_alpha_ij_updated

    def __log_received_allocation_states(
            self,
            agent,
            received_allocation_state: dict,
        ) -> None:
        """
        Log the received allocation states to the agent's local field
        """

        if agent.state.timestamp >= self.fleet[agent.id].state.timestamp:
            # -> Sort dataframe columns and index
            for allocation_state in received_allocation_state.values():
                if isinstance(allocation_state, pd.DataFrame):
                    allocation_state.sort_index(axis=0, inplace=True)
                    allocation_state.sort_index(axis=1, inplace=True)

            # -> Update local data fields
            self.fleet.update_item_fields(
                item=agent,
                field_value_pair={"allocation_state": received_allocation_state},
                add_to_local=True
            )

    # ---------------- tools
    # >>>> gcs mission interface
    def print_state(
            self,
            situation_awareness: bool = False,
            local_allocation_state: bool = False,
            shared_allocation_state: bool = False
    ):
        """
        Print the state of the agent

        :param situation_awareness: Flag to print situation awareness
        :param local_allocation_state: Flag to print local allocation state
        :param shared_allocation_state: Flag to print shared allocation state

        :return: str
        """

        print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Agent {self.id} state:")
        if situation_awareness:
            print("----- Situation awareness")
            pprint(self.task_log.to_list())
            pprint(self.fleet.to_list())

        if local_allocation_state:
            print("\nLocal bids c:")
            print(self.local_bids_c)

            print("\nLocal allocations d:")
            print(self.local_allocations_d)

        if shared_allocation_state:
            print("----- Shared allocation state")
            print("Winning bids y:")
            print(self.winning_bids_y)

            print("\n------------")
            print("Current bids b:")
            print(self.shared_bids_b)

            # print("\nCurrent bids priority beta:")
            # print(self.shared_bids_priority_beta)
            #
            # print("\n------------")
            # print("Current allocations a:")
            # print(self.shared_allocations_a)
            #
            # print("\nCurrent allocations priority alpha:")
            # print(self.shared_allocations_priority_alpha)

        print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")

    def priority_merge(
            self,
            task_id: str,
            agent_id: str,

            # -> Updated matrix
            # > Updated matrix value
            matrix_updated: pd.DataFrame,
            matrix_updated_ij: float,

            # > Updated priority value
            matrix_priority_updated: pd.DataFrame,
            priority_updated_ij: float,

            # -> Source matrix
            # > Source matrix value
            matrix_source: Optional[pd.DataFrame],
            matrix_source_ij: float,

            # > Source priority value
            matrix_priority_source: Optional[pd.DataFrame],
            priority_source_ij: float,

            reset: bool = False
    ) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """
        Wrapper for CBAA_priority_merge to be used by the agent class

        :param task_id: Task id
        :param agent_id: Agent id
        :param matrix_updated: Updated matrix
        :param matrix_updated_ij: Updated matrix value
        :param matrix_priority_updated: Updated priority value
        :param priority_updated_ij: Updated priority value
        :param matrix_source: Source matrix
        :param matrix_source_ij: Source matrix value
        :param matrix_priority_source: Source priority value
        :param priority_source_ij: Source priority value
        :param reset: Flag to reset task value to zero if source priority is higher

        :return: Updated matrix, updated priority
        """

        # -> Priority merge received bid into current bids b
        matrix_updated_ij, priority_updated_ij = self.CBAA_priority_merge(
                # Logging
                task_id=task_id,
                agent_id=agent_id,

                # Merging
                matrix_updated_ij=matrix_updated_ij,
                priority_updated_ij=priority_updated_ij,

                matrix_source_ij=matrix_source_ij,
                priority_source_ij=priority_source_ij
            )

        # -> Update local states
        matrix_updated.loc[task_id, agent_id] = matrix_updated_ij
        matrix_priority_updated.loc[task_id, agent_id] = priority_updated_ij

        return matrix_updated, matrix_priority_updated

    def CBAA_priority_merge(
            self,
            task_id: str,
            agent_id: str,

            matrix_updated_ij: float,
            priority_updated_ij: float,

            matrix_source_ij: float,
            priority_source_ij: float,
            ):
        """
        Merge two matrices values based on priority values. If source priority is higher, update the updated matrix value with the
        source matrix. If updated priority is higher, do nothing. If priorities are equal, apply other tie-breakers.

        Option to reset task value to zero if source priority is higher. If using reset, the tasks_value_x_ij must be
        provided.

        ### For logging purposes
        :param task_id: Task id
        :param agent_id: Agent id

        ### Merging variables
        :param matrix_updated_ij: Updated matrix value
        :param matrix_source_ij: Source matrix value to compare with updated matrix value
        :param priority_updated_ij: Updated priority value
        :param priority_source_ij: Source priority value used to compare source matrix value with updated priority value

        :return: updated matrix value, updated priority value
        """

        # -> If source priority is higher
        if priority_source_ij > priority_updated_ij:

            # -> Update matrix value with source matrix value
            matrix_updated_ij = matrix_source_ij

            # -> Update priority value with source priority value
            priority_updated_ij = priority_source_ij

        # -> If updated priority is higher
        elif priority_source_ij < priority_updated_ij:
            # -> Do nothing as updated priority is higher, therefore keep updated matrix value and priority value
            pass

        # -> If priorities are equal
        else:
            # Apply other tie-breakers
            # TODO: Implement tie-breakers, for now larger value is kept
            matrix_updated_ij = max(matrix_updated_ij, matrix_source_ij)

        return matrix_updated_ij, priority_updated_ij
