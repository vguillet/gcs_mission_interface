

##################################################################################################################
"""
"""
import datetime
import os
import sys
from typing import List, Optional
from json import loads, dumps
from copy import deepcopy, copy
from functools import partial
from threading import Thread
from pprint import pprint, pformat
import time

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
from .ui_singleton import UiSingleton, NucleusSingleton

from .Widgets.AgentOverviewWidget import AgentOverviewWidget
from .Widgets.TaskOverviewWidget import TaskOverviewWidget
from .Widgets.BaseTaskWidget import BaseTaskWidget
from .Widgets.GraphEnvView import GraphEnvView
# from.tools.Event_bus import EventBus

##################################################################################################################


class GCSNode(Node):
    def __init__(self, interface_id: str):
        super().__init__(interface_id + "_receiver")

        # -> Load ui singleton
        self.eventbus = EventBus()
        self.nucleus = NucleusSingleton()

        # > Create shallow copy to avoid concurrent access
        # self.env = copy(self.nucleus.env)
        self.fleet = copy(self.nucleus.fleet)
        self.task_log = copy(self.nucleus.task_log)

        # -> Create subscriber to update
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.update_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic="gcs_mission_interface/update",
            qos_profile=qos,
            callback=self.update_callback,
        )

        self.run_stats = {
            "format": [],
            "update": [],
            "topics": {},
        }

        self.prev_timestamp = 0

        self.first = True

    def update_callback(self, msg: TeamCommStamped) -> None:
        start_time = time.time()

        update = msg.memo

        update = loads(update)

        if update["timestamp"] <= self.prev_timestamp:
            return
        else:
            self.prev_timestamp = update["timestamp"]

        topics = update["topics"]
        state = update["state"]
        data = update["data"]

        t0 = time.time()

        # -> Convert update to fleet and task log objects
        state["fleet"] = Fleet.from_dict(state["fleet"])
        state["task_log"] = TaskLog.from_dict(state["task_log"])

        t1 = time.time()

        for agent in state["fleet"]:
            if "allocation_state" in agent.shared:
                agent.shared["allocation_state"] = loads(agent.shared["allocation_state"])

                for key in agent.shared["allocation_state"].keys():
                    if key == "task_log":
                        continue
                        agent.shared["allocation_state"][key] = TaskLog.from_dict(agent.shared["allocation_state"][key])
                    elif key == "fleet":
                        continue
                        agent.shared["allocation_state"][key] = Fleet.from_dict(agent.shared["allocation_state"][key])
                    else:
                        agent.shared["allocation_state"][key] = pd.DataFrame(agent.shared["allocation_state"][key])

        t2 = time.time()

        # fleet = self.fleet.clone()
        # task_log = self.task_log.clone()

        # -> Merge updates in local state
        self.fleet.merge(state["fleet"], prioritise_local=False)
        self.task_log.merge(state["task_log"], prioritise_local=False)

        t3 = time.time()

        print(f"Time: {round(t1 - t0, 4)} - {round(t2 - t1, 4)} - {round(t3 - t2, 4)}")

        # -> Convert data to correct type
        if "env_update" in topics:
            data = json_to_graph(graph_json=data)
            self.nucleus.env = data
            # self.nucleus.update_env_view(env=data)

        # elif "fleet" in topics:
        #     data = fleet[data["id"]]
        #
        # elif "task_log" in topics:
        #     data = task_log[data["id"]]
        #
        format_time = time.time()
        #
        # if "env_update" in topics and self.first:
        #     self.eventbus.publish(
        #         topic=["ros_node", "env_update"],
        #         data=data
        #     )
        #     self.first = False

        # elif topics == ["all"]:
        #     self.nucleus.update_all()
        #
        # elif "env_update" not in topics:
        #     self.eventbus.publish(
        #         topic=["ros_node"] + topics,
        #         data=data
        #     )

        # print(f"Update {data}: {topics} - Format: {round(format_time - start_time, 4)} - Update: {round(time.time() - format_time, 4)}")
        print(f"Update: {topics} - Format: {round(format_time - start_time, 4)} - Update: {round(time.time() - format_time, 4)}")

        # if topics != ["all"]:
        #     self.nucleus.event_bus.publish(
        #         topic=["ros_node"] + topics,
        #         data=data
        #     )
        # else:
        #     self.nucleus.update_all()

        # if "env_update" in topics and self.first:
        #     self.nucleus.event_bus.publish(
        #         topic=["ros_node", "env_update"],
        #         data=data
        #     )
        #     self.first = False

        # print(f"Update: {topics} - Format: {round(format_time, 4)} - Update: {round(time.time() - format_time, 4)}")

        # ================================================================ DEBUG
        update_time = time.time() - start_time

        if str(topics) not in self.run_stats["topics"]:
            self.run_stats["topics"][str(topics)] = {
                "format": [],
                "update": [],
            }

        self.run_stats["topics"][str(topics)]["format"].append(format_time)
        self.run_stats["topics"][str(topics)]["update"].append(update_time)

        # -> Compute average
        self.run_stats["topics"][str(topics)]["format_avg"] = np.mean(self.run_stats["topics"][str(topics)]["format"])
        self.run_stats["topics"][str(topics)]["update_avg"] = np.mean(self.run_stats["topics"][str(topics)]["update"])

        # -> Compute cumulated
        self.run_stats["topics"][str(topics)]["format_cum"] = np.sum(self.run_stats["topics"][str(topics)]["format"])
        self.run_stats["topics"][str(topics)]["update_cum"] = np.sum(self.run_stats["topics"][str(topics)]["update"])

        # self.get_logger().info(f"Update: {topics} - Format: {round(format_time, 4)} - Update: {round(update_time, 4)}")
        # print(f"Update: {topics} - Format: {round(format_time, 4)} - Update: {round(update_time, 4)}")
        #
        # self.get_logger().info("")
        #
        # for topic, data in self.run_stats["topics"].items():
        #     # self.get_logger().info(f"Topic: {topic} - Format time: {round(data['format_avg'], 4)} - Update time: {round(data['update_avg'], 4)}")
        #     self.get_logger().info(f"Topic: {topic} - Format time: {round(data['format_cum'], 4)} - Update time: {round(data['update_cum'], 4)}")
        #     # print(f"Topic: {topic} - Format time: {round(data['format_cum'], 4)} - Update time: {round(data['update_cum'], 4)}")
