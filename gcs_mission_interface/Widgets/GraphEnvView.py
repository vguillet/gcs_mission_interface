
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os
from typing import Optional
from pprint import pprint, pformat
import time
from copy import deepcopy, copy

# Libs
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
from pandas import to_datetime
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWidgets import QVBoxLayout, QWidget
import numpy as np

import pyqtgraph as pg
from pyqtgraph import mkPen, mkBrush

from PySide6.QtGui import QColor

# import matplotlib
# matplotlib.use('tkagg')
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
# from matplotlib.figure import Figure
# from matplotlib.animation import FuncAnimation

# Own modules
from orchestra_config.sim_config import *
from ..tools.BlitManager import BlitManager
from ..ui_loader import uic
from ..ui_singleton import NucleusSingleton

##################################################################################################################


# class MplCanvas(FigureCanvasQTAgg):
#     def __init__(self, parent=None, width=5, height=4, dpi=100):
#         self.fig = Figure(figsize=(width, height), dpi=dpi)
#         self.axes = self.fig.add_subplot(111)
#         super(MplCanvas, self).__init__(self.fig)
#
#         self.fig.subplots_adjust(left=0.001, right=0.999, bottom=0.001, top=0.999)


class GraphEnvView(QWidget):
    def __init__(self):
        super().__init__()

        self.env_type = "graph"

        # ----------------------------------- Load GUI
        # -> Load main singleton
        self.nucleus = NucleusSingleton()

        # self.env = copy(self.nucleus.env)
        self.fleet = copy(self.nucleus.fleet)
        self.task_log = copy(self.nucleus.task_log)

        # -> Widget settings
        self.show_goals = True

        # -> Create Plot Widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('w')
        self.plot_widget.setAspectLocked()

        self.plot_view = self.plot_widget.getPlotItem()

        # -> Enable autoscale of content on resize
        self.plot_view.enableAutoRange()

        # # -> Disable autscale of viewbox
        # self.plot_view.disableAutoRange()
        #
        # # -> Enable auto scale of viewbox
        # self.plot_view.enableAutoRange()

        self.plot_view.setAspectLocked(True)

        # > Setup layout
        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.plot_widget)
        self.setLayout(self.layout)

        # ------------------------------- Plotting
        self.elements_zorder = {
            "env": 1,
            "agents": 3,
            "tasks": 4,
            "agents_paths": 2
        }

        self.task_marker_types = {
            "GOTO": {
                "marker": "x",
                "color": (160, 235, 50, 255)
                # "color": "green"
            },
            "ACTION_1": {
                "marker": "t",
                "color": (255, 0, 0, 255)
                # "color": "red"
            },
            "ACTION_2": {
                "marker": "t1",
                "color": (255, 145, 0, 255)
                # "color": "orange"
            },
            "NO_TASK": {
                "marker": "2",
                "color": (180, 180, 180, 255)
                # "color": "gray"
            }
        }

        self.env_view_obj = {
            "env": None,
            "tasks": {},
            "agents": {},
        }

    def update_env(self):
        # ----- Plot Environment
        env = copy(self.nucleus.env)

        if env["env_type"] is not None:
            pos = np.array(list(env["pos"].values()))
            pos_lst = list(env["pos"].keys())
            graph = env["graph"]   # NetworkX graph

            edge_list = []

            # -> Get corresponding positions indices
            for edge in graph.edges():
                node_1 = pos_lst.index(edge[0])
                node_2 = pos_lst.index(edge[1])
                edge_list.append([node_1, node_2])

            edge_list = np.array(edge_list, dtype=int)
            edge_list = np.array(edge_list)

            graph = pg.GraphItem(
                # > Graph
                pos=pos,
                adj=edge_list,

                # > Node
                size=30,
                symbol='o',
                symbolPen='b',  # Edge
                symbolBrush='w',  # Node

                # > Edge
                pen=pg.mkPen(QColor(173, 216, 230, 100), width=10),

                # > Behavior
                movable=False,
                rotatable=False,
                resizable=False,
                removable=False,

                # > Node scaling
                pxMode=True

            )

            # > Order
            graph.setZValue(self.elements_zorder["env"])

            # > Enable autoscale
            self.plot_view.enableAutoRange()

            self.plot_view.addItem(graph)

            self.env_view_obj["env"] = graph

        self.update_plot()

    def update_plot(self):
        start = time.time()

        task_log = self.task_log.clone()
        fleet = self.fleet.clone()

        for task in task_log.tasks_terminated:
            # -> Check if any artist exists
            if task.id in self.env_view_obj["tasks"].keys():
                for artist in self.env_view_obj["tasks"][task.id].values():
                    self.plot_view.removeItem(artist)

                # -> Clear artists
                del self.env_view_obj["tasks"][task.id]

        removed_artist_time = time.time()

        x_offset = 0.3
        y_offset = 0.3

        # ----- Plot Tasks
        # # -> Compile dict of tasks by task type and action
        # tasks_df = self.task_log.asdf()
        #
        # # > Add pen column based on task type and action
        # def get_pen(task_type, task_action):
        #     if task_type == "GOTO":
        #         pen = (*self.task_marker_types[task_action]["color"], 1)
        #     else:
        #         pen = (*self.task_marker_types[task_type]["color"], 1)
        #
        #     return pen
        #
        # tasks_df["pen"] = tasks_df.apply(
        #     lambda row: get_pen(row["type"], row["instructions"]["ACTION_AT_LOC"]),
        #     axis=1
        # )
        #
        # # > Add marker type
        # tasks_df["marker"] = tasks_df.apply(
        #     lambda row: self.task_marker_types[row["type"]]["marker"],
        #     axis=1
        # )
        #
        # tasks_df["x"] = tasks_df["instructions"].apply(lambda x: x["x"])
        # tasks_df["y"] = tasks_df["instructions"].apply(lambda x: x["y"])
        #
        # # print(tasks_df.to_string())
        #
        # print("PENS", tasks_df["pen"])
        #
        # # ----- Task Marker
        # if self.env_view_obj["tasks"] is None:
        #     # -> Create artist
        #     artist = pg.PlotDataItem(
        #         pos=np.array(tasks_df[["x", "y"]]),
        #         size=15,
        #         symbol=tasks_df["marker"].tolist(),
        #         pen=np.array(tasks_df["pen"]),
        #         brush=np.array(tasks_df["pen"])
        #     )
        #
        #     # -> Store artist locally
        #     self.env_view_obj["tasks"] = artist
        #
        #     # -> Add artist to plot
        #     self.plot_view.addItem(artist)
        #
        # else:
        #     # -> Update artist position
        #     self.env_view_obj["tasks"].setData(pos=np.array(tasks_df[["x", "y"]]))

        # for task_type, task_actions in tasks.items():
        #     for task_action, markers_pos in task_actions.items():
        #         if task_type not in self.env_view_obj["tasks"].keys():
        #             self.env_view_obj["tasks"][task_type] = {}
        #
        #         if task_action not in self.env_view_obj["tasks"][task_type].keys():
        #             # -> Get color
        #             if task_type == "GOTO":
        #                 color = self.task_marker_types[task_action]["color"]
        #             else:
        #                 color = self.task_marker_types[task_type]["color"]
        #
        #             # -> Create artist
        #             artist = pg.ScatterPlotItem(
        #                 pos=np.array(markers_pos),
        #                 size=15,
        #                 symbol=self.task_marker_types[task_type]["marker"],
        #                 pen=mkPen(color=color, width=1),
        #                 brush=mkBrush(color=color)
        #             )
        #
        #             # -> Store artist locally
        #             self.env_view_obj["tasks"][task_type][task_action] = artist
        #
        #             # -> Add artist to plot
        #             self.plot_view.addItem(artist)
        #
        #         else:
        #             # -> Update artist position
        #             self.env_view_obj["tasks"][task_type][task_action].setData(pos=np.array(markers_pos))

        for task in task_log.tasks_pending:
            if task.id not in self.env_view_obj["tasks"].keys():
                self.env_view_obj["tasks"][task.id] = {}

            # > Get task position
            task_x, task_y = task.instructions["x"], task.instructions["y"]

            # ----- Task Label
            # -> Check if task has a text artist
            if "text" not in self.env_view_obj["tasks"][task.id].keys():
                # -> Create artists
                artist = pg.TextItem(
                    text=task.id,
                    color=(0, 0, 0),
                    # anchor=(task_x, task_y - y_offset),
                    anchor=(0.5, 0.5),
                    # border=mkPen(color=(0, 0, 0), width=0.5),
                    # fill=mkBrush(color=(255, 255, 255, 255)),
                    angle=0,

                )

                # > Set position
                artist.setPos(task_x, task_y - y_offset)

                # > Order
                artist.setZValue(self.elements_zorder["tasks"])

                # -> Store artist in task local
                self.env_view_obj["tasks"][task.id]["text"] = artist

                # -> Add artist to plot
                self.plot_view.addItem(artist)

            else:
                # -> If position changed, update position
                if self.env_view_obj["tasks"][task.id]["text"].pos() != (task_x, task_y - y_offset):
                    self.env_view_obj["tasks"][task.id]["text"].setPos(task_x, task_y - y_offset)

            # ----- Task Marker
            # -> Check if task has a marker artist
            if "marker" not in self.env_view_obj["tasks"][task.id].keys():
                # -> Get color
                if task.type == "GOTO":
                    color = self.task_marker_types[task.instructions["ACTION_AT_LOC"]]["color"]
                else:
                    color = self.task_marker_types[task.type]["color"]

                # -> Create artist
                artist = pg.ScatterPlotItem(
                    pos=np.array([[task_x, task_y]]),
                    size=15,
                    symbol=self.task_marker_types[task.type]["marker"],
                    pen=mkPen(color=(0, 0, 0), width=1),
                    brush=mkBrush(color=color)
                )

                # > Order
                artist.setZValue(self.elements_zorder["tasks"])

                # -> Store artist in task local
                self.env_view_obj["tasks"][task.id]["marker"] = artist

                # -> Add artist to plot
                self.plot_view.addItem(artist)

            else:
                # -> If position changed, update position
                if self.env_view_obj["tasks"][task.id]["marker"].pos() != (task_x, task_y):
                    self.env_view_obj["tasks"][task.id]["marker"].setData(pos=np.array([[task_x, task_y]]))

        updated_tasks_time = time.time()

        # # ----- Plot Agents
        # # -> Compile agents positions
        # agent_positions = {}
        #
        # for agent in self.fleet:
        #     if str(agent.skillset) not in agent_positions.keys():
        #         agent_positions[str(agent.skillset)] = []
        #
        #     agent_positions[str(agent.skillset)].append((agent.state.x, agent.state.y))
        #
        # for agent_type, agent_list in agent_positions.items():
        #     if agent_type not in self.env_view_obj["agents"].keys():
        #         # ->  Create artist
        #         artist = pg.ScatterPlotItem(
        #             pos=np.array(agent_list),
        #             size=15,
        #             symbol='o',
        #             pen=mkPen(color=(0, 0, 0), width=1),
        #             brush=mkBrush(color=(255, 255, 255, 255))
        #         )
        #
        #         # -> Store artist locally
        #         self.env_view_obj["agents"][agent_type] = artist
        #
        #         # -> Add artist to plot
        #         self.plot_view.addItem(artist)
        #
        #     else:
        #         # -> Update artist position
        #         self.env_view_obj["agents"][agent_type].setData(pos=np.array(agent_list))

        for agent in fleet:
            if agent.name == "GCS Mission Interface":
                continue

            if agent.id not in self.env_view_obj["agents"].keys():
                self.env_view_obj["agents"][agent.id] = {}

            # > Get agent position
            agent_x, agent_y = agent.state.x, agent.state.y

            # ----- Agent Path
            # -> Get index of agent position in path
            try:
                # -> Set all path points to int
                path = [(int(x), int(y)) for x, y in agent.plan.path]
                agent_loc_index = path.index((agent_x, agent_y))
            except:
                agent_loc_index = 0

            # > Get agent path
            x_list, y_list = [], []

            for step in agent.plan.path[agent_loc_index:]:
                x_list.append(step[0])
                y_list.append(step[1])

            # -> Check if task path has an artist
            if "path" not in self.env_view_obj["agents"][agent.id].keys():
                # -> Get color
                if "ACTION_1" in agent.skillset and "ACTION_2" in agent.skillset:
                    color = self.task_marker_types["ACTION_1/2"]["color"]
                elif "ACTION_1" in agent.skillset:
                    color = self.task_marker_types["ACTION_1"]["color"]
                elif "ACTION_2" in agent.skillset:
                    color = self.task_marker_types["ACTION_2"]["color"]
                else:
                    color = self.task_marker_types["NO_TASK"]["color"]

                # -> Create artist
                artist = pg.PlotDataItem(
                    x=x_list,
                    y=y_list,
                    connect="all",
                    pen=mkPen(color=color, width=2),
                    pxMode=False
                )

                # > Order
                artist.setZValue(self.elements_zorder["agents_paths"])

                # -> Store artist in agent local
                self.env_view_obj["agents"][agent.id]["path"] = artist

                # -> Add artist to plot
                self.plot_view.addItem(artist)

            else:
                # -> Update artist position
                self.env_view_obj["agents"][agent.id]["path"].setData(x=x_list, y=y_list)

                # -> If position changed, update position
                # if self.env_view_obj["agents"][agent.id]["path"].xData != x_list or self.env_view_obj["agents"][agent.id]["path"].yData != y_list:
                #     self.env_view_obj["agents"][agent.id]["path"].setData(x=x_list, y=y_list)

            # ----- Agent Marker
            # -> Check if agent has a marker artist
            if "marker" not in self.env_view_obj["agents"][agent.id].keys():
                # > Get color
                if "ACTION_1" in agent.skillset and "ACTION_2" in agent.skillset:
                    color = self.task_marker_types["ACTION_1/2"]["color"]
                elif "ACTION_1" in agent.skillset:
                    color = self.task_marker_types["ACTION_1"]["color"]
                elif "ACTION_2" in agent.skillset:
                    color = self.task_marker_types["ACTION_2"]["color"]
                else:
                    color = self.task_marker_types["NO_TASK"]["color"]

                # -> Create artist
                artist = pg.ScatterPlotItem(
                    pos=np.array([[agent_x, agent_y]]),
                    size=20,
                    symbol='o',
                    pen=mkPen(color=(0, 0, 0), width=1),
                    brush=mkBrush(color=color)
                )

                # > Order
                artist.setZValue(self.elements_zorder["agents"])

                # -> Store artist in agent local
                self.env_view_obj["agents"][agent.id]["marker"] = artist

                # -> Add artist to plot
                self.plot_view.addItem(artist)

            else:
                # -> If position changed, update position
                if self.env_view_obj["agents"][agent.id]["marker"].pos() != (agent_x, agent_y):
                    self.env_view_obj["agents"][agent.id]["marker"].setData(pos=np.array([[agent_x, agent_y]]))

            # ----- Agent Label
            # -> Check if agent has a text artist
            if "text" not in self.env_view_obj["agents"][agent.id].keys():
                # -> Create artist
                artist = pg.TextItem(
                    text=agent.id,
                    color=(0, 0, 0),
                    anchor=(0.5, 0.5),
                    # border=mkPen(color=(0, 0, 0), width=0.5),
                    # fill=mkBrush(color=(255, 255, 255, 255)),
                    angle=0,
                )

                # > Set position
                artist.setPos(agent_x + 2*0.2, agent_y + 2*0.2)

                # > Order
                artist.setZValue(self.elements_zorder["agents"])

                # -> Store artist in agent local
                self.env_view_obj["agents"][agent.id]["text"] = artist

                # -> Add artist to plot
                self.plot_view.addItem(artist)

            else:
                # -> If position changed, update position
                if self.env_view_obj["agents"][agent.id]["text"].pos() != (agent_x + x_offset, agent_y + y_offset):
                    self.env_view_obj["agents"][agent.id]["text"].setPos(agent_x + x_offset, agent_y + y_offset)

        updated_agents_time = time.time()

        # QtGui.QGuiApplication.processEvents()
        blit_time = time.time()

        # print(f"> Updated plot: {round(time.time() - start, 4)} (rmv art.: {round(removed_artist_time - start, 4)} - upd. tasks: {round(updated_tasks_time - removed_artist_time, 4)} - upd. agents: {round(updated_agents_time - updated_agents_time, 4)} - blit: {round(blit_time - updated_agents_time, 4)}")
        # print(f"    - Removed artists: {round(removed_artist_time - start, 4)}")
        # print(f"    - Updated tasks: {round(updated_tasks_time - removed_artist_time, 4)}")
        # print(f"    - Updated agents: {round(updated_agents_time - updated_agents_time, 4)}")
        # print(f"    - Blit: {round(blit_time - updated_agents_time, 4)}")

