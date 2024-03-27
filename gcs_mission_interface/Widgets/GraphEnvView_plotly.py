
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os
from typing import Optional
from pprint import pprint, pformat
import time

# Libs
import networkx as nx
import matplotlib.pyplot as plt
from pandas import to_datetime
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWidgets import QVBoxLayout, QWidget
from PySide6.QtWebEngineWidgets import QWebEngineView

# import matplotlib
# matplotlib.use('tkagg')
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
# from matplotlib.figure import Figure
# from matplotlib.animation import FuncAnimation

import plotly.graph_objects as go

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

        # -> Widget settings
        self.show_goals = True

        # -> Create canvas
        width, height, dpi = 5, 4, 100

        # self.canvas = MplCanvas(width=width, height=height, dpi=dpi)

        # self.fig = self.canvas.fig
        # self.axes = self.canvas.axes

        self.fig = go.Figure()

        # > Update plot content
        self.update()

        # -> Setup layout
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

        # -> Setup plot widget
        self.plot_widget = QWebEngineView()
        self.plot_widget.setHtml(self.fig.to_html(include_plotlyjs='cdn'))

        self.layout.addWidget(self.plot_widget)

        # > Add the canvas to the layout
        # self.layout.addWidget(self.canvas)

        # > Add the navigation toolbar
        # self.toolbar = NavigationToolbar2QT(self.canvas, self)
        # self.layout.addWidget(self.toolbar)

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
                "color": "green"
            },
            "ACTION_1": {
                "marker": "1",
                "color": "red"
            },
            "ACTION_2": {
                "marker": "2",
                "color": "orange"
            },
            "NO_TASK": {
                "marker": "2",
                "color": "gray"
            }
        }

        self.task_marker_types = {
            "GOTO": {
                "marker": "X",
                "color": "green"
            },
            "ACTION_1": {
                "marker": "v",
                "color": "red"
            },
            "ACTION_2": {
                "marker": "^",
                "color": "orange"
            },
            "NO_TASK": {
                "marker": "2",
                "color": "gray"
            }
        }

        # -> Create blit manager
        # self.sim_env_bm = BlitManager(canvas=self.fig.canvas)

    def update_env(self):
        # ----- Plot Environment
        if self.nucleus.env["env_type"] is not None:
            pos = self.nucleus.env["pos"]
            edge_x = []
            edge_y = []
            for edge in self.nucleus.env["graph"].edges():
                x0, y0 = pos[edge[0]]
                x1, y1 = pos[edge[1]]
                edge_x.extend([x0, x1, None])
                edge_y.extend([y0, y1, None])

            self.fig.add_trace(go.Scatter(x=edge_x, y=edge_y, mode='lines', line=dict(color='gray'), opacity=0.5))

    def update_plot(self):
        # ----- Verify all tasks and agents have "artists" local key
        for task in self.nucleus.task_log:
            if "artists" not in task.local.keys():
                task.local["artists"] = {}

        for agent in self.nucleus.fleet:
            if "artists" not in agent.local.keys():
                agent.local["artists"] = {}

        # ----- Remove all unnecessary artists
        for task in self.nucleus.task_log.tasks_terminated:
            # -> Check if any artist exists
            if task.local["artists"]:
                for artist in task.local["artists"].values():
                    # Remove traces from plotly figure
                    self.fig.data.remove(artist)

                # -> Clear artists
                task.local["artists"] = {}

        # ----- Plot Tasks
        for task in self.nucleus.task_log.tasks_pending:
            task_x, task_y = task.instructions["x"], task.instructions["y"]

            if "text" not in task.local["artists"].keys():
                # Create text label
                task_label = go.Scatter(x=[task_x], y=[task_y], mode='text', text=[task.id], textfont=dict(size=10),
                                        textposition="bottom center")
                self.fig.add_trace(task_label)
                task.local["artists"]["text"] = task_label

            if "marker" not in task.local["artists"].keys():
                # Create task marker
                marker_type = self.task_marker_types[task.type]["marker"]
                marker_color = self.task_marker_types[task.type]["color"]
                task_marker = go.Scatter(x=[task_x], y=[task_y], mode='markers',
                                         marker=dict(symbol=marker_type, color=marker_color, size=10))
                self.fig.add_trace(task_marker)
                task.local["artists"]["marker"] = task_marker

        # ----- Agent Path
        for agent in self.nucleus.fleet:
            x_list, y_list = [], []
            for step in agent.plan.path:
                x_list.append(step[0])
                y_list.append(step[1])

            if "path" not in agent.local["artists"].keys():
                path_trace = go.Scatter(x=x_list, y=y_list, mode='lines', line=dict(color='blue'),
                                        name=f"{agent.id} path")
                self.fig.add_trace(path_trace)
                agent.local["artists"]["path"] = path_trace

        # ----- Agent Position
        for agent in self.nucleus.fleet:
            if agent.name == "GCS Mission Interface":
                continue

            agent_x, agent_y = agent.state.x, agent.state.y

            if "marker" not in agent.local["artists"].keys():
                marker_color = self.task_marker_types["ACTION_1/2"]["color"]
                agent_marker = go.Scatter(x=[agent_x], y=[agent_y], mode='markers',
                                          marker=dict(symbol="diamond", color=marker_color, size=7), name=agent.id)
                self.fig.add_trace(agent_marker)
                agent.local["artists"]["marker"] = agent_marker

            if "text" not in agent.local["artists"].keys():
                agent_text = go.Scatter(x=[agent_x], y=[agent_y], mode='text', text=[agent.id], textfont=dict(size=10),
                                        textposition="bottom center")
                self.fig.add_trace(agent_text)
                agent.local["artists"]["text"] = agent_text

        self.plot_widget.setHtml(self.fig.to_html(include_plotlyjs='cdn'))

    # def update_env(self):
    #     # ----- Plot Environment
    #     if self.nucleus.env["env_type"] is not None:
    #         nx.draw(
    #             self.nucleus.env["graph"],
    #             pos=self.nucleus.env["pos"],
    #             ax=self.axes,
    #             node_size=150,
    #             width=5,
    #             edge_color="gray",
    #             node_color="lightblue",
    #             font_size=8,
    #         )
    #
    #     print("Updated environment")
    #
    # def update_plot(self):
    #     start = time.time()
    #
    #     # ----- Verify all tasks and agents have "artists" local key
    #     for task in self.nucleus.task_log:
    #         if "artists" not in task.local.keys():
    #             task.local["artists"] = {}
    #
    #     for agent in self.nucleus.fleet:
    #         if "artists" not in agent.local.keys():
    #             agent.local["artists"] = {}
    #
    #     # ----- Remove all unnecessary artists
    #     for task in self.nucleus.task_log.tasks_terminated:
    #         # -> Check if any artist exists
    #         if task.local["artists"]:
    #             for artist in task.local["artists"].values():
    #                 self.sim_env_bm.remove_artist(artist)
    #
    #             # -> Clear artists
    #             task.local["artists"] = {}
    #
    #     removed_artist_time = time.time()
    #
    #     # ----- Plot Tasks
    #     for task in self.nucleus.task_log.tasks_pending:
    #         # > Get task position
    #         task_x, task_y = task.instructions["x"], task.instructions["y"]
    #
    #         # ----- Task Label
    #         y_offset = 0.6
    #
    #         # -> Check if task has a text artist
    #         if "text" not in task.local["artists"].keys():
    #             # -> Create artist
    #             artist = self.axes.annotate(
    #                 xy=(task_x, task_y - y_offset),
    #                 xytext=(task_x, task_y - y_offset),
    #                 text=task.id,
    #                 fontsize=10,
    #                 ha="center",
    #                 va="center",
    #                 color="black",
    #                 zorder=self.elements_zorder["tasks"]
    #             )
    #
    #             # -> Store artist in task local
    #             task.local["artists"]["text"] = artist
    #
    #             # -> Add artist to blit manager
    #             self.sim_env_bm.add_artist(artist)
    #
    #         else:
    #             # -> Update artist position
    #             task.local["artists"]["text"].xy = (task_x, task_y - y_offset)
    #             task.local["artists"]["text"].set_position((task_x, task_y - y_offset))
    #
    #         # ----- Task Marker
    #         # -> Check if task has a marker artist
    #         if "marker" not in task.local["artists"].keys():
    #             # -> Get color
    #             if task.type == "GOTO":
    #                 color = self.task_marker_types[task.instructions["ACTION_AT_LOC"]]["color"]
    #             else:
    #                 color = self.task_marker_types[task.type]["color"]
    #
    #             # -> Create artist
    #             (artist,) = self.axes.plot(
    #                 task_x,
    #                 task_y,
    #                 self.task_marker_types[task.type]["marker"],
    #                 color=color,
    #                 markeredgecolor="black",
    #                 markersize=9,
    #                 markeredgewidth=.5,
    #                 label=task.id,
    #                 zorder=self.elements_zorder["tasks"]
    #             )
    #
    #             # -> Store artist in task local
    #             task.local["artists"]["marker"] = artist
    #
    #             # -> Add artist to blit manager
    #             self.sim_env_bm.add_artist(artist)
    #
    #         else:
    #             # -> Update artist position
    #             task.local["artists"]["marker"].set_data(task_x, task_y)
    #
    #     updated_tasks_time = time.time()
    #
    #     # ----- Agent Path
    #     for agent in self.nucleus.fleet:
    #         # > Get agent path
    #         x_list, y_list = [], []
    #         for step in agent.plan.path:
    #             x_list.append(step[0])
    #             y_list.append(step[1])
    #
    #         # -> Check if task path has an artist
    #         if "path" not in agent.local["artists"].keys():
    #             # -> Get color
    #             if "ACTION_1" in agent.skillset and "ACTION_2" in agent.skillset:
    #                 color = self.task_marker_types["ACTION_1/2"]["color"]
    #             elif "ACTION_1" in agent.skillset:
    #                 color = self.task_marker_types["ACTION_1"]["color"]
    #             elif "ACTION_2" in agent.skillset:
    #                 color = self.task_marker_types["ACTION_2"]["color"]
    #             else:
    #                 color = self.task_marker_types["NO_TASK"]["color"]
    #
    #             # -> Create artist
    #             (artist,) = self.axes.plot(
    #                 x_list,
    #                 y_list,
    #                 # linestyle="--",
    #                 color=color,
    #                 label=f"{agent.id} path",
    #                 zorder=self.elements_zorder["agents_paths"]
    #             )
    #
    #             # -> Store artist in task local
    #             agent.local["artists"]["path"] = artist
    #
    #             # -> Add artist to blit manager
    #             self.sim_env_bm.add_artist(artist)
    #
    #         else:
    #             # -> Update artist position
    #             agent.local["artists"]["path"].set_data(x_list, y_list)
    #
    #     updated_paths_time = time.time()
    #
    #     # ----- Agent Position
    #     for agent in self.nucleus.fleet:
    #         if agent.name == "GCS Mission Interface":
    #             continue
    #
    #         # > Get agent position
    #         agent_x, agent_y = agent.state.x, agent.state.y
    #
    #         # ----- Agent Marker
    #         # -> Check if agent has a marker artist
    #         if "marker" not in agent.local["artists"].keys():
    #             # > Get color
    #             if "ACTION_1" in agent.skillset and "ACTION_2" in agent.skillset:
    #                 color = self.task_marker_types["ACTION_1/2"]["color"]
    #             elif "ACTION_1" in agent.skillset:
    #                 color = self.task_marker_types["ACTION_1"]["color"]
    #             elif "ACTION_2" in agent.skillset:
    #                 color = self.task_marker_types["ACTION_2"]["color"]
    #             else:
    #                 color = self.task_marker_types["NO_TASK"]["color"]
    #
    #             # -> Create artist
    #             (artist,) = self.axes.plot(
    #                 agent_x,
    #                 agent_y,
    #                 "D",
    #                 color=color,
    #                 markeredgecolor="black",
    #                 markersize=7,
    #                 markeredgewidth=0.5,
    #                 label=agent.id,
    #                 zorder=self.elements_zorder["agents"]
    #             )
    #
    #             # -> Store artist in task local
    #             agent.local["artists"]["marker"] = artist
    #
    #             # -> Add artist to blit manager
    #             self.sim_env_bm.add_artist(artist)
    #
    #         else:
    #             # -> Update artist position
    #             agent.local["artists"]["marker"].set_data(agent_x, agent_y)
    #
    #         # ----- Agent Label
    #         # -> Check if agent has a text artist
    #         if "text" not in agent.local["artists"].keys():
    #             # -> Create artist
    #             artist = self.axes.annotate(
    #                 xy=(agent_x + 2*0.2, agent_y + 2*0.2),
    #                 xytext=(agent_x + 2*0.2, agent_y + 2*0.2),
    #                 text=agent.id,
    #                 fontsize=10,
    #                 ha="center",
    #                 va="center",
    #                 color="black",
    #                 zorder=self.elements_zorder["agents"]
    #             )
    #
    #             # -> Store artist in task local
    #             agent.local["artists"]["text"] = artist
    #
    #             # -> Add artist to blit manager
    #             self.sim_env_bm.add_artist(artist)
    #
    #         else:
    #             # -> Update artist position
    #             agent.local["artists"]["text"].xy = (agent_x + 2*0.2, agent_y + 2*0.2)
    #             agent.local["artists"]["text"].set_position((agent_x + 2*0.2, agent_y + 2*0.2))
    #
    #     updated_agents_time = time.time()
    #
    #     # -> Blit updated artists
    #     self.sim_env_bm.update()
    #
    #     blit_time = time.time()
    #
    #     print(f"> Updated plot: {round(time.time() - start, 4)}")
    #     # print(f"    - Removed artists: {round(removed_artist_time - start, 4)}")
    #     # print(f"    - Updated tasks: {round(updated_tasks_time - removed_artist_time, 4)}")
    #     # print(f"    - Updated paths: {round(updated_paths_time - updated_tasks_time, 4)}")
    #     # print(f"    - Updated agents: {round(updated_agents_time - updated_paths_time, 4)}")
    #     # print(f"    - Blit: {round(blit_time - updated_agents_time, 4)}")