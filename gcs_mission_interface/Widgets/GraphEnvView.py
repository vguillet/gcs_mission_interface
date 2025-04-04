
##################################################################################################################
"""
"""

# Built-in/Generic Imports

# Libs
import networkx as nx
from PySide6.QtWidgets import QVBoxLayout, QWidget

import matplotlib
matplotlib.use('tkagg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

# Own modules
try:
    from maaf_config.maaf_config import *

except ImportError:
    from mmaaf_config.aaf_config.maaf_config import *

##################################################################################################################


class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super(MplCanvas, self).__init__(self.fig)

        self.fig.subplots_adjust(left=0.001, right=0.999, bottom=0.001, top=0.999)


class GraphEnvView(QWidget):
    def __init__(self, agent_id, ros_node):
        super().__init__()

        self.env_type = "graph"
        self.ros_node = ros_node

        # ----------------------------------- Load GUI
        # -> Widget settings
        self.show_goals = True

        # -> Create canvas
        width, height, dpi = 5, 4, 100

        self.canvas = MplCanvas(width=width, height=height, dpi=dpi)

        self.fig = self.canvas.fig
        self.axes = self.canvas.axes

        # > Update plot content
        self.update()

        # -> Setup layout
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

        # > Add the canvas to the layout
        self.layout.addWidget(self.canvas)

        # > Add the navigation toolbar
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        self.layout.addWidget(self.toolbar)

        self.setLayout(self.layout)

    def update(self, *args, **kwargs) -> None:
        """
        Plot the current state of the simulation.
        """
        # -> Clear the axes
        self.axes.clear()

        # -> Add the environment graph to the plot
        nx.draw(self.ros_node.environment["graph"],
                pos=self.ros_node.environment["pos"],
                ax=self.axes,
                node_size=150,
                width=5,
                edge_color="gray",
                node_color="lightblue",
                font_size=8)

        # -> Add agents locations to the plot with legend
        offset = 0.2

        task_marker_types = {
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

        for task in self.ros_node.tasklog.tasks_pending:
            # -> Plot task with task id under the marker and marker type
            self.axes.text(task.instructions["x"], task.instructions["y"] - 2 * offset, task.id, fontsize=10,
                           ha="center", va="center", color="black")
            # self.axes.text(task.instructions["x"], task.instructions["y"] - 3 * offset, task.type, fontsize=8,
            #                ha="center", va="center", color="black")

            # > Get color
            if task.type == "GOTO":
                color = task_marker_types[task.instructions["ACTION_AT_LOC"]]["color"]
            else:
                color = task_marker_types[task.type]["color"]

            self.axes.plot(
                task.instructions["x"],
                task.instructions["y"],
                task_marker_types[task.type]["marker"],
                color=color,
                label=task.id,
                markersize=10,
                markeredgewidth=3
            )

        agent_marker_types = {
            "ACTION_1": {
                "marker": "1",
                "color": task_marker_types["ACTION_1"]["color"]
            },
            "ACTION_2": {
                "marker": "2",
                "color": task_marker_types["ACTION_2"]["color"]
            },
            "ACTION_1/2": {
                "marker": "2",
                "color": "blue"
            },
            "NO_TASK": {
                "marker": "2",
                "color": task_marker_types["NO_TASK"]["color"]
            }
        }

        # -> Plot agents paths
        for agent in self.ros_node.fleet:
            if agent.skillset == ["INTERFACE"]:
                continue

            # > Get agent position
            agent_x, agent_y = agent.state.x, agent.state.y

            # ----- Agent Path
            # -> Get path
            if agent.plan is not None:
                # -> Set all path points to int
                full_path, _, _ = agent.plan.get_path(
                    agent_id=agent.id,
                    tasklog=self.ros_node.tasklog,
                    requirement=None,
                    selection="shortest"
                )
                path = [(int(step[0]), int(step[1])) for step in full_path]

            else:
                path = [(agent_x, agent_y)]
                print(f"!!! Agent {agent.id} has no plan !!!")

            # -> Trim path to current position if agent has tasks in its plan
            if agent.plan.task_sequence:
                try:
                    path = path[path.index((agent_x, agent_y)):]
                except:
                    pass
                    # print(f"!!! Agent {agent.id} has no current position in its path !!!")
                    # path.insert(0, (agent_x, agent_y))

            # > Get agent path
            x_list, y_list = [], []

            for step in path:
                x_list.append(step[0])
                y_list.append(step[1])

            # > Get color
            if "ACTION_1" in agent.skillset and "ACTION_2" in agent.skillset:
                color = agent_marker_types["ACTION_1/2"]["color"]
            elif "ACTION_1" in agent.skillset:
                color = agent_marker_types["ACTION_1"]["color"]
            elif "ACTION_2" in agent.skillset:
                color = agent_marker_types["ACTION_2"]["color"]
            else:
                color = agent_marker_types["NO_TASK"]["color"]

            # > Plot agent path
            self.axes.plot(
                x_list,
                y_list,
                # linestyle="--",
                color=color,
                label=f"{agent.id} path"
            )

        # -> Plot agent position
        for agent in self.ros_node.fleet:
            if agent.skillset == ["INTERFACE"]:
                continue

            # > Get agent position
            x, y = agent.state.x, agent.state.y

            # > Get color
            if "ACTION_1" in agent.skillset and "ACTION_2" in agent.skillset:
                color = agent_marker_types["ACTION_1/2"]["color"]
            elif "ACTION_1" in agent.skillset:
                color = agent_marker_types["ACTION_1"]["color"]
            elif "ACTION_2" in agent.skillset:
                color = agent_marker_types["ACTION_2"]["color"]
            else:
                color = agent_marker_types["NO_TASK"]["color"]

            # > Plot agent position
            self.axes.plot(
                x,
                y,
                "o",
                color=color,
                label=agent.id,
                markersize=10,
                markeredgewidth=3
            )

            # -> Plot agents with agent name and markers representing skills under the marker
            self.axes.text(x + 2*offset, y + 2*offset, agent.id, fontsize=10, ha="center", va="center", color="black")
            # self.axes.text(x + 2*offset, y + 1*offset, agent.skillset, fontsize=4, ha="center", va="center", color="black")

        # -> Add legend
        # self.axes.legend()

        # -> Redraw the canvas
        self.canvas.draw()
