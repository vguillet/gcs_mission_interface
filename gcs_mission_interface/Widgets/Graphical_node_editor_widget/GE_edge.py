
import logging

from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *

import math

from numpy import source

from src.Tools.Serializable import Serializable
from src.GUI.Widgets.Graphical_node_editor_widget.GE_socket import *

from src.GUI.UI_singletons import Nucleus_singleton

logger = logging.getLogger("main.gui.graph_editor.edge")

EDGE_TYPE_DIRECT = 1
EDGE_TYPE_BEZIER = 2

DATASTREAM_STATE_UNKNOWN = 0
DATASTREAM_STATE_READY = 1
DATASTREAM_STATE_INVALID = 2


class GE_edge(Serializable):
    def __init__(self, scene, start_socket=None, end_socket=None, edge_type=EDGE_TYPE_BEZIER):
        logger.debug("")
        logger.debug(f"===== Creating edge <{self}>")
        Serializable.__init__(self)

        self.nucleus = Nucleus_singleton()

        # DAG
        self.dag_edge = None

        # GE
        self.scene = scene
        self.graphics = None

        # -> Setup edge sockets
        self._start_socket = None
        self._end_socket = None

        self.start_socket = start_socket
        self.end_socket = end_socket

        # -> Setup edge graphics item
        self._type = None
        self.edge_type = edge_type

        # -> Store edge in scene edge list
        self.scene.add_edge(self)                

    def __str__(self) -> str:
        return f"Edge {hex(id(self))[2:5]}..{hex(id(self))[-3:]}"

    def __repr__(self) -> str:
        return self.__str__()

    # ----------------------------------------- Properties
    @property
    def ready(self):
        # -> Check source block for readiness of all connected datastreams
        if self.start_socket is not None:
            _, state = self.start_socket.ge_node.block.check_datastream_ready(datastream_ref=self.start_socket.name)

        # -> Default to True
        else:
            state = 0

        # -> Apply not done
        if state == DATASTREAM_STATE_READY and not self.start_socket.ge_node.done:
            state = DATASTREAM_STATE_UNKNOWN
        
        # -> Apply not invalid
        elif state == DATASTREAM_STATE_INVALID and not self.start_socket.ge_node.invalid:
            state = DATASTREAM_STATE_UNKNOWN

        # -> Apply done
        elif self.start_socket.ge_node.done:
            state = DATASTREAM_STATE_READY

        return state

    @property
    def datastreams(self):
        if self.dag_edge is not None:
            return self.dag_edge.get_target_datastream_refs()
        else:
            return None

    @property
    def start_socket(self): return self._start_socket
    @start_socket.setter
    def start_socket(self, value):
        # -> Set start socket
        self._start_socket = value

        if self.start_socket is not None:
            logger.debug(f">>>>> Setting start socket of edge <{self}> to {self.start_socket}")
            self.start_socket.add_edge(edge=self)

            if self.start_socket is not None and self.end_socket is not None:
                self.__create_dag_edge()

        else:
            logger.debug(f">>>>> Setting start socket of edge <{self}> to None")

    @property
    def end_socket(self): return self._end_socket
    @end_socket.setter
    def end_socket(self, value):
        # -> Set end socket
        self._end_socket = value

        if self.end_socket is not None:
            logger.debug(f">>>>> Setting end socket of edge <{self}> to {self.end_socket}")
            self.end_socket.add_edge(edge=self)

            if self.start_socket is not None and self.end_socket is not None:
                self.__create_dag_edge()

        else:
            logger.debug(f">>>>> Setting end socket of edge <{self}> to None")

    def __create_dag_edge(self):
        # -> Flip sockets if start socket is an output socket or end socket is an input socket
        if self.start_socket.type == "input" or self.end_socket.type == "output":
            # Flip
            self._start_socket, self._end_socket = self.end_socket, self.start_socket
        else:
            pass
            
        logger.debug(f">>>>> Creating DAG edge between {self.start_socket} and {self.end_socket}")
        self.nucleus[self.scene.namespace]["DAG"].add_datastream_link(
                source_node=self.start_socket.dag_node,
                target_node=self.end_socket.dag_node,
                source_datastream_ref=self.start_socket.name,
                target_datastream_ref=self.end_socket.name
            )

        # -> Get DAG edge in between the two nodes. If it already exists, use existing one
        self.dag_edge = self.nucleus[self.scene.namespace]["DAG"].get_edge_between(
            source_node=self.start_socket.dag_node,
            target_node=self.end_socket.dag_node
        )

    # -------------------------------------------- Remove
    def remove(self):
        logger.debug("")
        logger.debug(f"===== Removing edge <{self}>")
        # -> Remove datastream link
        if self.start_socket is not None and self.end_socket is not None:
            self.nucleus[self.scene.namespace]["DAG"].remove_datastream_link(
                source_node=self.start_socket.ge_node.dag_node,
                target_node=self.end_socket.ge_node.dag_node,
                source_datastream_ref=self.start_socket.name,
                target_datastream_ref=self.end_socket.name
            )

        # -> Remove edge from sockets
        self._remove_from_sockets()
        self.scene.graphics_scene.removeItem(self.graphics)
        self.graphics = None

        # -> Try removing edge from scene (can fail if already removed)
        try:
            self.scene.remove_edge(self)
        except ValueError:
            pass

    def _remove_from_sockets(self):
        # -> Remove edge from start socket
        if self.start_socket is not None:
            self.start_socket.remove_edge(edge=self)

        # -> Remove edge from end socket
        if self.end_socket is not None:
            self.end_socket.remove_edge(edge=self)

        # -> Remove sockets from edge
        self.start_socket = None
        self.end_socket = None

    # -------------------------------------------- Misc
    @property
    def edge_type(self): return self._type
    @edge_type.setter
    def edge_type(self, value):
        # -> Destroy any existing graphics item if present
        if hasattr(self, "graphics") and self.graphics is not None:
            self.scene.graphics_scene.removeItem(self.graphics)

        self._type = value

        if self.edge_type == EDGE_TYPE_DIRECT:
            self.graphics = Edge_direct_graphics(self)

        elif self.edge_type == EDGE_TYPE_BEZIER:
            self.graphics = Edge_bezier_graphics(self)

        else:
            logging.warning(f"Invalid edge type ({self.edge_type}), defaulting to EDGE_TYPE_DIRECT")
            self.graphics = Edge_direct_graphics(self)

        # -> Add edge to graphics scene
        self.scene.graphics_scene.addItem(self.graphics)

        # -> Draw edge
        if self.start_socket is not None:
            self.update_positions()

    def update_positions(self):
        # -> Update start edge position
        start_pos = self.start_socket.get_position()
        start_pos[0] += self.start_socket.ge_node.graphics.pos().x()
        start_pos[1] += self.start_socket.ge_node.graphics.pos().y()

        self.graphics.set_start(*start_pos)

        # -> Update end edge position
        if self.end_socket is not None:
            end_pos = self.end_socket.get_position()
            end_pos[0] += self.end_socket.ge_node.graphics.pos().x()
            end_pos[1] += self.end_socket.ge_node.graphics.pos().y()

            self.graphics.set_end(*end_pos)
        else:
            self.graphics.set_end(*start_pos)

        # -> Update edge bezier control points based on start and end edge positions
        self.graphics.refresh()

    # -------------------------------------------- Serialization
    def serialize(self):
        logger.debug(f">>>>> Serializing edge <{self}>")
        return {
            "id": self.id,
            "edge_type": self.edge_type,
            "start_socket": self.start_socket.id,
            "end_socket": self.end_socket.id if self.end_socket is not None else None
        }

    def serialize_to_canvas(self):
        # -> Get edge start position
        if self.start_socket.type == "input":
            fromSide = "left"
        elif self.start_socket.type == "output":
            fromSide = "right"
        elif self.start_socket.type == "top":
            fromSide = "top"
        elif self.start_socket.type == "module":
            fromSide = "bottom"
        else:
            raise ValueError(f"Invalid end socket type ({self.start_socket.type})")

        # -> Get edge end position
        if self.end_socket.type == "input":
            toSide = "left"
        elif self.end_socket.type == "output":
            toSide = "right"
        elif self.end_socket.type == "top":
            toSide = "top"
        elif self.end_socket.type == "module":
            toSide = "bottom"
        else:
            raise ValueError(f"Invalid end socket type ({self.end_socket.type})")

        data = {
            "id": self.id,
            "fromNode": self.start_socket.ge_node.id,
            "fromSide": fromSide,
            "toNode": self.end_socket.ge_node.id,
            "toSide": toSide,
            "color": "",
            "label": ""
        }

        return data

    def deserialize(self,
                    data,
                    restore_id: bool = True,
                    hashmap={}
                    ):
        logger.debug(">>>>> Deserializing edge")
        if restore_id: self.id = data["id"]

        self.start_socket = hashmap[data["start_socket"]]
        self.end_socket = hashmap[data["end_socket"]]
        self.edge_type = data["edge_type"]
        return True


EDGE_CP_ROUNDNESS = 10


class GE_edge_graphics(QGraphicsPathItem):
    def __init__(self, edge, parent=None):
        super().__init__(parent)

        self.edge = edge

        # -> Set edge start and start coordinates
        self.position_start = [0, 0]
        self.position_end = [100, 100]

        # -> Init edge assets
        self.init_assets()

        # -> Setup edge behavior
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setZValue(-1)

        # Flags
        self._last_selected_state = False
        self._hovered = False

        # -> Setup graphics flags
        self.setAcceptHoverEvents(True)

    def onSelected(self):
        self.edge.scene.graphics_scene.itemSelected.emit()

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        
        # -> last selected flag
        if self._last_selected_state != self.isSelected() or self.edge.scene._last_selected_items != self.edge.scene.selectedItems():
            # -> Reset all flags
            self.edge.scene.reset_last_selected_state()

            # -> Set self flag
            self._last_selected_state = self.isSelected()
            self.onSelected()

    def init_assets(self):
        # -> Set edge graphics
        # Default pen
        self._default_color = QColor("#001000")
        self._pen_default = QPen(self._default_color)
        self._pen_default.setWidth(2.0)

        # Selected pen
        self._pen_selected = QPen(QColor(Qt.yellow))
        self._pen_selected.setWidth(2.0)

        # Dragging pen
        self._pen_dragging = QPen(self._default_color)
        self._pen_dragging.setStyle(Qt.DashLine)
        self._pen_dragging.setWidth(2.0)

        # Hovered pen
        self._pen_hovered = QPen(Qt.white)
        self._pen_hovered.setWidth(3.0)

        # Ready pen
        self._pen_ready = QPen(Qt.green)
        self._pen_ready.setWidth(3.0)

        # Invalid pen
        self._pen_invalid = QPen(Qt.red)
        self._pen_invalid.setWidth(3.0)

        # -> Setup graphics flags
        self.setAcceptHoverEvents(False)

    def set_start(self, x, y):
        self.position_start = [x, y]

    def set_end(self, x, y):
        self.position_end = [x, y]

    def boundingRect(self):
        return self.shape().boundingRect()

    def shape(self):
        return self.calculate_path()

    def hoverEnterEvent(self, event) -> None:
        self._hovered = True
        self.update()

    def hoverLeaveEvent(self, event) -> None:
        self._hovered = False
        self.update()

    @property
    def _pen(self):
        fully_connected = self.edge.start_socket is not None and self.edge.end_socket is not None

        if not fully_connected:
            return self._pen_dragging

        # -> Performance visuals
        if self.edge.end_socket.ge_node.dag_node.exhaustive_performance_mode:
            if self.edge.end_socket.ge_node.invalid is True:
                return self._pen_invalid

            elif self._hovered is True:
                return self._pen_hovered

            elif self.isSelected():
                return self._pen_selected

            else:
                return self._pen_default

        # -> Full performance visuals
        elif self._hovered is True:
            return self._pen_hovered

        elif self.isSelected():
            return self._pen_selected

        elif self.edge.end_socket.ge_node.block.get_datastream_requirement(datastream_ref=self.edge.end_socket.name) is not None:
            # -> Check datastreams compatibility
            requirements = self.edge.end_socket.ge_node.block.get_datastream_requirement(datastream_ref=self.edge.end_socket.name)

            source_socket = self.edge.start_socket
            source_datastream = \
                source_socket.ge_node.block.get_datastream_data(datastream_ref=source_socket.name)

            # -> Check if source datastream corresponds to self datastream requirements
            compatible = self.edge.end_socket.ge_node.block._check_data_against_requirements(
                    data=source_datastream,       # Source datastream data
                    requirements=requirements,          # Target datastream requirements
                    ignore_requirements_override=True
                )

            if not compatible:
                return self._pen_invalid

        ready_state = self.edge.ready

        if ready_state == DATASTREAM_STATE_READY:
            return self._pen_ready

        elif ready_state == DATASTREAM_STATE_INVALID:
            return self._pen_invalid

        else:
            return self._pen_default

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        # -> Determine edge path
        self.setPath(self.calculate_path())

        # -> Draw edge
        painter.setPen(self._pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(self.path())

    def calculate_path(self):
        """
        Handles drawing QPainterPath from point A to point B
        """
        raise NotImplemented("This method must be implemented in child class")

    def intersects_with(self, point_1, point_2):
        # -> Construct cutpath
        cutpath = QPainterPath(point_1)
        cutpath.lineTo(point_2)

        # -> Get edge path
        path = self.calculate_path()

        # -> Check if cutpath intersects with edge path
        return cutpath.intersects(path)


# ====================================================== Edge_graphics classes
class Edge_direct_graphics(GE_edge_graphics):
    def calculate_path(self):
        path = QPainterPath(QPointF(self.position_start[0], self.position_start[1]))
        path.lineTo(self.position_end[0], self.position_end[1])

        return path


class Edge_bezier_graphics(GE_edge_graphics):
    def calculate_path(self):
        # -> Solve for control points
        s = self.position_start
        d = self.position_end

        distance = (d[0] - s[0]) * 0.5

        # -> Construct control points
        cpx_s = +distance
        cpx_d = -distance
        cpy_s = 0
        cpy_d = 0

        sspos = self.edge.start_socket.position

        if s[0] > d[0] and sspos in (RIGHT_TOP, RIGHT_BOTTOM) or s[0] < d[0] and sspos in (LEFT_TOP, LEFT_BOTTOM):
            cpx_d *= -1
            cpx_s *= -1

            cpy_d = ((s[1] - d[1]) / math.fabs((s[1] - d[1]) or 0.00001)) * EDGE_CP_ROUNDNESS
            cpy_s = ((d[1] - s[1]) / math.fabs((d[1] - s[1]) or 0.00001)) * EDGE_CP_ROUNDNESS

        # -> Determine path
        path = QPainterPath(QPointF(self.position_start[0], self.position_start[1]))
        path.cubicTo(
            s[0] + cpx_s,
            s[1] + cpy_s,
            d[0] + cpx_d,
            d[1] + cpy_d,
            self.position_end[0],
            self.position_end[1]
        )

        return path

# -> Create list of edge graphics classes
edge_graphics_classes = (Edge_direct_graphics, Edge_bezier_graphics)
