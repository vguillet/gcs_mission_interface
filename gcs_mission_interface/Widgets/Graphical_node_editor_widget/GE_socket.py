
import logging

from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

from src.Tools.Serializable import Serializable

logger = logging.getLogger("main.gui.graph_editor.socket")


# Datastream states
DATASTREAM_STATE_UNKNOWN = 0
DATASTREAM_STATE_READY = 1
DATASTREAM_STATE_INVALID = 2

# Socket positions
LEFT_TOP = 1
LEFT_BOTTOM = 2
RIGHT_TOP = 3
RIGHT_BOTTOM = 4
MIDDLE_BOTTOM = 5
MIDDLE_TOP = 6


class GE_socket(Serializable):
    def __init__(self, 
                 ge_node, 
                 name: str, 
                 index: int = 0, 
                 position=LEFT_TOP, 
                 socket_type="misc", 
                 multi_edges: bool = True
                 ):
        Serializable.__init__(self)

        # -> Setup socket properties
        self.ge_node = ge_node
        self.name = name
        self.index = index
        self.position = position
        self.type = socket_type
        self.multi_edges = multi_edges

        self.edges = []

        # -> Setup socket graphics
        if socket_type == "misc":
            self.graphics = GE_socket_graphics(self)

        elif socket_type == "module":
            self.graphics = Module_socket_graphics(self)
        
        elif socket_type == "input":
            self.graphics = Input_socket_graphics(self)

        else:
            logger.info(f"Socket type {socket_type} not implemented, defaulting to misc socket type graphics")
            self.graphics = GE_socket_graphics(self)

        # -> Setup socket tooltip
        self.graphics.setToolTip(self.name.replace("_", " ").capitalize())

        # -> Add socket to ge_node
        self.graphics.setPos(*self.ge_node.get_socket_position(index=self.index, position=self.position))

    def __str__(self):
        return f"Socket - {self.name} ({hex(id(self))[2:5]}..{hex(id(self))[-3:]})"

    def __repr__(self):
        return self.__str__()

    # ----------------------------------------- Properties
    @property
    def dag_node(self): return self.ge_node.dag_node

    # -------------------------------------------- Add
    def add_edge(self, edge):
        self.edges.append(edge)

    # -------------------------------------------- Remove
    def remove_edge(self, edge):
        if edge in self.edges:
            self.edges.remove(edge)
        else:
            logger.warning(f"Trying to remove an edge ({edge}) not connected to socket {self}")

    # -------------------------------------------- Environment
    def get_position(self):
        return [*self.ge_node.get_socket_position(index=self.index, position=self.position)]

    # -------------------------------------------- Serialization
    def serialize(self):
        return {
            "id": self.id,
            "name": self.name,
            "type": self.type,
            "index": self.index,
            "position": self.position,
            "multi_edges": self.multi_edges
        }

    def deserialize(self,
                    data,
                    restore_id: bool = True,
                    hashmap={}):
        hashmap[data["id"]] = self
        if restore_id: self.id = data["id"]

        return True


class GE_socket_graphics(QGraphicsItem):
    def __init__(self, socket):
        super().__init__(socket.ge_node.graphics)

        self.socket = socket

        # -> Setup socket properties
        self.radius = 6.0
        self.edge_width = 1.5
        self._highlighted = False

        self._pen = QPen(QColor("#FF000000"))

        self._pen_highlighted = QPen(Qt.white) 
        self._pen_highlighted.setWidthF(2.0)
        
        self._brush_default = QBrush(QColor("#FFFF7700"))

        self._brush_invalid = QBrush(QColor(Qt.red))
        self._brush_ready = QBrush(QColor(Qt.green))

        # -> Setup socket label properties
        self._label_color = Qt.white
        self._label_font = QFont("Ubuntu", 9)
        self._label_padding = 8

        # -> Setup graphics flags
        self.setAcceptHoverEvents(True)

    @property
    def highlighted(self):
        return self._highlighted
    @highlighted.setter
    def highlighted(self, value):
        self._highlighted = value
        self.update()

    def boundingRect(self):
       return QRectF(
           -self.radius - self.edge_width,
           -self.radius - self.edge_width,
           (self.radius + self.edge_width) * 2,
           (self.radius + self.edge_width) * 2
       )
    
    def hoverEnterEvent(self, event) -> None:
        self.highlighted = True

    def hoverLeaveEvent(self, event) -> None:
        self.highlighted = False
    
    @property
    def _brush(self):
        if self.socket.ge_node.dag_node.exhaustive_performance_mode:
            if self.socket.ge_node.invalid is True:
                return self._brush_invalid
            else:
                return self._brush_default

        # Block invalid
        if self.socket.ge_node.invalid:
            return self._brush_invalid

        # Datastream output
        elif self.socket.type == "output":
            pass

        # Datastream has no requirements
        elif self.socket.ge_node.block.get_datastream_requirement(datastream_ref=self.socket.name) is not None:
            # -> Check datastreams compatibility
            requirements = self.socket.ge_node.block.get_datastream_requirement(datastream_ref=self.socket.name)

            for edge in self.socket.edges:
                source_socket = edge.start_socket
                source_datastream = \
                    source_socket.ge_node.block.get_datastream_data(datastream_ref=source_socket.name)

                # -> Check if source datastream corresponds to self datastream requirements
                compatible = self.socket.ge_node.block._check_data_against_requirements(
                        data=source_datastream,       # Source datastream data
                        requirements=requirements,          # Target datastream requirements
                        ignore_requirements_override=True
                    )
                
                if not compatible:
                    return self._brush_invalid
            
        # Datastream ready
        if self.socket.ge_node.block.check_datastream_ready(datastream_ref=self.socket.name)[1] == DATASTREAM_STATE_READY and self.socket.ge_node.done:
            return self._brush_ready
        else:
            return self._brush_default

    def _base_paint(self, painter):
        # -> Paint socket
        painter.setPen(self._pen_highlighted if self.highlighted else self._pen)
        painter.setBrush(self._brush)

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        # # -> Paint socket name
        # socket_label = QGraphicsTextItem(self)
        # socket_label.setPlainText(self.socket.name)
        #
        # socket_label.setDefaultTextColor(self._label_color)
        # socket_label.setFont(self._label_font)
        # socket_label.setPos(self._label_padding, -11.5)
        # socket_label.setTextWidth(self.width - 2 * self._header_padding)

        # -> Paint socket
        self._base_paint(painter=painter)

        # Ellipse
        painter.drawEllipse(-self.radius, -self.radius, self.radius * 2, self.radius * 2)


class Input_socket_graphics(GE_socket_graphics):
    def __init__(self, socket):
        GE_socket_graphics.__init__(self, socket)

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        # -> Paint socket
        self._base_paint(painter=painter)

        # Triangle
        points = QPolygon([
            QPoint(-self.radius + 0.5, -self.radius - 1),
            QPoint(self.radius + 2.5, 0),
            QPoint(-self.radius + 0.5, self.radius + 1)
        ])

        painter.drawPolygon(points)


class Module_socket_graphics(GE_socket_graphics):
    def __init__(self, socket):
        super().__init__(socket)

        # -> Setup socket properties
        self.width = self.radius * 2

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        # -> Paint socket
        self._base_paint(painter=painter)

        # Square
        path_outline = QPainterPath()
        path_outline.addRoundedRect(
            -self.radius, -self.radius,
            self.width, self.width,
            self.edge_width, self.edge_width
        )

        # -> Setup pen/brush
        painter.drawPath(path_outline.simplified())


# -> Create list of socket graphics classes
socket_graphics_classes = (GE_socket_graphics, Input_socket_graphics, Module_socket_graphics)
