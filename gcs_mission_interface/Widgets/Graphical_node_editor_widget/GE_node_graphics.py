
##################################################################################################################
"""

"""

# Built-in/Generic Imports
import logging

# Libs
from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *

# Own modules

##################################################################################################################

HEADER_ELEMENT = 0
BODY_ELEMENT = 1


class GE_node_graphics(QGraphicsItem):
    def __init__(self, node, graphics_shape: tuple, parent=None):
        super().__init__(parent)

        # Node
        self.node = node

        # Properties
        self.header_color = None
        self.body_color = None

        # -> Set node assets
        self.init_assets(graphics_shape=graphics_shape)

        # -> Set node content
        self.set_content()

        # -> Set node behavior
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setFlag(QGraphicsItem.ItemIsMovable)

        # Flags
        self.was_moved = False
        self._last_selected_state = False

    # ----------------------------------------- Inits
    def init_assets(self, graphics_shape: tuple):
        # -> Set node graphics
        self.graphics_shape = graphics_shape

        # Background
        self._background_brush = QBrush(QColor("#E3212121"))
        self._custom_background_brush = None

        # Edge
        self.width = self.graphics_shape[0]
        self.height = self.graphics_shape[1]
        self.edge_size = 10

        self._pen_default = QPen(QColor("#7F000000"))

        self._pen_selected = QPen(QColor("#FFFFA637"))
        self._pen_selected.setWidth(3.0)

        self._pen_invalid = QPen(QColor(Qt.red))
        self._pen_invalid.setWidth(4.0)
        
        self._pen_dirty = QPen(QColor(Qt.yellow))
        self._pen_dirty.setWidth(4.0)
        
        self._pen_done = QPen(QColor(Qt.green))
        self._pen_done.setWidth(4.0)

        # Header
        self._header = None
        self.header_item = None
        self._header_color = Qt.white
        self._header_font = QFont("Ubuntu", 10)
        self._header_height = 24.0
        self._header_padding = 14.0
        self._header_brush = QBrush(QColor("#FF313131"))
        self._custom_header_brush = None
        self._init_header()

        # Breakpoint
        self._breakpoint = False
        self.breakpoint_item = None
        self._breakpoint_pixmap = QPixmap("src/GUI/Data/Assets/breakpoint.png")
        self._init_breakpoint()

        # States
        self._state = None
        self.state_item = None
        self._state_padding = 24.0
        self._states_pixmaps = {
            "running": QPixmap("src/GUI/Data/Assets/running.svg"),
            "error": QPixmap("src/GUI/Data/Assets/error.svg"),
            "done": QPixmap("src/GUI/Data/Assets/done.svg"),
            "dirty": QPixmap("src/GUI/Data/Assets/dirty.svg"),
            "invalid": QPixmap("src/GUI/Data/Assets/invalid.svg"),
        }
        self._init_state()

    def _init_header(self):
        self.header_item = QGraphicsTextItem(self)

        # Adding node as property to allow selection of node through header graphics
        self.header_item.node = self.node

        self.header_item.setDefaultTextColor(self._header_color)
        self.header_item.setFont(self._header_font)
        self.header_item.setPos(self._header_padding, 0)
        self.header_item.setTextWidth(self.width - 2 * self._header_padding)

    def _init_breakpoint(self):
        self.breakpoint_item = QGraphicsPixmapItem(self)
        self.breakpoint_item.setPos(3.5, 6.3)

    def _init_state(self):
        self.state_item = QGraphicsPixmapItem(self)
        self.state_item.setPos(self.width - self._state_padding, 1)

    # ----------------------------------------- Properties
    @property
    def header(self) -> str:
        return self._header
    @header.setter
    def header(self, value) -> None:
        # -> Setup header item
        self._header = value
        self.header_item.setPlainText(self._header)

    @property
    def breakpoint(self) -> bool:
        return self._breakpoint
    @breakpoint.setter
    def breakpoint(self, value: bool) -> None:
        self._breakpoint = value

        if value is True:
            self.breakpoint_item.setPixmap(
                self._breakpoint_pixmap.scaled(12, 12, transformMode=Qt.SmoothTransformation)
            )
        else:
            self.breakpoint_item.setPixmap(QPixmap())

    @property
    def state(self) -> str:
        return self._state
    @state.setter
    def state(self, value: str) -> None:
        self._state = value

        if value is not None:
            self.state_item.setPixmap(
                self._states_pixmaps[value].scaled(22, 22)
            )
        else:
            self.state_item.setPixmap(QPixmap())

    # ----------------------------------------- Events
    def onSelected(self):
        self.node.scene.graphics_scene.itemSelected.emit()

    def mouseMoveEvent(self, event) -> None:
        """
        Update connection line position when mouse is moved
        """
        super().mouseMoveEvent(event)
        # -> Update edges connected to node
        self.node.update_connected_edges()

        # -> Update the edge position of all other selected nodes
        for node in self.scene().scene.nodes:
            if node.graphics.isSelected():
                node.update_connected_edges()
        
        self.was_moved = True
    
    def mouseReleaseEvent(self, event) -> None:
        super().mouseReleaseEvent(event)

        if self.was_moved:
            self.was_moved = False
            self.node.scene.history.store_history(description="Move node", set_modified=True)
            
            # -> Reset selection
            self.node.scene.reset_last_selected_state()

            # -> Store last selected state
            self.node.scene._last_selected_items = self.node.scene.selectedItems()
            
            return

        # -> last selected flag
        if self._last_selected_state != self.isSelected() \
                or self.node.scene._last_selected_items != self.node.scene.selectedItems():
            # -> Reset all flags
            self.node.scene.reset_last_selected_state()

            # -> Set self flag
            self._last_selected_state = self.isSelected()
            self.onSelected()

    # -------------------------------------------- Environment
    def set_edit_mode(self, state: bool) -> None:
        # -> Set node behavior
        self.setFlag(QGraphicsItem.ItemIsMovable, not state)

    def set_color(self, color: str, element: int) -> None:
        if element == HEADER_ELEMENT:
            self.header_color = color
            self._custom_header_brush = QBrush(QColor(color))

        elif element == BODY_ELEMENT:
            self.body_color = color
            self._custom_background_brush = QBrush(QColor(color))

        self.update()

    def set_content(self):
        self.content = self.node.content
        self.content_item = None

        self.content_item = QGraphicsProxyWidget(self)
        self.content.setGeometry(self.edge_size,
                                 self._header_height + self.edge_size,
                                 self.width - 2 * self.edge_size,
                                 self.height - 2 * self.edge_size - self._header_height)
        self.content_item.setWidget(self.content)

    def boundingRect(self):
        return QRectF(
            0,
            0,
            self.width,
            self.height,
        )

    @property
    def _edge_pen(self):
        if self.isSelected():
            return self._pen_selected
        
        elif self.node.dirty:
            return self._pen_dirty

        elif self.node.invalid:
            return self._pen_invalid
        
        elif self.node.done:
            return self._pen_done
        
        else:
            return self._pen_default
    
    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        # Outline
        path_outline = QPainterPath()
        path_outline.addRoundedRect(0, 0, self.width, self.height, self.edge_size, self.edge_size)

        # -> Setup pen/brush
        painter.setPen(self._edge_pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path_outline.simplified())

        # Name
        path_header = QPainterPath()
        path_header.setFillRule(Qt.WindingFill)
        path_header.addRoundedRect(0, 0, self.width, self._header_height, self.edge_size, self.edge_size)

        # -> Add rectangle to cover rounded bottom corners
        path_header.addRect(0, self._header_height - self.edge_size, self.edge_size, self.edge_size)
        path_header.addRect(self.width - self.edge_size, self._header_height - self.edge_size, self.edge_size, self.edge_size)

        # -> Setup pen/brush
        painter.setPen(Qt.NoPen)
        painter.setBrush(self._header_brush if self._custom_header_brush is None else self._custom_header_brush)
        painter.drawPath(path_header.simplified())

        # Content
        path_content = QPainterPath()
        path_content.setFillRule(Qt.WindingFill)
        path_content.addRoundedRect(0, self._header_height, self.width, self.height - self._header_height, self.edge_size, self.edge_size)

        # -> Add rectangle to cover rounded top corners
        path_content.addRect(0, self._header_height, self.edge_size, self.edge_size)
        path_content.addRect(self.width - self.edge_size, self._header_height, self.edge_size, self.edge_size)

        # -> Setup pen/brush
        painter.setPen(Qt.NoPen)
        painter.setBrush(self._background_brush if self._custom_background_brush is None else self._custom_background_brush)
        painter.drawPath(path_content.simplified())
