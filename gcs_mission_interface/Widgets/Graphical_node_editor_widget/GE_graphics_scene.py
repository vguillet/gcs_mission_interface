
import math

from PySide6.QtWidgets import QGraphicsScene, QGraphicsView, QWidget
from PySide6.QtCore import *
from PySide6.QtGui import *


class GE_graphics_scene(QGraphicsScene):
    # -> Signals
    itemSelected = Signal()
    itemsDeselected = Signal()

    def __init__(self, scene, parent=None):
        super().__init__(parent)

        self.setItemIndexMethod(QGraphicsScene.NoIndex)

        self.scene = scene

        # Canvas settings
        # TODO: Add to settings
        self._color_background = QColor("#393939")
        self._color_light = QColor("#2f2f2f")
        self._color_dark = QColor("#292929")

        self._grid_size = 20
        self._grid_squares = 5

        # Canvas tools
        self._pen_light = QPen(self._color_light)
        self._pen_light.setWidth(1)

        self._pen_dark = QPen(self._color_dark)
        self._pen_dark.setWidth(2)

        # -> Setup canvas
        # Background color
        self.setBackgroundBrush(self._color_background)

    # Drag event overwrite -> !!! necessary !!! <-
    def dragMoveEvent(self, event):
        pass

    def set_scene(self, width, height):
        self.setSceneRect(-width/2, -height/2, width, height)

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)

        # -> Construct grid
        left_margin = int(math.floor(rect.left()))
        right_margin = int(math.floor(rect.right()))
        top_margin = int(math.floor(rect.top()))
        bottom_margin = int(math.floor(rect.bottom()))

        first_left = left_margin - (left_margin % self._grid_size)
        first_top = top_margin - (top_margin % self._grid_size)

        # -> Create lines making up grid
        lines_light_array = []
        lines_dark_array = []

        for x in range(first_left, right_margin, self._grid_size):
            if x % (self._grid_size * self._grid_squares) != 0:
                lines_light_array.append(QLine(x, top_margin, x, bottom_margin))
            else:
                lines_dark_array.append(QLine(x, top_margin, x, bottom_margin))

        for y in range(first_top, bottom_margin, self._grid_size):
            if y % (self._grid_size * self._grid_squares) != 0:
                lines_light_array.append(QLine(left_margin, y, right_margin, y))
            else:
                lines_dark_array.append(QLine(left_margin, y, right_margin, y))

        # -> Draw the lines
        painter.setPen(self._pen_light)
        try:
            painter.drawLines(*lines_light_array)       # supporting PySide6
        except TypeError:
            painter.drawLines(lines_light_array)        # supporting PySide2

        painter.setPen(self._pen_dark)
        try:
            painter.drawLines(*lines_dark_array)        # supporting PySide6
        except TypeError:
            painter.drawLines(lines_dark_array)         # supporting PySide2
