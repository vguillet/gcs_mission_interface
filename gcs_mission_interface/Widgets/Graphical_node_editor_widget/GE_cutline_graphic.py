

"""
If crash, look at video 24 for possible solution.
 > https://www.youtube.com/watch?v=FPP4RcGeQpU&list=PLZSNHzwDCOggHLThIbCxUhWTgrKVemZkz&index=26
"""
from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *


class Cutline(QGraphicsItem):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.line_points = []

        self._pen = QPen(Qt.white)
        self._pen.setWidthF(2.)
        self._pen.setDashPattern([3, 3])

        self.setZValue(2)

    def boundingRect(self):
        return QRectF(0, 0, 1, 1)

    def paint(self, painter, option, widget=None):
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(self._pen)
        painter.setBrush(Qt.NoBrush)

        poly = QPolygonF(self.line_points)
        painter.drawPolyline(poly)
