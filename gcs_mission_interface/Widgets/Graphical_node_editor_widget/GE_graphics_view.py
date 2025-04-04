
##################################################################################################################

import math
import logging

from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *

from src.GUI.Widgets.Graphical_node_editor_widget.GE_node_graphics import GE_node_graphics
from src.GUI.Widgets.Graphical_node_editor_widget.GE_socket import GE_socket_graphics, socket_graphics_classes
from src.GUI.Widgets.Graphical_node_editor_widget.GE_edge import GE_edge, GE_edge_graphics, edge_graphics_classes
from src.GUI.Widgets.Graphical_node_editor_widget.GE_cutline_graphic import Cutline

from src.GUI.Widgets.Graphical_node_editor_widget.GE_edge_snapping import Edge_snapping

from src.GUI.UI_singletons import Nucleus_singleton

##################################################################################################################

# TODO: Add to settings
MODE_NOOP = 1
MODE_EDGE_DRAG = 2
MODE_EDGE_CUT = 3

EDGE_DRAG_START_THRESHOLD = 10

# -> Enable edge dragging feature
EDGE_DRAGGING = True

# -> Enable edge snapping feature
EDGE_SNAPPING = True
EDGE_SNAPPING_RADIUS = 11

logger = logging.getLogger("main.gui.graph_editor.graphics_view")
main_logger = logging.getLogger("main")


class GE_graphics_view(QGraphicsView):
    # -> Signals
    scenePosChanged = Signal(int, int)

    def __init__(self, parent_widget, graphics_scene, parent=None):
        super().__init__(parent)
        self.parent_widget = parent_widget

        self.nucleus = Nucleus_singleton()

        # -> Setup states
        self.mode = MODE_NOOP
        self.editing_flag = False      # Used to prevent handling delete key while interacting with content
        self.rubberband_dragging_rectangle = False
        self.last_lmb_press_pos = None

        # -> Setup graphics scene in view
        self.graphics_scene = graphics_scene
        self.setScene(self.graphics_scene)

        # -> Add self reference to graphics scene
        self.graphics_scene.view = self

        # -> Setup view anti-aliasing
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)

        # -> Fix caching
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

        # -> Setup view states
        self.zoom = 9              # TODO: Add to settings
        self.zoom_in_factor = 1.25
        self.zoom_step = 1

        self.zoom_range = [0, 10]   # TODO: Add to settings
        self.zoom_clamp = True      # TODO: Add to settings

        # Initial zoom
        self.set_zoom(zoom_factor=1)

        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.RubberBandDrag)

        # -> Setup edge drag
        self.drag_start_socket = None

        # -> Setup snapping
        self.snapping = Edge_snapping(view=self, snapping_radius=EDGE_SNAPPING_RADIUS)

        # -> Setup cutline
        self.cutline = Cutline()
        self.graphics_scene.addItem(self.cutline)

        # -> Set flags
        self.setAcceptDrops(True)

        # -> Setup listeners
        self._drag_enter_listeners = []
        self._drop_listeners = []

    def set_zoom(self, zoom_factor):
        # -> Apply clamping
        clamped = False
        if self.zoom < self.zoom_range[0]:
            self.zoom = self.zoom_range[0]
            clamped = True

        if self.zoom > self.zoom_range[1]:
            self.zoom = self.zoom_range[1]
            clamped = True

        # -> Set scene scale
        if not clamped or self.zoom_clamp is False:
            self.scale(zoom_factor, zoom_factor)

        # -> Calculate current zoom factor % based on zoom range
        zoom_factor_percent = (self.zoom - self.zoom_range[0]) / (self.zoom_range[1] - self.zoom_range[0])

        # -> Calculate zoom factor slider value based on zoom factor % and zoom slider range inverted
        zoom_slider_range = self.parent_widget.ui.zoom_slider.maximum() - self.parent_widget.ui.zoom_slider.minimum()
        zoom_slider_value = zoom_factor_percent * zoom_slider_range + self.parent_widget.ui.zoom_slider.minimum()
        zoom_slider_value = self.parent_widget.ui.zoom_slider.maximum() - zoom_slider_value + self.parent_widget.ui.zoom_slider.minimum()

        # -> Set zoom slider value
        self.parent_widget.ui.zoom_slider.blockSignals(True)
        self.parent_widget.ui.zoom_slider.setValue(zoom_slider_value)
        self.parent_widget.ui.zoom_slider.blockSignals(False)

        self.parent_widget.ui.current_zoom_level.setText(f"{int(zoom_factor_percent * 100)}%")

    def isEdgeDraggingEnabled(self, event=None):
        return EDGE_DRAGGING and not self.parent_widget.lock_layout

    def isSnappingEnabled(self, event=None):
        return EDGE_SNAPPING and not (event.modifiers() and Qt.CTRL) if event else True

    # ================================================== Drag/drop event
    def dragEnterEvent(self, event):
        for callback in self._drag_enter_listeners: callback(event=event)

    def add_drag_enter_listener(self, callback):
        self._drag_enter_listeners.append(callback)

    def dropEvent(self, event):
        for callback in self._drop_listeners: callback(event=event)

    def add_drop_listener(self, callback):
        self._drop_listeners.append(callback)

    # ================================================== Mouse events
    def mousePressEvent(self, event) -> None:
        # -> Set editing flag
        item = self._get_item_at_click(event)

        if item is not None \
                and type(item) not in edge_graphics_classes \
                and not hasattr(item, "node"):
            logger.debug("\n -------------------> Editing flag set")
            self.editing_flag = True

        else:
            logger.debug("\n -------------------> Editing flag cleared")
            self.editing_flag = False

        # -> Pass event to relevant handler
        if event.button() == Qt.MiddleButton:
            self.middle_mouse_button_press(event)

        elif event.button() == Qt.LeftButton:
            self.left_mouse_button_press(event)
            self._check_emit_item_selected()

        elif event.button() == Qt.RightButton:
            self.right_mouse_button_press(event)

        else:
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MiddleButton:
            self.middle_mouse_button_release(event)

        elif event.button() == Qt.LeftButton:
            self.left_mouse_button_release(event)

        elif event.button() == Qt.RightButton:
            self.right_mouse_button_release(event)

        else:
            super().mouseReleaseEvent(event)

    def mouseMoveEvent(self, event) -> None:
        # -> Get current mouse scene_pos
        scene_pos = self.mapToScene(event.pos())

        if self.isSnappingEnabled(event=event):
            # self._set_sockets_highlight(scene_pos=scene_pos, state=False, radius=EDGE_SNAPPING_RADIUS + 5000)
            _, scene_pos = \
                self.snapping.get_snapped_socket_position(scene_pos=scene_pos, start_socket=self.drag_start_socket)

        if self.mode == MODE_EDGE_DRAG:
            # -> Update edge destination position
            self.dragged_edge.graphics.set_end(scene_pos.x(), scene_pos.y())

            # -> Update edge graphics
            self.dragged_edge.graphics.refresh()

        if self.mode == MODE_EDGE_CUT and self.cutline is not None:
            self.cutline.line_points.append(scene_pos)
            self.cutline.refresh()

        # -> Emit scenePosChanged signal
        self.last_scene_mouse_position = self.mapToScene(event.pos())

        self.scenePosChanged.emit(
            int(self.last_scene_mouse_position.x()),
            int(self.last_scene_mouse_position.y())
        )

        super().mouseMoveEvent(event)

    # ----- Middle mouse
    def middle_mouse_button_press(self, event):
        # -> Fake mice release event
        release_event = QMouseEvent(
            QEvent.MouseButtonRelease,
            event.localPos(),
            event.screenPos(),
            Qt.LeftButton,
            Qt.NoButton,
            event.modifiers()
        )

        super().mouseReleaseEvent(release_event)

        # -> Toggle view drag mode
        self.setDragMode(QGraphicsView.ScrollHandDrag)

        # -> Fake mice left press event
        press_event = QMouseEvent(
            event.type(),
            event.localPos(),
            event.screenPos(),
            Qt.LeftButton,
            event.buttons() | Qt.LeftButton,
            event.modifiers()
        )

        super().mousePressEvent(press_event)

    def middle_mouse_button_release(self, event):
        # -> Construct fake event
        fake_event = QMouseEvent(
            event.type(),
            event.localPos(),
            event.screenPos(),
            Qt.LeftButton,
            event.buttons() & ~Qt.LeftButton,
            event.modifiers()
        )

        super().mouseReleaseEvent(fake_event)

        # -> Toggle view drag mode
        self.setDragMode(QGraphicsView.RubberBandDrag)

    # ----- Left mouse
    def left_mouse_button_press(self, event):
        logger.debug("\n ++++++++ Left mouse button press ++++++++")

        # -> Set states
        self.last_lmb_press_pos = self.mapToScene(event.pos())

        # -> Get item at click
        item = self._get_item_at_click(event)

        # -> If item clicked is socket
        if type(item) in socket_graphics_classes:
            # -> Toggle mode is mode is noop
            if self.mode == MODE_NOOP:
                if self.isEdgeDraggingEnabled(event=event):
                    # -> Start edge drag
                    self._edge_drag_start(item=item)
                return

        # -> If already in mode edge drag
        if self.mode == MODE_EDGE_DRAG:
            # -> Override item to use as end
            if self.isSnappingEnabled(event=event):
                item = self.snapping.get_snapped_socket_item(event=event, start_socket=self.drag_start_socket)

            # -> End edge drag
            res = self._edge_drag_end(item=item)

            if res:
                return

        if item is None:
            if event.modifiers() & Qt.ShiftModifier:
                self.mode = MODE_EDGE_CUT

                # -> Create fake release event
                fake_event = QMouseEvent(
                    QEvent.MouseButtonRelease,
                    event.localPos(),
                    event.screenPos(),
                    Qt.LeftButton,
                    Qt.NoButton,
                    event.modifiers()
                )

                super().mouseReleaseEvent(fake_event)

                # -> Change cursor to crosshair
                QApplication.setOverrideCursor(Qt.CrossCursor)

                return

            else:
                self.rubberband_dragging_rectangle = True

        # -> Pass event to super (to apply other handlers for this event)
        super().mousePressEvent(event)

    def left_mouse_button_release(self, event):
        logger.debug("\n ------- Left mouse button release -------")

        # -> Get item at click
        item = self._get_item_at_click(event)

        # -> If already in mode edge drag
        if self.mode == MODE_EDGE_DRAG:
            # -> If mouse travel is above threshold
            if self._lmb_mouse_moved(event):
                # -> Override item to use as end
                if self.isSnappingEnabled(event=event):
                    item = self.snapping.get_snapped_socket_item(event=event, start_socket=self.drag_start_socket)

                # -> End edge drag mode
                res = self._edge_drag_end(item=item)

                if res:
                    return

        if self.mode == MODE_EDGE_CUT:
            # -> Check if cutline is intersecting with any item
            self._cut_intersecting_edges(event)

            # -> Reset cutline
            self.cutline.line_points = []
            self.cutline.refresh()

            # -> Change cursor to arrow
            QApplication.setOverrideCursor(Qt.ArrowCursor)

            # -> Reset mode
            self.mode = MODE_NOOP

            return

        if self.rubberband_dragging_rectangle:
            # -> Record action in history
            self.rubberband_dragging_rectangle = False
            # self.graphics_scene.scene.history.store_history("Select", set_modified=False)

            # current_selected_items = self.graphics_scene.selectedItems()
            #
            # if current_selected_items != self.graphics_scene.scene._last_selected_items:
            #     if current_selected_items == []:
            #         self.graphics_scene.itemsDeselected.emit()
            #
            #     else:
            #         self.graphics_scene.itemSelected.emit()
            #
            #     self.graphics_scene.scene._last_selected_items = current_selected_items
                
            return
        
        # -> Deselect everything
        if item is None:
            self.graphics_scene.itemsDeselected.emit()

        super().mouseReleaseEvent(event)

    def _lmb_mouse_moved(self, event):
        # -> Get mouse travel
        mouse_travel = self.mapToScene(event.pos()) - self.last_lmb_press_pos
        mouse_travel_length = math.sqrt(mouse_travel.x() ** 2 + mouse_travel.y() ** 2)

        # -> If mouse has moved enough
        if mouse_travel_length > EDGE_DRAG_START_THRESHOLD:
            return True

        return False

    def _edge_drag_start(self, item):
        logger.debug(f"-> Edge drag start on Socket {item}")
        # -> Store previous edge
        self.drag_start_socket = item.socket

        # -> Create new edge
        if item.socket.type == "module":
            edge_type = 1
        else:
            edge_type = 2

        self.dragged_edge = GE_edge(
            scene=self.graphics_scene.scene,
            start_socket=item.socket,
            end_socket=None,
            edge_type=edge_type
        )

        logger.debug(f"-> Edge created{self.dragged_edge}")

        # -> Start edge drag mode
        self.mode = MODE_EDGE_DRAG

    def _edge_drag_end(self, item):
        """
        Return True to skip the rest of the code
        """
        logger.debug("-> Edge drag end")

        # -> Stop edge drag mode
        self.mode = MODE_NOOP

        # -> If item clicked is socket
        if isinstance(item, GE_socket_graphics):
            # -> Prevent
            #     > Connecting to same socket
            #     > Connecting to same type of socket
            #     > Connecting to self

            if item.socket != self.drag_start_socket \
                    and item.socket.type != self.drag_start_socket.type \
                    and item.socket.ge_node != self.drag_start_socket.ge_node:
                logger.debug(f"    Edge drag end on Socket {item}")

                # -> Remove/reset dragged edge
                self.dragged_edge.remove()
                self.dragged_edge = None

                # -> Create new edge
                if self.drag_start_socket.type == "module":
                    edge_type = 1
                else:
                    edge_type = 2

                if not self.graphics_scene.scene.check_has_edge(
                        start_socket=self.drag_start_socket,
                        end_socket=item.socket,
                    ):

                    GE_edge(
                        scene=self.graphics_scene.scene,
                        start_socket=self.drag_start_socket,
                        end_socket=item.socket,
                        edge_type=edge_type
                    )

                    # -> Record action in history
                    self.graphics_scene.scene.history.store_history("Create edge", set_modified=True)

                return True

        logger.debug("    Edge drag end on nothing")
        self.dragged_edge.remove()
        self.dragged_edge = None

        return False

    def _get_item_at_click(self, event):
        # -> Get item at click
        pos = event.pos()

        item = self.itemAt(pos)

        if isinstance(item, GE_edge_graphics):
            logger.debug(f"    Item at LMB click: {item.edge}, connecting:"
                  f" {item.edge.start_socket} <----> {item.edge.end_socket}")

        if type(item) is GE_socket_graphics:
            logger.debug(f"    Item at LMB click: Socket {item} has edges {item.socket.edges}")
        
        if item is not None:
            logger.debug(f"< Found {item} at click >")
        
        else:
            logger.debug("SCENE CONTENT:")
            logger.debug("> Nodes:")
            for node in self.graphics_scene.scene.nodes:
                logger.debug(f"    - {node}")

            logger.debug("> Edges:")
            for edge in self.graphics_scene.scene.edges:
                logger.debug(f"    - {edge}")

        # -> Return item
        return item

    def _cut_intersecting_edges(self, event):
        # ... for every point in the cutline
        for i in range(len(self.cutline.line_points) - 1):
            p1 = self.cutline.line_points[i]
            p2 = self.cutline.line_points[i + 1]

            # ... for every edge in the scene
            for edge in self.graphics_scene.scene.edges:
                # -> Check if edge is intersecting with cutline
                if edge.graphics.intersects_with(p1, p2):
                    edge.remove()
        
        # -> Record action in history
        self.graphics_scene.scene.history.store_history("Cut edges", set_modified=True)

    # ----- Right mouse
    def right_mouse_button_press(self, event):
        super().mousePressEvent(event)

    def right_mouse_button_release(self, event):
        super().mouseReleaseEvent(event)

    # ----- Wheel
    def wheelEvent(self, event) -> None:
        # -> If ctrl is pressed
        if event.modifiers() == Qt.ControlModifier:
            # -> Calculate zoom factor
            zoom_out_factor = 1 / self.zoom_in_factor

            # -> Calculate zoom
            if event.angleDelta().y() > 0:
                zoom_factor = self.zoom_in_factor
                self.zoom += self.zoom_step

            else:
                zoom_factor = zoom_out_factor
                self.zoom -= self.zoom_step

            self.set_zoom(zoom_factor=zoom_factor)

        else:
            super().wheelEvent(event)

    # - Environment
    def _set_sockets_highlight(self, scene_pos: QPointF,  state: bool, radius: int):
        """ Disable/enable socket highlight in a given radius"""
        # -> Define scan rectangle
        scan_rect = QRectF(
            scene_pos.x() - radius, scene_pos.y() - radius,
            radius * 2, radius * 2
        )

        # -> Retrieve all items provided in scan rectangle
        items = self.graphics_scene.items(scan_rect)

        # -> Retrieve all sockets in items
        items_filtered = []
        for item in items:
            if type(item) in socket_graphics_classes:
                items_filtered.append(item)
            
        for grSocket in items: grSocket._highlighted = state

        return items_filtered

    def _check_emit_item_selected(self):
        current_selected_items = self.graphics_scene.selectedItems()

        if current_selected_items != self.graphics_scene.scene._last_selected_items:
            if not current_selected_items:
                self.graphics_scene.itemsDeselected.emit()

            else:
                self.graphics_scene.itemSelected.emit()

            self.graphics_scene.scene._last_selected_items = current_selected_items

    # ================================================== Keyboard events
    def keyPressEvent(self, event) -> None:
        # -------------- Save/load events
        if event.key() == Qt.Key_L and event.modifiers() & Qt.ControlModifier:
            logger.debug(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Load")
            path = self.nucleus[self.graphics_scene.scene.namespace]["project_path"] + "/DAG.dag"    # TODO: Refactor session management
            self.graphics_scene.scene.load_from_file(
                path=path,
                pkl_block=False
            )
            self.graphics_scene.scene.history.store_history("Load DAG", set_modified=False)

        # -> Pass event to super (to apply default handlers for this event)
        else:
            super().keyPressEvent(event)

    def delete_selected_items(self):
        if self.parent_widget.lock_layout:
            return

        # -> Deleted all selected items
        deleted_nodes = False
        deleted_edges = False

        # -> Sort items by attribute, edges first, then nodes
        sorted_items = []

        for item in self.graphics_scene.selectedItems():
            if isinstance(item, GE_edge_graphics):
                sorted_items.append(item)

        for item in self.graphics_scene.selectedItems():
            if hasattr(item, "node"):
                sorted_items.append(item)

        for item in sorted_items:
            # -> Delete nodes
            if hasattr(item, "node"):
                item.node.remove()
                deleted_nodes = True

                main_logger.info(f"- Deleted node {item.node.ref}")

            # -> Delete edges
            elif isinstance(item, GE_edge_graphics):
                item.edge.remove()
                deleted_edges = True

                start_node = item.edge.start_socket.ge_node
                source_datastream = item.edge.start_socket.name

                end_node = item.edge.end_socket.ge_node
                end_datastream = item.edge.end_socket.name

                main_logger.info(f"> Connected {source_datastream} ({start_node.ref}) -> {end_datastream} ({end_node.ref})")

        # -> Record action in history
        if deleted_nodes and not deleted_edges:
            self.graphics_scene.scene.history.store_history("Delete node", set_modified=True)
            self.graphics_scene.itemsDeselected.emit()

        elif not deleted_nodes and deleted_edges:
            self.graphics_scene.scene.history.store_history("Delete edge", set_modified=True)
        
        elif deleted_nodes and deleted_edges:
            self.graphics_scene.scene.history.store_history("Delete multiple elements", set_modified=True)
            self.graphics_scene.itemsDeselected.emit()
            
        else:
            pass
