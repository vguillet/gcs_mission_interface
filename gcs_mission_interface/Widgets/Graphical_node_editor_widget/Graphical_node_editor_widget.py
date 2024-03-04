
##################################################################################################################

# Built-in/Generic Imports
import logging
import threading

# Libs
from src.GUI.Tools.UI_importer import uic
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from PySide6.QtWidgets import QWidget, QVBoxLayout, QGraphicsView, QGraphicsItem, QApplication

# Own modules
from src.GUI.Widgets.Graphical_node_editor_widget.GE_scene import GE_scene
from src.GUI.Widgets.Graphical_node_editor_widget.GE_graphics_view import GE_graphics_view
from src.GUI.Widgets.Graphical_node_editor_widget.GE_node import GE_node
from src.GUI.Widgets.Graphical_node_editor_widget.GE_edge import *
# from src.GUI.Widgets.Connection_dialog.Connection_dialog import Connection_dialog

from src.GUI.UI_singletons import Nucleus_singleton
from src.Backend.Utils.Block_loader import *

##################################################################################################################

logger = logging.getLogger("main.gui.graph_editor.main_widget")
main_logger = logging.getLogger(name="main")

EDITOR_MODE_EDIT = 0
EDITOR_MODE_VISUALISE = 1


class Graphical_node_editor_widget(QWidget):
    def __init__(self, namespace: str):
        super().__init__()

        # ===== Attributes
        self.nucleus = Nucleus_singleton()
        self.namespace = namespace

        # -> Load widget
        self.ui = uic.loadUi("src/GUI/Widgets/Graphical_node_editor_widget/Graphical_node_editor_widget.ui")
        
        # -> Create graphics scene
        self.scene = GE_scene(parent_widget=self, namespace=self.namespace)
        self.graphics_scene = self.scene.graphics_scene

        # -> Create graphics view
        self.view = GE_graphics_view(parent_widget=self, graphics_scene=self.graphics_scene)

        # -> Connect buttons and signals
        self.view.scenePosChanged.connect(self._update_mouse_location)
        self.ui.lock_layout.stateChanged.connect(self.toggle_lock_layout)
        self.ui.toggle_editor_mode.clicked.connect(self.toggle_editor_mode)

        # Zoom slider
        self.ui.zoom_slider.valueChanged.connect(self.set_zoom)
        self.ui.negative_zoom_increment.clicked.connect(lambda: self.increment_slider(value=self.ui.zoom_slider.singleStep()))
        self.ui.positive_zoom_increment.clicked.connect(lambda: self.increment_slider(value=-self.ui.zoom_slider.singleStep()))

        # -> Set slider style to remove selected range color
        self.ui.zoom_slider.setStyleSheet("""QRangeSlider::groove:horizontal {background: transparent;}""")

        self.scene.add_has_been_modified_listener(self.update_project_path)

        # -> Add view to layout
        self.ui.centralwidget_layout.addWidget(self.view)

        # -> Setup states
        self.editor_mode = EDITOR_MODE_EDIT
        self.lock_layout = False

        # -> Setup listeners
        self._lock_layout_listeners = []

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.ui)
        self.setLayout(self.layout)

        # -> Setup interface
        self.ui.toggle_editor_mode.setIcon(QIcon("src/GUI/Data/Assets/view.png"))

    def increment_slider(self, value: int):
        self.ui.zoom_slider.setValue(self.ui.zoom_slider.value() + value)

        # -> Get slider value percentage
        slider_range = self.ui.zoom_slider.maximum() - self.ui.zoom_slider.minimum()
        slider_value = self.ui.zoom_slider.value() - self.ui.zoom_slider.minimum()
        slider_value_percentage = slider_value / slider_range
        slider_value_percentage = 1 - slider_value_percentage

        self.ui.current_zoom_level.setText(f"{int(slider_value_percentage * 100)}%")

    def set_zoom(self, value: int):
        # -> Get slider value percentage
        slider_range = self.ui.zoom_slider.maximum() - self.ui.zoom_slider.minimum()
        slider_value = value - self.ui.zoom_slider.minimum()
        slider_value_percentage = slider_value / slider_range

        # -> Invert percentage
        slider_value_percentage = 1 - slider_value_percentage

        # -> Get zoom value from percentage and zoom range
        zoom_range = self.view.zoom_range
        self.view.zoom = zoom_range[0] + (zoom_range[1] - zoom_range[0]) * slider_value_percentage

        # -> Solve for zoom factor
        zoom_range = (0.15, self.view.zoom_in_factor)
        zoom_factor = zoom_range[0] + (zoom_range[1] - zoom_range[0]) * slider_value_percentage

        # -> Set zoom factor of view
        transform = QTransform()
        transform.scale(zoom_factor, zoom_factor)
        self.view.setTransform(transform)

        self.ui.current_zoom_level.setText(f"{int(slider_value_percentage * 100)}%")

        # print(zoom_factor, zoom_factor)
        # print(self.view.transform().m11(), self.view.transform().m22())

    def add_lock_layout_listener(self, callback):
        self._lock_layout_listeners.append(callback)

    def toggle_lock_layout(self, state: bool):
        # -> Update lock layout state
        self.lock_layout = state

        # -> Call all lock layout listeners with opposite of current state
        for callback in self._lock_layout_listeners: callback(state=state)

    def contextMenuEvent(self, event) -> None:
        try:
            item = self.scene.get_item_at(pos=event.pos())

            if hasattr(item, "node") or type(item) == QGraphicsProxyWidget:
                self._handle_node_context_menu(event=event)
            
            elif hasattr(item, "socket"):
                self._handle_socket_context_menu(event=event)

            elif hasattr(item, "edge"):
                self._handle_edge_context_menu(event=event)
            
            else:
                self._handle_default_context_menu(event=event)

            return super().contextMenuEvent(event)

        except Exception as e: return super().contextMenuEvent(event)

    def _handle_node_context_menu(self, event):
        # -> Get selection
        item = self.scene.get_item_at(pos=event.pos())

        if type(item) == QGraphicsProxyWidget:
            ge_node = item.widget().ge_node

        elif hasattr(item, "ge_node"):
            ge_node = item.ge_node

        elif hasattr(item, "node"):
            ge_node = item.node

        else:
            return

        # -> Set selection to ge_node
        if not ge_node.graphics.isSelected():
            self.graphics_scene.clearSelection()
            ge_node.graphics.setSelected(True)

        # -> Create context menu
        context_menu = QMenu(self)

        # -> Add actions
        # Run
        pixmap = QPixmap("src/GUI/Data/Assets/play.svg").scaled(15, 15)
        run_node_action = context_menu.addAction(QIcon(pixmap), "Run node")
        
        pixmap = QPixmap("src/GUI/Data/Assets/play_from.svg").scaled(15, 15)
        run_from_node_action = context_menu.addAction(QIcon(pixmap), "Run from node")
        context_menu.addSeparator()

        # Block
        refresh_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/refresh.png"), "Refresh block")
        clear_datastreams = context_menu.addAction(QIcon("src/GUI/Data/Assets/clear.png"), "Clear datastreams")
        context_menu.addSeparator()

        # Edit
        cut_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/cut.png"), "Cut")
        copy_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/copy.png"), "Copy")
        paste_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/paste.png"), "Paste")
        delete_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/delete.png"), "Delete")
        context_menu.addSeparator()

        # Flagging
        pixmap = QPixmap("src/GUI/Data/Assets/breakpoint.png").scaled(12, 12)
        add_breakpoint = context_menu.addAction(QIcon(pixmap), "Add breakpoint")
        remove_breakpoint = context_menu.addAction("Remove breakpoint")
        context_menu.addSeparator()

        flag_dirty = context_menu.addAction(QIcon("src/GUI/Data/Assets/flag_filled.png"), "Flag dirty")
        unflag_dirty = context_menu.addAction(QIcon("src/GUI/Data/Assets/flag_empty.png"), "Unflag dirty")

        # -> Map menu to event
        action = context_menu.exec_(self.mapToGlobal(event.pos()))

        # -> Apply choice
        if action == run_node_action:
            # -> Run node in thread
            thread = threading.Thread(target=ge_node.run)

            main_logger.info(f"Running node {ge_node.block.name}")
            thread.start()
            main_logger.info(f"Done running node {ge_node.block.name}")

        elif action == run_from_node_action:
            # TODO: Connect run from node action
            pass

        elif action == refresh_action:
            ge_node.refresh_block()

        elif action == clear_datastreams:
            # -> Reset states
            ge_node.dag_node.reset_states()

            # -> Clear datastreams
            ge_node.block.reset_datastreams(exceptions=["settings"])

        elif action == cut_action:
            self.scene.clipboard.cut()

        elif action == copy_action:
            self.scene.clipboard.copy()

        elif action == paste_action:
            self.scene.clipboard.paste()

        elif action == delete_action:
            self.view.delete_selected_items()

        elif action == add_breakpoint:
            ge_node.breakpoint = True
            self.scene.history.store_history(description="Add breakpoint", set_modified=True)

        elif action == remove_breakpoint:
            ge_node.breakpoint = False
            self.scene.history.store_history(description="Remove breakpoint", set_modified=True)

        elif action == flag_dirty:
            ge_node.dirty = True
            self.scene.history.store_history(description="Add dirty flag", set_modified=True)

        elif action == unflag_dirty:
            ge_node.dirty = False
            self.scene.history.store_history(description="Remove dirty flag", set_modified=True)

    def _handle_socket_context_menu(self, event):
        # -> Get selection
        item = self.scene.get_item_at(pos=event.pos())

        if hasattr(item, "socket"):
            ge_node = item.socket.ge_node
            datastream_ref = item.socket.name

        else:
            return

        # -> Set selection to ge_node
        if not ge_node.graphics.isSelected():
            self.graphics_scene.clearSelection()
            ge_node.graphics.setSelected(True)

        # -> Create context menu
        context_menu = QMenu(self)

        # -> Add actions
        # Edit
        cut_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/cut.png"), "Cut")
        copy_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/copy.png"), "Copy")
        paste_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/paste.png"), "Paste")
        delete_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/delete.png"), "Delete")
        context_menu.addSeparator()
        clear_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/clear.png"), "Clear all")
        context_menu.addSeparator()

        # Link editor
        open_link_editor = context_menu.addAction("Link editor")
        context_menu.addSeparator()

        # -> Map menu to event
        action = context_menu.exec_(self.mapToGlobal(event.pos()))

        # -> Apply choice
        if action == cut_action:
            self.scene.clipboard.cut()

        elif action == copy_action:
            self.scene.clipboard.copy()

        elif action == paste_action:
            self.scene.clipboard.paste()

        elif action == delete_action:
            self.view.delete_selected_items()

        elif action == clear_action:
            self.scene.clear()

        elif action == open_link_editor:
            self._open_link_editor(ge_node=ge_node, datastream_ref=datastream_ref)

    def _handle_edge_context_menu(self, event):
        # -> Get selection
        item = self.scene.get_item_at(pos=event.pos())

        if hasattr(item, "edge"):
            ge_edge = item.edge

        else:
            return

        # -> Set selection to ge_edge
        if not ge_edge.graphics.isSelected():
            self.graphics_scene.clearSelection()
            ge_edge.graphics.setSelected(True)

        # -> Create context menu
        context_menu = QMenu(self)

        # -> Add actions
        # Edit
        cut_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/cut.png"), "Cut")
        copy_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/copy.png"), "Copy")
        paste_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/paste.png"), "Paste")
        delete_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/delete.png"), "Delete")
        context_menu.addSeparator()
        clear_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/clear.png"), "Clear all")
        context_menu.addSeparator()

        # Link editor
        open_link_editor = context_menu.addAction("Link editor")
        context_menu.addSeparator()
        
        # Link style
        bezier_action = context_menu.addAction("Toggle Bezier edge")
        straight_action = context_menu.addAction("Toggle straight edge")

        # -> Map menu to event
        action = context_menu.exec_(self.mapToGlobal(event.pos()))

        # -> Apply choice
        if action == cut_action:
            self.scene.clipboard.cut()

        elif action == copy_action:
            self.scene.clipboard.copy()

        elif action == paste_action:
            self.scene.clipboard.paste()

        elif action == delete_action:
            self.view.delete_selected_items()

        elif action == clear_action:
            self.scene.clear()

        elif action == open_link_editor:
            pass

        elif action == bezier_action:
            ge_edge.edge_type = EDGE_TYPE_BEZIER

        elif action == straight_action:
            ge_edge.edge_type = EDGE_TYPE_DIRECT

    def _handle_default_context_menu(self, event):
        # -> Create context menu
        context_menu = QMenu(self)

        # -> Add actions
        # Edit
        cut_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/cut.png"), "Cut")
        copy_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/copy.png"), "Copy")
        paste_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/paste.png"), "Paste")
        delete_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/delete.png"), "Delete")
        context_menu.addSeparator()
        clear_action = context_menu.addAction(QIcon("src/GUI/Data/Assets/clear.png"), "Clear all")
        context_menu.addSeparator()

        # -> Map menu to event
        action = context_menu.exec_(self.mapToGlobal(event.pos()))

        # -> Apply choice
        if action == cut_action:
            self.scene.clipboard.cut()

        elif action == copy_action:
            self.scene.clipboard.copy()

        elif action == paste_action:
            self.scene.clipboard.paste()

        elif action == delete_action:
            self.view.delete_selected_items()

        elif action == clear_action:
            self.scene.clear()
    
    def _update_mouse_location(self):
        self.ui.mouse_location.setText(
            f"Scene Pos: ({int(self.view.last_scene_mouse_position.x())}, {int(self.view.last_scene_mouse_position.y())})")

    def _open_link_editor(self, ge_node, datastream_ref: str):
        print(">>>>>>>>>>> Opening link editor")
        return

        # -> Open dialog to edit links logic
        dialog = Connection_dialog(
            parent_module_reference=self.namespace,
            link=link
        )

        if dialog.exec():
            pass
        else:
            pass

    def add_node(self, library: str, category: str, block_name: str, pos=(0, 0)):
        if self.lock_layout:
            logger.warning("Cannot add node while layout is locked")
            return None

        # -> Load blocks dict
        block_class = get_block_class_by_name(
            library=library,
            category=category,
            block_name=block_name,
        )

        # -> Create block instance
        block = block_class()

        # -> Create front end node
        new_node = GE_node(
            scene=self.scene,
            namespace=self.namespace,
            block=block,
            position=pos
        )

        # -> Add node set edit mode to lock layout listeners
        self.add_lock_layout_listener(callback=new_node.set_edit_mode)

        self.scene.history.store_history("Add node", set_modified=True)

        main_logger.info(f"> Added node {block_name} ({category})")

        return new_node 

    def add_edge(self,
                 start_node: GE_node,
                 source_datastream: str,
                 end_node: GE_node,
                 end_datastream: str):

        if self.lock_layout:
            logger.warning("Cannot add edge while layout is locked")
            return None

        start_socket = start_node.get_datastream_socket(datastream_ref=source_datastream)
        end_socket = end_node.get_datastream_socket(datastream_ref=end_datastream)

        if not self.scene.check_has_edge(
            start_socket=start_socket,
            end_socket=end_socket
        ):

            GE_edge(self.scene,
                    start_socket=start_socket,
                    end_socket=end_socket
                    )

            self.scene.history.store_history("Add edge", set_modified=True)

            main_logger.info(f"> Connected {source_datastream} ({start_node.ref}) -> {end_datastream} ({end_node.ref})")

    def toggle_editor_mode(self):
        if self.editor_mode == EDITOR_MODE_EDIT:
            # -> Update editor mode
            self.editor_mode = EDITOR_MODE_VISUALISE

            # -> Update button icon
            self.ui.toggle_editor_mode.setIcon(QIcon("src/GUI/Data/Assets/edit.png"))

        else:
            # -> Update editor mode icon
            self.editor_mode = EDITOR_MODE_EDIT

            # -> Update button icon
            self.ui.toggle_editor_mode.setIcon(QIcon("src/GUI/Data/Assets/view.png"))

        self.ui.editor_stack.setCurrentIndex(self.editor_mode)

    def update_project_path(self):
        self.ui.project_path.setText(self.nucleus[self.namespace]["project_path"])    # TODO: Refactor session management

    def update_node_datastream_state(self, node: GE_node, datastream_ref: str, state: bool):
        node.set_datastream_ready_state(datastream_ref=datastream_ref, state=state)
