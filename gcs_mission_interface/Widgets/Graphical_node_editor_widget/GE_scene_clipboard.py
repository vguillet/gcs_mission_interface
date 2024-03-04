
import json
import logging

from PySide6.QtWidgets import QApplication

from src.GUI.Widgets.Graphical_node_editor_widget.GE_edge import GE_edge, GE_edge_graphics
from src.GUI.Widgets.Graphical_node_editor_widget.GE_node import GE_node as GE_node

from src.GUI.UI_singletons import Nucleus_singleton

logger = logging.getLogger("main.gui.graph_editor.clipboard")


class Scene_clipboard:
    def __init__(self, scene, namespace: str):

        # ===== Attributes
        self.namespace = namespace
        self.nucleus = Nucleus_singleton()

        self.scene = scene

        self.nodes = []
        self.edges = []

    def cut(self):
        if self.scene.parent_widget.lock_layout:
            return

        logger.debug(">>>>> Cutting")

        # -> Serialize selected
        data = self.serialize_selected(delete=True)
        str_data = json.dumps(data, indent=4, sort_keys=False)

        # -> Store serialized data in clipboard
        QApplication.instance().clipboard().setText(str_data)

    def copy(self):
        if self.scene.parent_widget.lock_layout:
            return

        logger.debug(">>>>> Copying")

        # -> Serialize selected
        data = self.serialize_selected(delete=False)
        str_data = json.dumps(data, indent=4, sort_keys=False)

        # -> Store serialized data in clipboard
        QApplication.instance().clipboard().setText(str_data)

    def paste(self):
        if self.scene.parent_widget.lock_layout:
            return

        logger.debug(">>>>> Pasting")

        # -> Retrieve serialized data from clipboard
        raw_data = QApplication.instance().clipboard().text()
        try:
            data = json.loads(raw_data)
        except ValueError as e:
            logger.debug("Could not load clipboard data, clipboard data is not valid JSON", e)
            return

        # -> Check if json data is valid
        if "nodes" not in data.keys():
            logger.debug("json does not contain nodes in keys")
            return

        # -> Deserialize data
        self.deserialize_clipboard_content(data=data)

    def serialize_selected(self, delete: bool = True) -> dict:
        logger.debug("Serializing selected to clipboard")

        # -> Get selected items
        selected_items = self.scene.graphics_scene.selectedItems()

        # -> Create empty storage lists
        sel_nodes = []
        sel_edges = []
        sel_sockets = {}

        # -> Sort edges and nodes
        for item in selected_items:
            if hasattr(item, "node"):
                sel_nodes.append(item.node.serialize())

                for socket in (item.node._input_sockets + item.node._output_sockets):
                    sel_sockets[socket.id] = socket

            elif isinstance(item, GE_edge_graphics):
                sel_edges.append(item.edge)

        # -> Remove all edges connecting to nodes not in out node list
        edges_to_remove = []

        for edge in sel_edges:
            if edge.start_socket.id in sel_sockets and edge.end_socket.id in sel_sockets:
                pass
            else:
                logger.debug(f"       > Edge {edge.id} is not fully connected to selected nodes")
                edges_to_remove.append(edge)

        for edge in edges_to_remove:
            sel_edges.remove(edge)

        # -> Serialize final list of edges
        sel_edges = [edge.serialize() for edge in sel_edges]

        data = {
            "nodes": sel_nodes,
            "edges": sel_edges
        }

        if delete:
            # -> Delete selected items if delete is True
            self.scene.view.delete_selected_items()

            # -> Store history
            self.scene.history.store_history(description="Cut to clipboard", set_modified=True)

        return data

    def deserialize_clipboard_content(self, data):
        logger.debug(f"Deserializing clipboard content {data}")

        hashmap = {}

        # -> Calculate mouse pointer - scene positions
        view = self.scene.view
        mouse_scene_pos = view.last_scene_mouse_position

        # -> Calculate selected objects bbox and center
        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0

        for node_data in data["nodes"]:
            x = node_data["graphics"]["pos_x"]
            y = node_data["graphics"]["pos_y"]

            if x < min_x: min_x = x
            if x > max_x: max_x = x

            if y < min_y: min_y = y
            if y > max_y: max_y = y

        bbox_center_x = (min_x + max_x) / 2
        bbox_center_y = (min_y + max_y) / 2

        # center = view.mapToScene(view.rect().center())

        # -> Calculate offset from center to mouse pointer
        offset_x = mouse_scene_pos.x() - bbox_center_x
        offset_y = mouse_scene_pos.y() - bbox_center_y

        # -> Create nodes
        for node_data in data["nodes"]:
            new_node = GE_node(scene=self.scene,
                               namespace=self.namespace,
                               data=node_data,
                               path=None,
                               pkl_block=False,
                               restore_ref=True, 
                               restore_id=False,
                               hashmap=hashmap
                               )

            # -> Add node set edit mode to lock layout listeners
            new_node.set_edit_mode(state=self.scene.parent_widget.lock_layout)
            self.scene.parent_widget.add_lock_layout_listener(callback=new_node.set_edit_mode)

            # -> Adjust new node position
            pos = new_node.graphics.pos()
            new_node.set_position(pos.x() + offset_x, pos.y() + offset_y)

        # -> Create edges
        for edge_data in data["edges"]:
            GE_edge(scene=self.scene).deserialize(
                data=edge_data,
                restore_id=False,
                hashmap=hashmap)

        # -> Reset block ids
        self.nucleus[self.scene.namespace]["DAG"].reset_ids()

        # -> Store history
        self.scene.history.store_history(description="Paste from clipboard", set_modified=True)

        return True