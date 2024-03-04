
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os
import json
import logging

# Libs
from PySide6.QtCore import *
from PySide6.QtGui import *

# Own modules
from src.Tools.Serializable import Serializable
from src.GUI.Widgets.Graphical_node_editor_widget.GE_graphics_scene import GE_graphics_scene
from src.GUI.Widgets.Graphical_node_editor_widget.GE_scene_history import Scene_history
from src.GUI.Widgets.Graphical_node_editor_widget.GE_scene_clipboard import Scene_clipboard
from src.GUI.Widgets.Graphical_node_editor_widget.GE_node import GE_node as GE_node
from src.GUI.Widgets.Graphical_node_editor_widget.GE_edge import GE_edge as GE_edge

from src.GUI.UI_singletons import Nucleus_singleton

##################################################################################################################

logger = logging.getLogger("main.gui.graph_editor.scene")


class GE_scene(Serializable):
    # -> Signals
    hasBeenModified = Signal()

    def __init__(self, parent_widget, namespace: str = None):
        Serializable.__init__(self)

        self.parent_widget = parent_widget

        # ===== Attributes
        self.namespace = namespace
        self.nucleus = Nucleus_singleton()

        self.nodes = []
        self.edges = []

        self._canvas_width = 64000
        self._canvas_height = 64000

        # -> Setup graphic scene
        self.graphics_scene = GE_graphics_scene(scene=self)

        self.graphics_scene.set_scene(
            width=self._canvas_width,
            height=self._canvas_height
        )

        # -> Setup scene history
        self.history = Scene_history(scene=self)

        # -> Setup scene clipboard
        self.clipboard = Scene_clipboard(scene=self, namespace=self.namespace)

        # -> Setup states
        self._has_been_modified = False
        self._last_selected_items = []

        # -> Setup listeners
        self._has_been_modified_listeners = []
        self._item_selected_listeners = []
        self._items_deselected_listeners = []

        # -> Connect signals
        self.graphics_scene.itemSelected.connect(self.on_item_selected)
        self.graphics_scene.itemsDeselected.connect(self.on_items_deselected)

    def selectedItems(self):
        return self.graphics_scene.selectedItems()

    # -------------------------------------------- Add
    def add_node(self, node):
        self.nodes.append(node)

    def add_edge(self, edge):
        self.edges.append(edge)

    # -------------------------------------------- Remove
    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)
        else:
            logger.warning(f"Trying to remove a node ({node}) not present in scene")

    def remove_edge(self, edge):
        if edge in self.edges:
            self.edges.remove(edge)
        else:
            logger.warning(f"Trying to remove an edge ({edge}) not present in scene")

    # -------------------------------------------- Misc
    def check_has_edge(self, start_socket, end_socket) -> bool:
        for edge in self.edges:
            if edge.start_socket == start_socket and edge.end_socket == end_socket \
                    or edge.end_socket == start_socket and edge.start_socket == end_socket:
                return True

        return False
    
    def get_item_at(self, pos):
        return self.view.itemAt(pos)
    
    @property
    def view(self):
        return self.graphics_scene.view

    def check_is_modified(self) -> bool:
        return self.has_been_modified

    @property
    def has_been_modified(self):
        return self._has_been_modified
    @has_been_modified.setter
    def has_been_modified(self, value):
        self._has_been_modified = value
        
        for callback in self._has_been_modified_listeners: callback()

        # -> Save canvas to canvas
        if self.parent_widget.ui.canvas_sync.isChecked():
            self.save_to_canvas()

    def add_has_been_modified_listener(self, callback):
        self._has_been_modified_listeners.append(callback)

    def on_item_selected(self):
        logger.debug("~~~~~ Scene got on item selected")
        
        current_selected_items = self.selectedItems()

        if current_selected_items != self._last_selected_items:
            self._last_selected_items = current_selected_items

            # -> Store history
            self.history.store_history("Select", set_modified=False)

            for callback in self._item_selected_listeners: callback(selection=current_selected_items)

    def add_item_selected_listener(self, callback):
        self._item_selected_listeners.append(callback)

    def on_items_deselected(self):
        logger.debug("~~~~~ Scene got on items deselected")

        if self._last_selected_items:
            self._last_selected_items = []

            # -> Store history
            self.history.store_history("Deselect", set_modified=False)

            for callback in self._items_deselected_listeners: callback()

    def add_items_deselected_listeners(self, callback):
        self._items_deselected_listeners.append(callback)

    def add_drag_enter_listener(self, callback):
        self.view.add_drag_enter_listener(callback=callback)

    def add_drop_listener(self, callback):
        self.view.add_drop_listener(callback=callback)

    def reset(self):
        self._clear()
        self.has_been_modified = False
        self.history.reset_history()

    def clear(self):
        self._clear()
        self.history.store_history(
            description="Clear scene",
            set_modified=True
        )
 
    def _clear(self):
        self.nucleus[self.namespace]["DAG"].reset_ids()

        result = True
        while len(self.nodes) > 0 and result:
            logger.debug(f"Removing: {self.nodes[0]}")
            result = self.nodes[0].remove()

            logger.debug(f"Nodes left: {self.nodes}\n")

        if len(self.nodes) != 0:
            logger.warning("Scene could not be fully cleared")

        self.has_been_modified = False

    def reset_last_selected_state(self):
        """
        Custom flag flag to detect node or edge has been selected
        """

        for node in self.nodes:
            node.graphics._last_selected_state = False
        
        for edge in self.edges:
            edge.graphics._last_selected_state = False

    def update_scene_states(self):
        pass

    # -------------------------------------------- Serialization
    def serialize(self,
                  path: str = None,
                  serialize_states: list or bool = True,
                  pkl_block: bool = False,
                  random_ids: bool = False
                  ) -> dict:

        return {
            "id": self.id,
            "scene_width": self._canvas_width,
            "scene_height": self._canvas_height,
            "nodes": [node.serialize(
                serialize_states=serialize_states,
                path=path,
                pkl_block=pkl_block,
                random_id=random_ids
            ) for node in self.nodes],
            "edges": [edge.serialize() for edge in self.edges]
        }

    def serialize_to_canvas(self):
        data = {
            "nodes": [node.serialize_to_canvas(project_path=self.nucleus[self.namespace]['project_path']) for node in self.nodes],
            "edges": [edge.serialize_to_canvas() for edge in self.edges]
        }

        return data

    def deserialize(self,
                    data,
                    path: str = None,
                    pkl_block: bool = False,
                    restore_refs: bool = True,
                    restore_ids: bool = True,
                    additive: bool = False,
                    hashmap={}
                    ):

        # -> Clear GUI
        if not additive:
            self._clear()

        # -> Deserialize graph/scene
        if restore_ids:
            self.id = data["id"]

        self._canvas_width = data["scene_width"]
        self._canvas_height = data["scene_height"]

        # -> Create nodes
        for node_data in data["nodes"]:
            new_node = GE_node(scene=self,
                               namespace=self.namespace,
                               data=node_data,
                               path=path,
                               pkl_block=pkl_block,
                               restore_ref=restore_refs,
                               restore_id=restore_ids,
                               hashmap=hashmap
                               )

            # -> Add node set edit mode to lock layout listeners
            new_node.set_edit_mode(state=self.parent_widget.lock_layout)
            self.parent_widget.add_lock_layout_listener(callback=new_node.set_edit_mode)

        # -> Create edges
        for edge_data in data["edges"]:
            GE_edge(scene=self).deserialize(
                                    data=edge_data,
                                    restore_id=restore_ids,
                                    hashmap=hashmap)

        # -> Reset block ids
        if not restore_ids:
            self.nucleus[self.namespace]["DAG"].reset_ids()

        return True

    # -------------------------------------------- File management
    def save_to_file(self,
                     path: str,
                     name: str = "DAG",
                     serialize_states: list or bool = True,
                     pkl_block: bool = False,
                     random_ids: bool = False
                     ):

        logger.debug(f">>>>> Saving to file ({path})")

        # -> Create DAG directory
        if not os.path.exists(path):
            os.makedirs(path)

        with open(f"{path}/{name}.dag", "w") as file:
            data = self.serialize(
                path=path,
                serialize_states=serialize_states,
                pkl_block=pkl_block,
                random_ids=random_ids
            )
            json.dump(data, file, indent=4, sort_keys=False)

        logger.debug(f"Saved scene to file: {path}")
        self.has_been_modified = False

        try:
            with open(f"{path}/{name}.dag", "w") as file:
                data = self.serialize(
                    path=path,
                    serialize_states=serialize_states,
                    pkl_block=pkl_block,
                    random_ids=random_ids
                )
                json.dump(data, file, indent=4, sort_keys=False)

            logger.debug(f"Saved scene to file: {path}")
            self.has_been_modified = False
            
        except:
            logger.warning(f"Unable to save scene to file ({path})")

    def save_to_canvas(self):
        if self.nucleus[self.namespace]['project_path'] is None:
            return

        # -> Get project path
        path = self.nucleus[self.namespace]['project_path']
        project_name = self.nucleus[self.namespace]['project_name']

        # -> Check if doc folder exists
        if not os.path.exists(f"{path}/docs"):
            os.makedirs(f"{path}/docs")

        # -> Serialize scene to canvas
        data = self.serialize_to_canvas()

        # -> Save data to .canvas file
        with open(f"{path}/docs/{project_name}.canvas", "w") as file:
            json.dump(data, file, indent=4, sort_keys=False)

    def load_from_file(self,
                       path: str,
                       pkl_block: bool = False,
                       restore_ids: bool = True,
                       restore_refs: bool = True,
                       additive: bool = False,
                       ):
        
        logger.debug(f">>>>> Loading from file ({path})")

        with open(path, "r") as file:
                data = json.load(file)

                directory_path = "/".join(path.split("/")[:-1])

                self.deserialize(
                    data=data,
                    path=directory_path,
                    pkl_block=pkl_block,
                    restore_ids=restore_ids,
                    restore_refs=restore_refs,
                    additive=additive
                )

        try:
            with open(path, "r") as file:
                data = json.load(file)

                directory_path = "/".join(path.split("/")[:-1])

                self.deserialize(
                    data=data,
                    path=directory_path,
                    pkl_block=pkl_block,
                    restore_ids=restore_ids,
                    restore_refs=restore_refs,
                    additive=additive
                )
            
            logger.debug(f"Loaded scene from file: {path}")
            self.has_been_modified = False

        except:
            logger.error(f"Something went wrong while loading DAG from {path},"
                         f"Possible source of error:" 
                         "         > File not found"
                         "         > Serialization failure")
