
import logging
from src.GUI.Widgets.Graphical_node_editor_widget.GE_edge import GE_edge_graphics

logger = logging.getLogger("main.gui.graph_editor.history")


class Scene_history:
    def __init__(self, scene) -> None:
        self.scene = scene

        self.stack = []
        self.pointer = -1
        self.stack_limit = 30   # TODO: Add to settings

        self._history_modified_listeners = []

    def reset_history(self):
        self.stack = []
        self.pointer = -1

        # -> Call all history modified listeners
        for callback in self._history_modified_listeners: callback(history_stack=self.stack, pointer=self.pointer)   

    def add_history_modified_listener(self, callback):
        self._history_modified_listeners.append(callback)

    def undo(self):
        logger.debug("Undo")

        # -> If items are present in the stack
        if self.pointer > 0:
            # -> Increment item backwards once 
            self.pointer -= 1

            # -> Restore state
            self.restore_history()

    def redo(self):
        logger.debug("Redo")

        if self.pointer + 1 < len(self.stack):
            # -> Increment item backwards once 
            self.pointer += 1

            # -> Restore state
            self.restore_history()

    def restore_history(self):
        logger.debug(f"Restoring history step: {self.pointer} / {len(self.stack)}")

        self.restore_history_stamp(self.stack[self.pointer])

        # -> Call all history modified listeners
        for callback in self._history_modified_listeners: callback(history_stack=self.stack, pointer=self.pointer)

    def store_history(self, description: str, set_modified: bool = False):       
        # If pointer is not at the end of the history stack
        if self.pointer + 1 < len(self.stack):
            # -> Remove all entries after the pointer
            self.stack = self.stack[0:self.pointer + 1]

        # If the history is outside the stack_limit
        if self.pointer + 1 >= self.stack_limit:
            # -> Remove first (oldest item) from the list
            self.stack = self.stack[1:]
            self.pointer -= 1

        # -> Creating history stamp
        hs = self.create_history_stamp(description=description)

        # -> Add history stamp to history stack
        self.stack.append(hs)

        # -> Increment history pointer
        self.pointer += 1

        logger.debug(f"Storing history step <{description}>: {self.pointer} / {len(self.stack)}")
        
        if set_modified:
            self.scene.has_been_modified = True
        
        # -> Call all history modified listeners
        for callback in self._history_modified_listeners: callback(history_stack=self.stack, pointer=self.pointer)
            
    def restore_history_stamp(self, history_stamp: dict):
        logger.debug(f"- HS: {history_stamp['description']}")

        self.scene.deserialize(
            data=history_stamp["snapshot"],
            path=None,
            pkl_block=False,
            restore_ids=True,
            restore_refs=True,
            additive=False,
        )

        for edge_id in history_stamp["selection"]["edges"]:
            for edge in self.scene.edges:
                if edge.id == edge_id:
                    edge.graphics.setSelected(True)
                    break
        
        for node_id in history_stamp["selection"]["nodes"]:
            for node in self.scene.nodes:
                if node.id == node_id:
                    node.graphics.setSelected(True)
                    break
        
    def create_history_stamp(self, description: str):
        sel_obj = {
            "nodes": [],
            "edges": []
        }

        for item in self.scene.graphics_scene.selectedItems():
            if hasattr(item, "node"):
                sel_obj["nodes"].append(item.node.id)

            elif isinstance(item, GE_edge_graphics):
                sel_obj["edges"].append(item.edge.id)

        history_stamp = {
            "description": description,
            "snapshot": self.scene.serialize(
                path=None,
                serialize_states=True,
                pkl_block=False,
                random_ids=True
            ),
            "selection": sel_obj
            }
        
        return history_stamp
