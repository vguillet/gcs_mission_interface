
##################################################################################################################
"""

"""

# Built-in/Generic Imports
import logging
import threading

# Libs

# Own modules
from src.Tools.Serializable import Serializable
from src.GUI.UI_singletons import Nucleus_singleton

from src.Backend.Utils.Block_loader import *
from src.Backend.Block_modules.Block_core import *
from src.GUI.Widgets.Graphical_node_editor_widget.GE_node_graphics import *
from src.GUI.Widgets.Graphical_node_editor_widget.GE_node_base_content_widget import GE_base_node_content_widget
from src.GUI.Widgets.Graphical_node_editor_widget.GE_socket import (
    GE_socket,
    LEFT_TOP, LEFT_BOTTOM,
    RIGHT_TOP, RIGHT_BOTTOM,
    MIDDLE_BOTTOM, MIDDLE_TOP
    )

##################################################################################################################

logger = logging.getLogger("main.gui.graph_editor.node")

# TODO: Add to settings
SOCKETS_SPACING = 25


class GE_node(Serializable):
    def __init__(
        self,
        scene,
        namespace: str,
        block=None,

        # DAG 
        order_dependencies: list = None, 
        data: dict = None,
        path: str = None,
        pkl_block: bool = False,
        restore_ref: bool = True,
        restore_id: bool = False,                      
        
        # GE
        position: tuple = None,
        hashmap: dict = {}
    ):

        Serializable.__init__(self, create_id=False)

        # ===== Attributes
        self.namespace = namespace
        self.nucleus = Nucleus_singleton()

        # -> Setup node properties
        self.scene = scene

        # GE node
        self.content = None
        self.comments = None
        self.graphics = None
        self._input_sockets = None
        self._output_sockets = None
        self._module_sockets = None
        self._modules_count = 0

        # -> Node states
        self._running = False
        self._error = False

        # DAG node
        self.init_DAG_node(
            block=block,
            order_dependencies=order_dependencies,
            data=data,
            path=path,
            pkl_block=pkl_block,
            restore_ref=restore_ref,
            restore_id=restore_id,
            hashmap=hashmap
        )

        # GE node
        # -> Position node
        if position is not None:
            self.set_position(position[0], position[1])

    def __str__(self):
        return f"Graph node ({self.block_name}): {self.ref} - ({str(self.id)[0:3]}..{str(self.id)[-3:]})"

    def __repr__(self) -> str:
        return self.__str__()

    # ----------------------------------------- Inits
    def init_DAG_node(self,
                      block=None,

                      # DAG
                      order_dependencies: list = None,
                      data: dict = None,
                      path: str = None,
                      pkl_block: bool = False,
                      restore_ref: bool = True,
                      restore_id: bool = False,

                      # GE
                      hashmap: dict = {}
                      ):

        # -> If initiating new node
        if data is None and block is not None:
            # -> Create DAG node
            self.dag_node = self.nucleus[self.scene.namespace]["DAG"].add_node(
                block=block,
                order_dependencies=order_dependencies,
                data=data,
                path=path,
                pkl_block=pkl_block,
                restore_ref=restore_ref
            )

            # -> Add DAG node listeners
            self._init_listeners()

            # -> Create GE graphics
            self._init_graphics()

        # -> If restoring node
        elif data is not None:
            self.deserialize(
                data=data,
                path=path,
                pkl_block=pkl_block,
                restore_ref=restore_ref,
                restore_id=restore_id,
                hashmap=hashmap
            )

        # Error
        elif data is None and block is None:
            logger.error("GE node invalid: data or block must be provided")

    def _init_graphics(self):
        # -> Load block graphics
        self.block_visuals = get_block_visuals_from_instance(block_instance=self.block)

        # -> Setup node content
        self.content = GE_base_node_content_widget(ge_node=self)

        body_color = self.block_visuals["NODE"]["body_color"]
        header_color = self.block_visuals["NODE"]["header_color"]

        # -> Get size of content widget
        graphics_shape = self.content.shape

        # -> If no shape is provided, use one from block config
        if graphics_shape is None:
            graphics_shape = self.block_visuals["NODE"]["shape"]

        # -> Load block config
        self.block_config = get_block_config_from_instance(block_instance=self.block)
        datastreams_types = get_block_datastreams_types_from_instance(block_instance=self.block)

        # -> Sort datastreams into inputs and outputs
        inputs = []
        outputs = []

        for datastream_ref, datastream_type in datastreams_types.items():
            if DATASTREAM_TYPE_INPUT in datastream_type:
                inputs.append(datastream_ref)

            if DATASTREAM_TYPE_OUTPUT in datastream_type:
                outputs.append(datastream_ref)

        # inputs = self.block_config["CONFIG"]["input_datastreams"]
        # outputs = self.block_config["CONFIG"]["output_datastreams"]
        modules = self.block_config["CONFIG"]["modules"]
        self._modules_count = len(modules)

        # -> Setup node graphics
        self.graphics = GE_node_graphics(
            node=self,
            graphics_shape=graphics_shape
        )

        # -> Setup node colors
        self.graphics.set_color(color=header_color, element=HEADER_ELEMENT)
        self.graphics.set_color(color=body_color, element=BODY_ELEMENT)

        # -> Setup node sockets
        self._input_sockets = []
        for i, input_name in enumerate(inputs):
            self._input_sockets.append(
                GE_socket(ge_node=self,
                          name=input_name,
                          index=i,
                          position=LEFT_TOP,
                          socket_type="input")
            )

        self._output_sockets = []
        for i, output_name in enumerate(outputs):
            self._output_sockets.append(
                GE_socket(ge_node=self,
                          name=output_name,
                          index=i,
                          position=RIGHT_TOP,
                          socket_type="output")
            )

        self._module_sockets = []
        for i, module_name in enumerate(modules):
            self._module_sockets.append(
                GE_socket(ge_node=self,
                          name=module_name,
                          index=i,
                          position=MIDDLE_BOTTOM,
                          socket_type="module")
            )

        # -> Add node to scene
        self.scene.add_node(self)
        self.scene.graphics_scene.addItem(self.graphics)

        # -> Setup node          !!! Order of call is important !!!
        self.ref = self.dag_node.ref

        # -> Setup node visual states
        self._set_visual_state()

    def _init_listeners(self):
        # Datastream update
        self.block.add_datastream_update_listener(callback=self._set_visual_state)

        # Ref
        self.block.add_ref_update_listener(self._update_ref)

        # Running
        self.dag_node.add_running_listener(callback=self._set_visual_state)

        # Done (can only be set by block itself)
        self.dag_node.add_done_toggled_listener(callback=self._set_visual_state)

        # Invalid (can only be set by block itself)
        self.dag_node.add_invalid_toggled_listener(callback=self._set_visual_state)

        # Breakpoint (bypass performance mode, set manually)
        self.dag_node.add_breakpoint_toggled_listener(callback=lambda placeholder: self._set_visual_state(manual=True))

        # Dirty (bypass performance mode, set manually)
        self.dag_node.add_dirty_toggled_listener(callback=lambda placeholder: self._set_visual_state(manual=True))

    # ----------------------------------------- States
    # -------------- GE level
    @property
    def error(self) -> bool: return self._error
    @error.setter
    def error(self, value: bool) -> None:
        self._error = value
        self._set_visual_state()

    # -------------- DAG state proxies
    @property
    def running(self) -> bool: 
        """ Proxy to DAG node running state """
        return self.dag_node.running
    @running.setter
    def running(self, value: bool) -> None:
        """ Proxy to DAG node running state """
        self.self.dag_node.running = value

    # Done
    @property
    def done(self) -> bool:
        """ Proxy to DAG node done state """
        return self.dag_node.done
    @done.setter
    def done(self, value: bool) -> None:
        """ Proxy to DAG node done state """
        self.dag_node.done = value

    # Breakpoint
    @property
    def breakpoint(self) -> bool:
        """ Proxy to DAG node breakpoint state """
        return self.dag_node.breakpoint
    @breakpoint.setter
    def breakpoint(self, value: bool) -> None:
        """ Proxy to DAG node breakpoint state """
        self.dag_node.breakpoint = value

    # Dirty
    @property
    def dirty(self) -> bool:
        """ Proxy to DAG node dirty state """
        return self.dag_node.dirty
    @dirty.setter
    def dirty(self, value: bool) -> None:
        """ Proxy to DAG node dirty state """
        self.dag_node.dirty = value

    # Invalid
    @property
    def invalid(self) -> bool:
        """ Proxy to DAG node invalid state """
        return self.dag_node.invalid
    @invalid.setter
    def invalid(self, value: bool) -> None:
        """ Proxy to DAG node invalid state """
        self.dag_node.invalid = value

    # Performance mode
    @property
    def performance_mode(self) -> bool:
        """ Proxy to DAG node performance state """
        return self.dag_node.performance_mode
    @performance_mode.setter
    def performance_mode(self, value: bool) -> None:
        """ Proxy to DAG node performance state """
        self.dag_node.performance_mode = value

    # Datastream update functions
    def set_datastream_ready_state(self, datastream_ref: str, state: bool) -> None:
        """ Proxy to DAG node set_datastream_ready_state """
        self.dag_node.set_datastream_ready_state(datastream_ref=datastream_ref, state=state)

    def reset_datastreams_ready(self) -> None:
        """ Proxy to DAG node reset_datastreams_ready """
        self.dag_node.reset_datastreams_ready()

    def reset_states(self) -> None:
        """ Proxy to DAG node reset_states """
        self.dag_node.reset_states()

        self.scene.history.store_history(
            description="Reset block states",
            set_modified=True
            )

    # -------------- Visual states
    def _set_visual_state(self, manual=False, *args, **kwargs) -> None:
        # -> Performance mode
        if self.dag_node.exhaustive_performance_mode and not manual:
            if self.invalid is True:
                self.graphics.state = "invalid"
            
            return
        
        # -> Regular mode
        else:
            # -> Set node breakpoint
            self.graphics.breakpoint = self.breakpoint

            # -> Set node state
            if self.running is True:
                self.graphics.state = "running"

            elif self.error is True:
                self.graphics.state = "error"

            elif self.done is True:
                self.graphics.state = "done"

            elif self.invalid is True:
                self.graphics.state = "invalid"

            elif self.dirty is True:
                self.graphics.state = "dirty"

            else:
                self.graphics.state = None

            # for socket in self._output_sockets:
            #     # -> Set node socket state
            #     socket.graphics.update()
            #
            #     for edge in socket.edges:
            #         # -> Set edge state
            #         edge.graphics.update()
            #
            #         # -> Set end socket state
            #         edge.end_socket.graphics.update()

    # ----------------------------------------- Properties
    # -------------- DAG properties proxies
    # Block
    @property
    def block(self):
        """ Proxy to DAG node block """
        return self.dag_node.block

    # Block name
    @property
    def block_name(self):
        """ Proxy to DAG node block name """
        return self.dag_node.block_name

    # ID
    @property
    def id(self):
        """ Proxy to DAG node id """
        return self.dag_node.id
    @id.setter
    def id(self, value):
        """ Proxy to DAG node id """
        self.dag_node.id = value

    # Ref
    @property
    def ref(self):
        """ Proxy to DAG node ref """
        return self.block.ref
    @ref.setter
    def ref(self, value):
        """ Proxy to DAG node ref """
        self.block.ref = value

    def _update_ref(self):
        self.graphics.header = f"{self.dag_node.ref}   ({self.block_name.replace('_', ' ').capitalize()})"

    # -------------- GE properties
    @property
    def header_color(self):
        """ Proxy to DAG node header color """
        return self.graphics.header_color

    @property
    def body_color(self):
        """ Proxy to DAG node body color """
        return self.graphics.body_color

    # -------------------------------------------- Get
    def get_datastream_socket(self, datastream_ref: str):
        for socket in self._input_sockets + self._output_sockets + self._module_sockets:
            if socket.name == datastream_ref:
                return socket
        else:
            logger.warning(f"{self} does not contain a socket with name <{datastream_ref}>")

    # -------------------------------------------- Remove
    def remove(self):
        # -> Remove dag node
        result = self.dag_node.dag.remove_node(node=self.dag_node)

        if result:
            # -> Remove all edges from sockets
            for socket in self._input_sockets + self._output_sockets + self._module_sockets:
                while len(socket.edges) > 0:
                    socket.edges[0].remove()

            # -> Remove graphics node
            logger.debug(f">>>>> Deleting GE node <{self}>")
            self.scene.graphics_scene.removeItem(self.graphics)

            # -> Remove node from scene
            self.scene.remove_node(self)

            return True

        else:
            return False

    # -------------------------------------------- Run
    def run(self) -> bool:
        logger.debug(">>>>> Running block")
        result = self.dag_node.run()

        for node in self.dag_node.get_descendants():
            node.pull_all()

        return result is not None
        
    # -------------------------------------------- Environment
    def refresh_block(self):
        self.dag_node.refresh_block()

    def update_connected_edges(self):
        for socket in self._input_sockets + self._output_sockets + self._module_sockets:
            for edge in socket.edges:
                edge.update_positions()

    def set_position(self, x, y):
        self.graphics.setPos(x, y)

    def set_color(self, color: str, element: int):
        if color is None:
            return
        else:
            self.graphics.set_color(color=color, element=element)

    def set_edit_mode(self, state: bool) -> None:
        # -> Set node behavior
        self.graphics.set_edit_mode(state=state)

    def get_socket_position(self, index: int, position: int):
        """
        Get the relative `x, y` position of a socket

        :param index: Order number of the Socket. (0, 1, 2, ...)
        :param position: `Socket Position Constant` describing where the Socket is located.
        # :param num_out_of: Total number of Sockets on this `Socket Position`
        :return: Position of described Socket on the `Node`
        """

        if position in (LEFT_TOP, LEFT_BOTTOM):
            x = 0
        
        elif position in (MIDDLE_BOTTOM, MIDDLE_TOP):    
            is_even = self._modules_count % 2 == 0

            # -> If index in lower half of list
            if index < self._modules_count / 2:
                x = self.graphics.width / 2 - (index * SOCKETS_SPACING) - 10 * is_even
            
            else:
                x = self.graphics.width / 2 + (index - int(self._modules_count / 2) + (1 * is_even)) * SOCKETS_SPACING - 10 * is_even

        else:
            x = self.graphics.width

        if position in (LEFT_BOTTOM, RIGHT_BOTTOM):
            y = self.graphics.height - self.graphics._header_height - index * SOCKETS_SPACING

        elif position in (MIDDLE_BOTTOM, MIDDLE_TOP):
            y = self.graphics.height

        else:
            y = self.graphics._header_height + 10 + index * SOCKETS_SPACING

        return x, y

    def get_socket_scene_position(self, socket):
        """
        Get absolute Socket position in the Scene

        :param socket: `Socket` which position we want to know
        :return: (x, y) Socket's scene position
        """
        nodepos = self.graphics.pos()
        socketpos = self.get_socket_position(
            index=socket.index, 
            position=socket.position
            )
        return (nodepos.x() + socketpos[0], nodepos.y() + socketpos[1])

    # -------------------------------------------- Serialization
    def serialize(self,
                  path: str = None,
                  serialize_states: list or bool = True,
                  pkl_block: bool = False,
                  random_id: bool = False
                  ) -> dict:

        # -> Serialize dag node
        data = self.dag_node.serialize(
                    serialize_states=serialize_states,
                    path=path,
                    pkl_block=pkl_block,
                    random_id=random_id
                    )

        # -> Add graphical node data
        data["graphics"] = {
                "header_color": self.graphics.header_color,
                "body_color": self.graphics.body_color,
                "pos_x": self.graphics.x(),
                "pos_y": self.graphics.y(),
                "shape": self.graphics.graphics_shape,
                "content": self.content.serialize(),
                "comments": self.comments,
                "inputs": [socket.serialize() for socket in self._input_sockets],
                "outputs": [socket.serialize() for socket in self._output_sockets],
                "modules": [socket.serialize() for socket in self._module_sockets]
            }

        return data

    def serialize_to_canvas(self, project_path: str) -> dict:
        # -> Extract project name
        project_name = project_path.split("/")[-1]

        # -> Convert color to rgb
        note_color = self.graphics.header_color.lstrip('#')

        # -> Serialize dag node to canvas
        data = {
            "id": self.id,
            "x": self.graphics.x()*2,
            "y": self.graphics.y()*2,
            "width": 400,
            "height": 500,
            "color": note_color,
            "type": "file",
            "file": f"docs/{self.ref}.md"
        }

        # -> Check if file exists
        if not os.path.exists(f"{project_path}/{data['file']}"):
            note_content = f"""---
tags: {project_name}, block
name: {self.ref}
---

- [[{self.dag_node.block_name}|Block documentation]]

# Notes

# Tasks

"""
            # -> Create file
            with open(f"{project_path}/{data['file']}", "w") as file:
                file.write(note_content)

        return data

    def deserialize(self,
                    data,

                    # DAG
                    path: str = None,
                    pkl_block: bool = False,
                    restore_ref: bool = True,

                    # GE
                    restore_id: bool = True,
                    hashmap: dict = {}
                    ):
        # -> Adding node to hashmap for restoring DAG structure
        hashmap[data["id"]] = self

        # ------------------------ Deserialize DAG node data
        # -> Create DAG node
        self.dag_node = self.nucleus[self.scene.namespace]["DAG"].add_node(
                    data=data,
                    path=path,
                    pkl_block=pkl_block,
                    restore_ref=restore_ref
            )

        # -> Add DAG node listeners
        self._init_listeners()

        # -> Create GE graphics
        self._init_graphics()

        self.set_position(data["graphics"]["pos_x"], data["graphics"]["pos_y"])
        self.set_color(color=data["graphics"]["header_color"], element=0)
        self.set_color(color=data["graphics"]["body_color"], element=1)
        self.comments = data["graphics"]["comments"]

        # ------------------------ Deserialize GE node data
        # -> Sort node data
        data["graphics"]["inputs"].sort(key=lambda socket: socket["index"] + socket["position"] * 10000)
        data["graphics"]["outputs"].sort(key=lambda socket: socket["index"] + socket["position"] * 10000)
        data["graphics"]["modules"].sort(key=lambda socket: socket["index"] + socket["position"] * 10000)

        # -> Create input sockets
        self._input_sockets = []

        for socket_data in data["graphics"]["inputs"]:
            new_socket = GE_socket(ge_node=self,
                                   name=socket_data["name"],
                                   index=socket_data["index"],
                                   position=socket_data["position"],
                                   socket_type="input",
                                   multi_edges=socket_data["multi_edges"]
                                   )

            new_socket.deserialize(
                data=socket_data,
                restore_id=restore_id,
                hashmap=hashmap)
            self._input_sockets.append(new_socket)

        # -> Create output sockets
        self._output_sockets = []

        for socket_data in data["graphics"]["outputs"]:
            new_socket = GE_socket(ge_node=self,
                                   name=socket_data["name"],
                                   index=socket_data["index"],
                                   position=socket_data["position"],
                                   socket_type="output")

            new_socket.deserialize(
                data=socket_data,
                restore_id=restore_id,
                hashmap=hashmap)
            self._output_sockets.append(new_socket)

        # -> Create module sockets
        self._module_sockets = []
        self._modules_count = len(data["graphics"]["modules"])

        for socket_data in data["graphics"]["modules"]:
            new_socket = GE_socket(ge_node=self,
                                   name=socket_data["name"],
                                   index=socket_data["index"],
                                   position=socket_data["position"],
                                   socket_type="module")

            new_socket.deserialize(
                data=socket_data,
                restore_id=restore_id,
                hashmap=hashmap)
            self._module_sockets.append(new_socket)

        return True
