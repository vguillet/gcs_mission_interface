
##################################################################################################################

from socket import socket
from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *

from src.GUI.Widgets.Graphical_node_editor_widget.GE_socket import socket_graphics_classes

##################################################################################################################


class Edge_snapping:
    def __init__(self, view, snapping_radius: float = 24) -> None:
        self.view = view
        self.graphics_scene = self.view.graphics_scene
        self.snapping_radius = snapping_radius

    def get_snapped_socket_item(self, event: QMouseEvent, start_socket: socket):
        scene_pos = self.view.mapToScene(event.pos())

        gr_socket, _ = self.get_snapped_socket_position(scene_pos=scene_pos, start_socket=start_socket)

        return gr_socket 

    def get_snapped_socket_position(self, scene_pos: QPointF, start_socket: socket):
        """
        Return GrSocket and scene position to the nearest socket
        or original position if no socket is found
        """
        
        # -> Define scan rectangle
        scan_rect = QRectF(
            scene_pos.x() - self.snapping_radius, scene_pos.y() - self.snapping_radius,
            self.snapping_radius * 2, self.snapping_radius * 2
        )

        # -> Retrieve all items provided in scan rectangle
        items = self.graphics_scene.items(scan_rect)

        # -> Retrieve all sockets in items
        items_filtered = []
        for item in items:
            if type(item) in socket_graphics_classes:
                items_filtered.append(item)

        # -> Retrieve specific socket
        if len(items_filtered) == 0:
            return None, scene_pos

        selected_item = items_filtered[0]

        if len(items_filtered) > 1:
            # -> Solve for nearest socket
            nearest = 10000000000000
            for grSocket in items_filtered:
                # -> Get socket position
                grSocket_pos = grSocket.socket.ge_node.get_socket_scene_position(socket=grSocket.socket)
                
                # -> Solve for socket pos to mouse vector
                gpdist = QPointF(*grSocket_pos) - scene_pos

                # -> Solve for vector magnitude
                distance = gpdist.x() ** 2 + gpdist.y() ** 2 
                
                # -> Check if socket is closest
                if distance > nearest:
                    nearest = distance
                    selected_item = grSocket

        if start_socket is not None:
            if selected_item == start_socket \
                    or selected_item.socket.type == start_socket.type \
                    or selected_item.socket.ge_node == start_socket.ge_node \
                    or self.graphics_scene.scene.check_has_edge(
                                                        start_socket=start_socket,
                                                        end_socket=selected_item.socket,
                                                        ):
                return None, scene_pos

        # -> Get selected socket
        calc_pos = selected_item.socket.ge_node.get_socket_scene_position(socket=selected_item.socket)

        # -> Toggle hover effect
        # selected_item.highlighted = True

        return selected_item, QPointF(*calc_pos)
