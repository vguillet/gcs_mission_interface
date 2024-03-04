
##################################################################################################################
"""

"""

# Built-in/Generic Imports
import logging

# Libs
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from src.GUI.Tools.UI_importer import uic

# Own modules
from src.Tools.Serializable import Serializable
from src.Tools.Importer import import_widget
from src.Backend.Utils.Block_loader import *

from src.GUI.UI_singletons import Nucleus_singleton

# from src.GUI.Widgets.Connection_dialog.Connection_dialog import Connection_dialog
from src.GUI.Widgets.Block_code_editor_dialog.Block_code_editor_dialog import Block_code_editor_dialog
from src.GUI.Widgets.Block_documentation_widget.Block_documentation_widget import Block_documentation_widget
from src.Tools.Importer import import_block_documentation

from src.Backend.Utils.Block_loader import *
from src.GUI.Widgets.User_input_widget.User_input.User_input_widget import UIWidget

##################################################################################################################

logger = logging.getLogger("main.gui.graph_editor.node_base_content_widget")


class GE_base_node_content_widget(QWidget, Serializable):
    def __init__(self, ge_node):
        super().__init__()

        # ===== Attributes
        self.namespace = ge_node.namespace
        self.nucleus = Nucleus_singleton()

        # -> Setup node properties
        self.ge_node = ge_node
        self.dag_node = ge_node.dag_node
        self.block_widgets = []

        # -> Setup UI
        self.ui = uic.loadUi("src/GUI/Widgets/Graphical_node_editor_widget/Base_node_content_widget.ui")
        self.setAttribute(Qt.WA_TranslucentBackground)

        self.ui.open_widget.setIcon(QIcon("src/GUI/Data/Assets/widget.png"))
        self.ui.open_logs.setIcon(QIcon("src/GUI/Data/Assets/logs.png"))
        self.ui.open_code.setIcon(QIcon("src/GUI/Data/Assets/code.png"))
        self.ui.open_doc.setIcon(QIcon("src/GUI/Data/Assets/documentation.png"))

        # -> Generate user input shortcut widget
        try:
            self.user_input_widget = self._create_settings_widget()

            self.block.add_datastream_update_listener(self._refresh_user_input_widget)

            self.user_input_widget.setAttribute(Qt.WA_TranslucentBackground)
            self.ui.custom_content_layout.addWidget(self.user_input_widget)

        except:
            self.user_input_widget = None

        # -> Add custom widget
        # # TODO: Integrate custom widget
        # if self.ge_node.block.preview_widget is not None:
        #     widget = self.dag_node.block.preview_widget()
        #     widget.setAttribute(Qt.WA_TranslucentBackground)
        #     widget.ge_node = self.ge_node
        #
        #     self.content.custom_content_layout.addWidget(widget)

        # -> Connect buttons
        self.ui.open_widget.clicked.connect(self.open_block_widget)
        self.ui.open_logs.clicked.connect(self.open_block_logs)
        self.ui.open_code.clicked.connect(self.open_code_editor_widget)
        self.ui.open_doc.clicked.connect(self.open_documentation_widget)

        # -> Setup layout
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.ui)
        self.setLayout(self.layout)

    @property
    def block(self):
        return self.dag_node.block

    @property
    def shape(self):
        # TODO: Include custom widget in shape once implemented
        if self.user_input_widget is not None:
            shape = self.user_input_widget.sizeHint().toTuple()
            return (shape[0] + 30, shape[1] + 120)

        else:
            return None

    def _create_settings_widget(self):
        set_datastream_data_callback = \
            lambda data: self.block.set_datastream_data(
                datastream_ref="settings",
                data=data,
                emit_signal=True
                )

        widget = UIWidget(definitions=self.block.datastreams_requirement["settings"],
                          definition_depth=0,
                          callback=set_datastream_data_callback
                          )

        # TODO: Clean up once user input can take in none data
        settings_datastream_data = self.block.get_datastream_data(datastream_ref="settings")
        if settings_datastream_data is not None:
            widget.set_values(values=settings_datastream_data)

        return widget

    def _refresh_user_input_widget(self, *args, **kwargs):
        # TODO: Clean up once user input can take in none data
        settings_datastream_data = self.block.get_datastream_data(datastream_ref="settings")
        if settings_datastream_data is not None:
            self.user_input_widget.set_values(values=settings_datastream_data)

    def open_block_widget(self):
        logger.debug(">>>>> Open block widget")

        try:
            # -> Fetch block widget
            widget = import_widget(block_name=self.dag_node.block_name)

            # -> Create viewer widget
            widget = widget()

            # -> Show viewer widget
            widget.show()

            # -> Add widget to list of widgets
            self.block_widgets.append(widget)

        except:
            pass

    def open_block_logs(self):
        logger.debug(">>>>> Open block logs")
        block_logs_text = get_block_logs(block_instance=self.dag_node.block)

        # -> Create logs widget
        widget = Block_documentation_widget(
            block_instance=self.dag_node.block,
            text=block_logs_text,
            dialogue_type="Logs",
            editable=False
        )

        # -> Show logs widget
        widget.show()

        # -> Add widget to list of widgets
        self.block_widgets.append(widget)

    def open_code_editor_widget(self):
        logger.debug(">>>>> Open code editor widget")

        dlg = Block_code_editor_dialog(block_instance=self.dag_node.block)

        if dlg.exec():
            pass
        else:
            pass

    def open_documentation_widget(self):
        logger.debug(">>>>> Open documentation widget")

        # -> Fetch block path
        block_path = get_block_directory_parent_folder_path_from_instance(block_instance=self.dag_node.block)

        # -> Fetch block documentation
        block_documentation_text = import_block_documentation(
            block_name=self.dag_node.block.name,
            block_directory_parent_folder_path=block_path
        )

        # -> Create documentation widget
        widget = Block_documentation_widget(
            block_instance=self.dag_node.block,
            text=block_documentation_text,
            dialogue_type="Documentation",
            editable=True
        )

        # -> Show documentation widget
        widget.show()

        # -> Add widget to list of widgets
        self.block_widgets.append(widget)

    # -------------------------------------------- Serialization
    def serialize(self):
        return {
            # "id": self.id # TODO: Review id serialize fix
        }

    def deserialize(self, data, hashmap={}):
        return False

