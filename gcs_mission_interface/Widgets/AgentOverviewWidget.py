
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os

# Libs
from pandas import to_datetime
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtWidgets import QVBoxLayout, QWidget

# Own modules
from ..ui_loader import uic

##################################################################################################################


class AgentOverviewWidget(QWidget):
    def __init__(self, agent_id, ros_node):
        super().__init__()

        self.agent_id = agent_id
        self.ros_node = ros_node

        # ----------------------------------- Load GUI
        # -> Load ui singleton
        self.ui = uic.loadUi(os.getcwd() + "/src/gcs_mission_interface/gcs_mission_interface/resources/uis/robot_overview.ui")

        # > Update header
        self.__update_header()
        self.update()

        # -> Setup layout
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.ui)
        self.setLayout(self.layout)

    @property
    def current_interface_view(self) -> dict:
        current_interface_view = {}

        # -> tabWidget
        current_interface_view["tabWidget_index"] = self.ui.tabWidget.currentIndex()

        # -> toolBox
        current_interface_view["toolBox_index"] = self.ui.toolBox.currentIndex()

        # -> tableWidgets
        # > tabWidget_allocation_state
        current_interface_view[
            "tabWidget_allocation_state_index"] = self.ui.tabWidget_allocation_state.currentIndex()

        # > tableWidget_task_list_x
        current_interface_view[
            "tableWidget_task_list_x_scroll_horizontal"] = self.ui.tableWidget_task_list_x.horizontalScrollBar().value()
        current_interface_view[
            "tableWidget_task_list_y_scroll_vertical"] = self.ui.tableWidget_task_list_x.verticalScrollBar().value()

        # > tableWidget_local_bids_c
        current_interface_view[
            "tableWidget_local_bids_c_scroll_horizontal"] = self.ui.tableWidget_local_bids_c.horizontalScrollBar().value()
        current_interface_view[
            "tableWidget_local_bids_c_scroll_vertical"] = self.ui.tableWidget_local_bids_c.verticalScrollBar().value()

        # > tableWidget_local_allocations_d
        current_interface_view[
            "tableWidget_local_allocations_d_scroll_horizontal"] = self.ui.tableWidget_local_allocations_d.horizontalScrollBar().value()
        current_interface_view[
            "tableWidget_local_allocations_d_scroll_vertical"] = self.ui.tableWidget_local_allocations_d.verticalScrollBar().value()

        # > tableWidget_winning_bids_y
        current_interface_view[
            "tableWidget_winning_bids_y_scroll_horizontal"] = self.ui.tableWidget_winning_bids_y.horizontalScrollBar().value()
        current_interface_view[
            "tableWidget_winning_bids_y_scroll_vertical"] = self.ui.tableWidget_winning_bids_y.verticalScrollBar().value()

        # > tableWidget_shared_bids_f
        current_interface_view[
            "tableWidget_shared_bids_f_scroll_horizontal"] = self.ui.tableWidget_shared_bids_f.horizontalScrollBar().value()
        current_interface_view[
            "tableWidget_shared_bids_f_scroll_vertical"] = self.ui.tableWidget_shared_bids_f.verticalScrollBar().value()

        # > tableWidget_shared_bids_priority_beta
        current_interface_view[
            "tableWidget_shared_bids_priority_beta_scroll_horizontal"] = self.ui.tableWidget_shared_bids_priority_beta.horizontalScrollBar().value()
        current_interface_view[
            "tableWidget_shared_bids_priority_beta_scroll_vertical"] = self.ui.tableWidget_shared_bids_priority_beta.verticalScrollBar().value()

        # > tableWidget_shared_allocations_a
        current_interface_view[
            "tableWidget_shared_allocations_a_scroll_horizontal"] = self.ui.tableWidget_shared_allocations_a.horizontalScrollBar().value()
        current_interface_view[
            "tableWidget_shared_allocations_a_scroll_vertical"] = self.ui.tableWidget_shared_allocations_a.verticalScrollBar().value()

        # > tableWidget_shared_allocation_priority_alpha
        current_interface_view[
            "tableWidget_current_allocation_priority_alpha_scroll_horizontal"] = self.ui.tableWidget_shared_allocation_priority_alpha.horizontalScrollBar().value()
        current_interface_view[
            "tableWidget_current_allocation_priority_alpha_scroll_vertical"] = self.ui.tableWidget_shared_allocation_priority_alpha.verticalScrollBar().value()

        return current_interface_view

    @current_interface_view.setter
    def current_interface_view(self, current_interface_view) -> None:
        # > tabWidget
        self.ui.tabWidget.setCurrentIndex(current_interface_view["tabWidget_index"])

        # > toolBox
        self.ui.toolBox.setCurrentIndex(current_interface_view["toolBox_index"])

        # -> tableWidgets
        # > tabWidget_allocation_state
        self.ui.tabWidget_allocation_state.setCurrentIndex(current_interface_view["tabWidget_allocation_state_index"])

        # > tableWidget_task_list_x
        self.ui.tableWidget_task_list_x.horizontalScrollBar().setValue(current_interface_view["tableWidget_task_list_x_scroll_horizontal"])
        self.ui.tableWidget_task_list_x.verticalScrollBar().setValue(current_interface_view["tableWidget_task_list_y_scroll_vertical"])

        # > tableWidget_local_bids_c
        self.ui.tableWidget_local_bids_c.horizontalScrollBar().setValue(current_interface_view["tableWidget_local_bids_c_scroll_horizontal"])
        self.ui.tableWidget_local_bids_c.verticalScrollBar().setValue(current_interface_view["tableWidget_local_bids_c_scroll_vertical"])

        # > tableWidget_local_allocations_d
        self.ui.tableWidget_local_allocations_d.horizontalScrollBar().setValue(current_interface_view["tableWidget_local_allocations_d_scroll_horizontal"])
        self.ui.tableWidget_local_allocations_d.verticalScrollBar().setValue(current_interface_view["tableWidget_local_allocations_d_scroll_vertical"])

        # > tableWidget_winning_bids_y
        self.ui.tableWidget_winning_bids_y.horizontalScrollBar().setValue(current_interface_view["tableWidget_winning_bids_y_scroll_horizontal"])
        self.ui.tableWidget_winning_bids_y.verticalScrollBar().setValue(current_interface_view["tableWidget_winning_bids_y_scroll_vertical"])

        # > tableWidget_shared_bids_f
        self.ui.tableWidget_shared_bids_f.horizontalScrollBar().setValue(current_interface_view["tableWidget_shared_bids_f_scroll_horizontal"])
        self.ui.tableWidget_shared_bids_f.verticalScrollBar().setValue(current_interface_view["tableWidget_shared_bids_f_scroll_vertical"])

        # > tableWidget_shared_bids_priority_beta
        self.ui.tableWidget_shared_bids_priority_beta.horizontalScrollBar().setValue(current_interface_view["tableWidget_shared_bids_priority_beta_scroll_horizontal"])
        self.ui.tableWidget_shared_bids_priority_beta.verticalScrollBar().setValue(current_interface_view["tableWidget_shared_bids_priority_beta_scroll_vertical"])

        # > tableWidget_shared_allocations_a
        self.ui.tableWidget_shared_allocations_a.horizontalScrollBar().setValue(current_interface_view["tableWidget_shared_allocations_a_scroll_horizontal"])
        self.ui.tableWidget_shared_allocations_a.verticalScrollBar().setValue(current_interface_view["tableWidget_shared_allocations_a_scroll_vertical"])

        # > tableWidget_shared_allocation_priority_alpha
        self.ui.tableWidget_shared_allocation_priority_alpha.horizontalScrollBar().setValue(current_interface_view["tableWidget_current_allocation_priority_alpha_scroll_horizontal"])
        self.ui.tableWidget_shared_allocation_priority_alpha.verticalScrollBar().setValue(current_interface_view["tableWidget_current_allocation_priority_alpha_scroll_vertical"])

    def refresh(self, current_interface_view):
        self.update(current_interface_view)

    def __update_header(self):
        # -> Update agent details
        self.ui.label_agent_id.setText(str(self.ros_node.fleet[self.agent_id].id))
        self.ui.label_agent_class.setText(self.ros_node.fleet[self.agent_id].agent_class)

    def add_to_logs(self, msg):
        # -> If the message is not for this agent, return
        if msg["target"] != self.ros_node.fleet[self.agent_id].id and msg["target"] != "all":
            return

        # -> Construct log message
        log = ""
        # log += "----------------------------"
        log += f"\nReceived fleet msg ({msg['meta_action']}) from {msg['source']} (sent at {msg['timestamp']})"
        # log += f"\n{pformat(msg['memo'])}"

        # -> Add to logs
        self.ui.plainTextEdit_logs.appendPlainText(log)

    def update(self, *args, **kwargs):
        # -> Agent state
        # > Last update timestamp
        last_update_timestamp = to_datetime(self.ros_node.fleet[self.agent_id].state.timestamp, unit="s")
        last_update_timestamp = last_update_timestamp.replace(microsecond=0, nanosecond=0)

        self.ui.label_last_update_timestamp.setText(str(last_update_timestamp))

        # > Battery level
        self.ui.progressBar_battery_level.setValue(self.ros_node.fleet[self.agent_id].state.battery_level)

        # > Status
        if self.ros_node.fleet[self.agent_id].state.status == "active":
            # > Set radio button color
            self.ui.radioButton_availability_state.setStyleSheet("color: green;")
        else:
            # > Set radio button color
            self.ui.radioButton_availability_state.setStyleSheet("color: red;")

        # > Comm. link
        if True:    # TODO: Fix once comm. link is implemented
            # > Set radio button color
            self.ui.radioButton_comms_link_state.setStyleSheet("color: green;")
        else:
            # > Set radio button color
            self.ui.radioButton_comms_link_state.setStyleSheet("color: red;")

        # -> Allocation state
        if "allocation_state" not in self.ros_node.fleet[self.agent_id].local.keys():
            # self.ros_node.get_logger().warning(f"Agent {self.ros_node.fleet[self.agent_id].id} has no allocation state ({self.ros_node.fleet[self.agent_id].local.keys()})")
            return

        # > tableWidget_task_list_x
        if "task_list_x" in self.ros_node.fleet[self.agent_id].local["allocation_state"]:
            self.__set_table_widget(
                table_widget=self.ui.tableWidget_task_list_x,
                data=self.ros_node.fleet[self.agent_id].local["allocation_state"]["task_list_x"]
            )

        # > tableWidget_local_bids_c
        if "local_bids_c" in self.ros_node.fleet[self.agent_id].local["allocation_state"]:
            self.__set_table_widget(
                table_widget=self.ui.tableWidget_local_bids_c,
                data=self.ros_node.fleet[self.agent_id].local["allocation_state"]["local_bids_c"]
            )

        # > tableWidget_local_allocations_d
        if "local_allocations_d" in self.ros_node.fleet[self.agent_id].local["allocation_state"]:
            self.__set_table_widget(
                table_widget=self.ui.tableWidget_local_allocations_d,
                data=self.ros_node.fleet[self.agent_id].local["allocation_state"]["local_allocations_d"]
            )

        # > tableWidget_winning_bids_y
        if "winning_bids_y" in self.ros_node.fleet[self.agent_id].local["allocation_state"]:
            self.__set_table_widget(
                table_widget=self.ui.tableWidget_winning_bids_y,
                data=self.ros_node.fleet[self.agent_id].local["allocation_state"]["winning_bids_y"]
            )

        # > tableWidget_shared_bids_f
        if "shared_bids_f" in self.ros_node.fleet[self.agent_id].local["allocation_state"]:
            self.__set_table_widget(
                table_widget=self.ui.tableWidget_shared_bids_f,
                data=self.ros_node.fleet[self.agent_id].local["allocation_state"]["shared_bids_f"]
            )

        # > tableWidget_shared_bids_priority_beta
        if "shared_bids_priority_beta" in self.ros_node.fleet[self.agent_id].local["allocation_state"]:
            self.__set_table_widget(
                table_widget=self.ui.tableWidget_shared_bids_priority_beta,
                data=self.ros_node.fleet[self.agent_id].local["allocation_state"]["shared_bids_priority_beta"]
            )

        # > tableWidget_shared_allocations_a
        if "shared_allocations_a" in self.ros_node.fleet[self.agent_id].local["allocation_state"]:
            self.__set_table_widget(
                table_widget=self.ui.tableWidget_shared_allocations_a,
                data=self.ros_node.fleet[self.agent_id].local["allocation_state"]["shared_allocations_a"]
            )

        # > tableWidget_shared_allocation_priority_alpha
        if "shared_allocations_priority_alpha" in self.ros_node.fleet[self.agent_id].local["allocation_state"]:
            self.__set_table_widget(
                table_widget=self.ui.tableWidget_shared_allocation_priority_alpha,
                data=self.ros_node.fleet[self.agent_id].local["allocation_state"]["shared_allocations_priority_alpha"]
            )

    def __set_table_widget(self, table_widget, data):
        # -> Remove column with self ID
        if self.ros_node.id in data.columns:
            data = data.drop(columns=self.ros_node.id)

        # -> Sort by column and index
        data = data.sort_index().sort_index(axis=1)

        table_widget.setRowCount(len(data.index))
        table_widget.setColumnCount(len(data.columns))

        # -> Set the column headers
        table_widget.setHorizontalHeaderLabels(data.columns)

        # -> Set the ids as the row headers
        table_widget.setVerticalHeaderLabels(data.index)

        for i, row in enumerate(data.values):
            for j, cell in enumerate(row):
                # -> Set the data
                table_widget.setItem(i, j, QtWidgets.QTableWidgetItem(str(round(cell, 3))))

                # -> Center the values in the cells
                table_widget.item(i, j).setTextAlignment(QtCore.Qt.AlignCenter)

        # -> Adjust column width
        table_widget.resizeColumnsToContents()
