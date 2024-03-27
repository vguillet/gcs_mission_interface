
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os
from copy import copy
import threading

# Libs
from PySide6 import QtCore, QtGui, QtUiTools

# Own modules
from .ui_loader import uic

from maaf_tools.Event_bus import EventBus

from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet

##################################################################################################################


class Singleton(type):
    _instances = {}
    _lock = threading.Lock()

    def __call__(cls, *args, **kwargs):
        with cls._lock:
            if cls not in cls._instances:
                cls._instances[cls] = super().__call__(*args, **kwargs)
        return cls._instances[cls]

    # __instances = {}
    #
    # def __call__(cls, *args, **kwargs):
    #     if cls not in cls.__instances:
    #         cls.__instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
    #     return cls.__instances[cls]


class UiSingleton(metaclass=Singleton):
    def __init__(self):
        # -> Load GUI
        root_path = str(os.getcwd())

        self.ui = uic.loadUi(root_path + "/src/gcs_mission_interface/gcs_mission_interface/resources/uis/gsc_mission_interface_main.ui")


# ----- Singleton version of Fleet
class FleetSingletonMeta(Fleet.__class__, Singleton):
    pass


class FleetSingleton(Fleet, metaclass=FleetSingletonMeta):
    pass


# ----- Singleton version of TaskLog
class TaskLogSingletonMeta(TaskLog.__class__, Singleton):
    pass


class TaskLogSingleton(TaskLog, metaclass=TaskLogSingletonMeta):
    pass

# ----- Singleton version of Dict
class NucleusSingleton(metaclass=Singleton):
    def __init__(self):
        # -> Base attributes
        # self.event_bus = EventBus()

        self.id = None
        self.fleet = Fleet()
        self.task_log = TaskLog()

        # self.fleet = FleetSingleton()
        # self.task_log = TaskLogSingleton()

        self.env = {}
