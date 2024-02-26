
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os

# Libs
from PySide6 import QtCore, QtGui, QtUiTools

# Own modules

##################################################################################################################


class uic:
    """
    Placeholder for uic module to simplify transition from PyQT5 to pyside6
    """
    @staticmethod
    def loadUi(uifilename: str, parent=None):
        """
        """
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(uifilename)
        uifile.open(QtCore.QFile.ReadOnly)
        ui = loader.load(uifile, parent)
        uifile.close()
        return ui