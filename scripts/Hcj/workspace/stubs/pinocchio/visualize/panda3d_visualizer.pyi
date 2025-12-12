from __future__ import annotations
import hppfcl as hppfcl
from pinocchio import pinocchio_pywrap as pin
from pinocchio.utils import npToTuple
import pinocchio.visualize.base_visualizer
from pinocchio.visualize.base_visualizer import BaseVisualizer
import warnings as warnings
__all__: list = ['Panda3dVisualizer']
class Panda3dVisualizer(pinocchio.visualize.base_visualizer.BaseVisualizer):
    """
    
        A Pinocchio display using panda3d engine.
        
    """
    def display(self, q = None):
        """
        Display the robot at configuration q in the viewer by placing all the bodies.
        """
    def displayCollisions(self, visibility):
        """
        Set whether to display collision objects or not.
        """
    def displayVisuals(self, visibility):
        """
        Set whether to display visual objects or not.
        """
    def getViewerNodeName(self, geometry_object, geometry_type):
        """
        Return the name of the geometry object inside the viewer.
        """
    def initViewer(self, viewer = None, load_model = False):
        """
        Init the viewer by attaching to / creating a GUI viewer.
        """
    def loadViewerModel(self, group_name, color = None):
        """
        Create a group of nodes displaying the robot meshes in the viewer.
        """
WITH_HPP_FCL_BINDINGS: bool = True
