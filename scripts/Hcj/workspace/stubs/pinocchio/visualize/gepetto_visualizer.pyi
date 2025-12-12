from __future__ import annotations
import hppfcl as hppfcl
from pinocchio import pinocchio_pywrap as pin
from pinocchio.shortcuts import buildModelsFromUrdf
from pinocchio.shortcuts import createDatas
from pinocchio.utils import npToTuple
import pinocchio.visualize.base_visualizer
from pinocchio.visualize.base_visualizer import BaseVisualizer
import warnings as warnings
__all__: list = ['GepettoVisualizer']
class GepettoVisualizer(pinocchio.visualize.base_visualizer.BaseVisualizer):
    """
    A Pinocchio display using Gepetto Viewer
    """
    def display(self, q = None):
        """
        Display the robot at configuration q in the viewer by placing all the bodies.
        """
    def displayCollisions(self, visibility):
        """
        Set whether to display collision objects or not
        """
    def displayVisuals(self, visibility):
        """
        Set whether to display visual objects or not
        """
    def getViewerNodeName(self, geometry_object, geometry_type):
        """
        Return the name of the geometry object inside the viewer
        """
    def initViewer(self, viewer = None, windowName = 'python-pinocchio', sceneName = 'world', loadModel = False):
        """
        Init GepettoViewer by loading the gui and creating a window.
        """
    def loadPrimitive(self, meshName, geometry_object):
        ...
    def loadViewerGeometryObject(self, geometry_object, geometry_type):
        """
        Load a single geometry object
        """
    def loadViewerModel(self, rootNodeName = 'pinocchio'):
        """
        Create the scene displaying the robot meshes in gepetto-viewer
        """
WITH_HPP_FCL_BINDINGS: bool = True
