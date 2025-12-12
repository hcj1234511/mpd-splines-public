from __future__ import annotations
import hppfcl as hppfcl
import numpy as np
import numpy
import os as os
from pinocchio import pinocchio_pywrap as pin
from pinocchio.utils import npToTuple
import pinocchio.visualize.base_visualizer
from pinocchio.visualize.base_visualizer import BaseVisualizer
import typing
import warnings as warnings
__all__: list = ['MeshcatVisualizer']
class MeshcatVisualizer(pinocchio.visualize.base_visualizer.BaseVisualizer):
    """
    A Pinocchio display using Meshcat
    """
    CAMERA_PRESETS: typing.ClassVar[dict]  # value = {'preset0': [array([0., 0., 0.]), [3.0, 0.0, 1.0]], 'preset1': [array([0., 0., 0.]), [1.0, 1.0, 1.0]], 'preset2': [[0.0, 0.0, 0.6], [0.8, 1.0, 1.2]], 'acrobot': [[0.0, 0.1, 0.0], [0.5, 0.0, 0.2]], 'cam_ur': [[0.4, 0.6, -0.2], [1.0, 0.4, 1.2]], 'cam_ur2': [[0.4, 0.3, 0.0], [0.5, 0.1, 1.4]], 'cam_ur3': [[0.4, 0.3, 0.0], [0.6, 1.3, 0.3]], 'cam_ur4': [[-1.0, 0.3, 0.0], [1.3, 0.1, 1.2]], 'cam_ur5': [[-1.0, 0.3, 0.0], [-0.05, 1.5, 1.2]], 'talos': [[0.0, 1.2, 0.0], [1.5, 0.3, 1.5]], 'talos2': [[0.0, 1.1, 0.0], [1.2, 0.6, 1.5]]}
    FORCE_SCALE: typing.ClassVar[float] = 0.06
    FRAME_VEL_COLOR: typing.ClassVar[int] = 65280
    def _check_meshcat_has_get_image(self):
        ...
    def _draw_vectors_from_frame(self, vecs, frame_ids, vec_names, colors):
        """
        Draw vectors extending from given frames.
        """
    def addGeometryObject(self, obj, color = None):
        """
        Add a visual GeometryObject to the viewer, with an optional color.
        """
    def captureImage(self, w = None, h = None):
        """
        Capture an image from the Meshcat viewer and return an RGB array.
        """
    def clean(self):
        ...
    def delete(self, geometry_object, geometry_type):
        ...
    def disableCameraControl(self):
        ...
    def display(self, q = None):
        """
        Display the robot at configuration q in the viewer by placing all the bodies.
        """
    def displayCollisions(self, visibility):
        """
        Set whether to display collision objects or not.
        """
    def displayFrames(self, visibility, frame_ids = None, axis_length = 0.2, axis_width = 2):
        """
        Set whether to display frames or not.
        """
    def displayVisuals(self, visibility):
        """
        Set whether to display visual objects or not.
        """
    def drawFrameVelocities(self, frame_id, v_scale = 0.2, color = 65280):
        ...
    def enableCameraControl(self):
        ...
    def getViewerNodeName(self, geometry_object, geometry_type):
        """
        Return the name of the geometry object inside the viewer.
        """
    def initViewer(self, viewer = None, open = False, loadModel = False):
        """
        Start a new MeshCat server and client.
                Note: the server can also be started separately using the "meshcat-server" command in a terminal:
                this enables the server to remain active after the current script ends.
                
        """
    def initializeFrames(self, frame_ids = None, axis_length = 0.2, axis_width = 2):
        """
        Initializes the frame objects for display.
        """
    def loadMesh(self, geometry_object):
        ...
    def loadViewerGeometryObject(self, geometry_object, geometry_type, color = None):
        """
        Load a single geometry object
        """
    def loadViewerModel(self, rootNodeName = 'pinocchio', color = None):
        """
        Load the robot in a MeshCat viewer.
                Parameters:
                    rootNodeName: name to give to the robot in the viewer
                    color: optional, color to give to the robot. This overwrites the color present in the urdf.
                           Format is a list of four RGBA floats (between 0 and 1)
                
        """
    def reload(self, new_geometry_object, geometry_type = None):
        """
        Reload a geometry_object given by its name and its type
        """
    def setBackgroundColor(self, preset_name = 'gray'):
        """
        Set the background.
        """
    def setCameraPose(self, pose = ...):
        ...
    def setCameraPosition(self, position):
        ...
    def setCameraPreset(self, preset_key):
        """
        Set the camera angle and position using a given preset.
        """
    def setCameraTarget(self, target):
        ...
    def setCameraZoom(self, zoom):
        ...
    def updateFrames(self):
        """
        
                Updates the frame visualizations with the latest transforms from model data.
                
        """
    def updatePlacements(self, geometry_type):
        ...
def createCapsule(length, radius, radial_resolution = 30, cap_resolution = 10):
    ...
def isMesh(geometry_object):
    """
    Check whether the geometry object contains a Mesh supported by MeshCat
    """
def loadMesh(mesh):
    ...
def loadPrimitive(geometry_object):
    ...
COLOR_PRESETS: dict  # value = {'gray': ([0.98, 0.98, 0.98], [0.8, 0.8, 0.8]), 'white': (array([1., 1., 1.]))}
DEFAULT_COLOR_PROFILES: dict  # value = {'gray': ([0.98, 0.98, 0.98], [0.8, 0.8, 0.8]), 'white': (array([1., 1., 1.]))}
FRAME_AXIS_COLORS: numpy.ndarray  # value = array([[1. , 1. , 0. , 0.6, 0. , 0. ],...
FRAME_AXIS_POSITIONS: numpy.ndarray  # value = array([[0., 1., 0., 0., 0., 0.],...
WITH_HPP_FCL_BINDINGS: bool = True
