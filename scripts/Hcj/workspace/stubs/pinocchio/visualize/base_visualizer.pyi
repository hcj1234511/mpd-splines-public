from __future__ import annotations
import imageio as imageio
import numpy as np
from pinocchio import pinocchio_pywrap as pin
from pinocchio.shortcuts import buildModelsFromUrdf
from pinocchio.shortcuts import createDatas
import posixpath as osp
import time as time
__all__: list = ['BaseVisualizer']
class BaseVisualizer:
    """
    Pinocchio visualizers are employed to easily display a model at a given configuration.
        BaseVisualizer is not meant to be directly employed, but only to provide a uniform interface and a few common methods.
        New visualizers should extend this class and override its methods as neeeded.
        
    """
    _video_writer = None
    def __init__(self, model = ..., collision_model = None, visual_model = None, copy_models = False, data = None, collision_data = None, visual_data = None):
        """
        Construct a display from the given model, collision model, and visual model.
                If copy_models is True, the models are copied. Otherwise, they are simply kept as a reference.
        """
    def captureImage(self, w = None, h = None):
        """
        Captures an image from the viewer and returns an RGB array.
        """
    def clean(self):
        """
        Delete all the objects from the whole scene
        """
    def create_video_ctx(self, filename = None, fps = 30, directory = None, **kwargs):
        """
        Create a video recording context, generating the output filename if necessary.
        
                Code inspired from https://github.com/petrikvladimir/RoboMeshCat.
                
        """
    def disableCameraControl(self):
        ...
    def display(self, q = None):
        """
        Display the robot at configuration q or refresh the rendering
                from the current placements contained in data by placing all the bodies in the viewer.
        """
    def displayCollisions(self, visibility):
        """
        Set whether to display collision objects or not.
        """
    def displayVisuals(self, visibility):
        """
        Set whether to display visual objects or not.
        """
    def drawFrameVelocities(self, *args, **kwargs):
        """
        Draw current frame velocities.
        """
    def enableCameraControl(self):
        ...
    def getViewerNodeName(self, geometry_object, geometry_type):
        """
        Return the name of the geometry object inside the viewer.
        """
    def has_video_writer(self):
        ...
    def initViewer(self, *args, **kwargs):
        """
        Init the viewer by loading the gui and creating a window.
        """
    def loadViewerModel(self, *args, **kwargs):
        """
        Create the scene displaying the robot meshes in the viewer
        """
    def play(self, q_trajectory, dt = None, callback = None, capture = False, **kwargs):
        """
        Play a trajectory with given time step. Optionally capture RGB images and returns them.
        """
    def rebuildData(self):
        """
        Re-build the data objects. Needed if the models were modified.
                Warning: this will delete any information stored in all data objects.
        """
    def reload(self, new_geometry_object, geometry_type = None):
        """
        Reload a geometry_object given by its type
        """
    def setBackgroundColor(self, *args, **kwargs):
        """
        Set the visualizer background color.
        """
    def setCameraPose(self, pose = ...):
        """
        Set camera 6D pose using a 4x4 matrix.
        """
    def setCameraPosition(self, position):
        """
        Set the camera's 3D position.
        """
    def setCameraTarget(self, target):
        """
        Set the camera target.
        """
    def setCameraZoom(self, zoom):
        """
        Set camera zoom value.
        """
    def sleep(self, dt):
        ...
class VideoContext:
    def __enter__(self):
        ...
    def __exit__(self, *args):
        ...
    def __init__(self, viz, fps, filename, **kwargs):
        ...
IMAGEIO_SUPPORT: bool = True
