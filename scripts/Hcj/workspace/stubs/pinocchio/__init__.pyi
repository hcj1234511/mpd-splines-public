import __future__
from __future__ import annotations
import hppfcl as hppfcl
from hppfcl.hppfcl import CachedMeshLoader
from hppfcl.hppfcl import CollisionGeometry
from hppfcl.hppfcl import CollisionResult
from hppfcl.hppfcl import Contact
from hppfcl.hppfcl import DistanceResult
from hppfcl.hppfcl import MeshLoader
from hppfcl.hppfcl import StdVec_CollisionResult
from hppfcl.hppfcl import StdVec_Contact
from hppfcl.hppfcl import StdVec_DistanceResult
import inspect as inspect
import numpy as numpy
from pinocchio.deprecated import GeometryObject
from pinocchio.deprecated import XYZQUATToSe3
from pinocchio.deprecated import buildGeomFromUrdf
from pinocchio.deprecated import computeCentroidalDynamics
from pinocchio.deprecated import forwardDynamics
from pinocchio.deprecated import frameJacobian
from pinocchio.deprecated import impulseDynamics
from pinocchio.deprecated import jointJacobian
from pinocchio.deprecated import kineticEnergy
from pinocchio.deprecated import potentialEnergy
from pinocchio.deprecated import se3ToXYZQUAT
from pinocchio.deprecated import se3ToXYZQUATtuple
from pinocchio.deprecated import setGeometryMeshScales
from pinocchio.deprecation import DeprecatedWarning
from pinocchio.deprecation import deprecated
from pinocchio.explog import exp
from pinocchio.explog import log
from pinocchio.pinocchio_pywrap import AngleAxis
from pinocchio.pinocchio_pywrap import ArgumentPosition
from pinocchio.pinocchio_pywrap import CollisionPair
from pinocchio.pinocchio_pywrap import Data
from pinocchio.pinocchio_pywrap import Exception
from pinocchio.pinocchio_pywrap import Force
from pinocchio.pinocchio_pywrap import Frame
from pinocchio.pinocchio_pywrap import FrameType
from pinocchio.pinocchio_pywrap import GeometryData
from pinocchio.pinocchio_pywrap import GeometryModel
from pinocchio.pinocchio_pywrap import GeometryNoMaterial
from pinocchio.pinocchio_pywrap import GeometryPhongMaterial
from pinocchio.pinocchio_pywrap import GeometryPool
from pinocchio.pinocchio_pywrap import GeometryType
from pinocchio.pinocchio_pywrap import Inertia
from pinocchio.pinocchio_pywrap import JointDataComposite
from pinocchio.pinocchio_pywrap import JointDataFreeFlyer
from pinocchio.pinocchio_pywrap import JointDataMimic_JointDataRX
from pinocchio.pinocchio_pywrap import JointDataMimic_JointDataRY
from pinocchio.pinocchio_pywrap import JointDataMimic_JointDataRZ
from pinocchio.pinocchio_pywrap import JointDataPX
from pinocchio.pinocchio_pywrap import JointDataPY
from pinocchio.pinocchio_pywrap import JointDataPZ
from pinocchio.pinocchio_pywrap import JointDataPlanar
from pinocchio.pinocchio_pywrap import JointDataPrismaticUnaligned
from pinocchio.pinocchio_pywrap import JointDataRUBX
from pinocchio.pinocchio_pywrap import JointDataRUBY
from pinocchio.pinocchio_pywrap import JointDataRUBZ
from pinocchio.pinocchio_pywrap import JointDataRX
from pinocchio.pinocchio_pywrap import JointDataRY
from pinocchio.pinocchio_pywrap import JointDataRZ
from pinocchio.pinocchio_pywrap import JointDataRevoluteUnaligned
from pinocchio.pinocchio_pywrap import JointDataRevoluteUnboundedUnalignedTpl
from pinocchio.pinocchio_pywrap import JointDataSpherical
from pinocchio.pinocchio_pywrap import JointDataSphericalZYX
from pinocchio.pinocchio_pywrap import JointDataTranslation
from pinocchio.pinocchio_pywrap import JointModel
from pinocchio.pinocchio_pywrap import JointModelComposite
from pinocchio.pinocchio_pywrap import JointModelFreeFlyer
from pinocchio.pinocchio_pywrap import JointModelMimic_JointModelRX
from pinocchio.pinocchio_pywrap import JointModelMimic_JointModelRY
from pinocchio.pinocchio_pywrap import JointModelMimic_JointModelRZ
from pinocchio.pinocchio_pywrap import JointModelPX
from pinocchio.pinocchio_pywrap import JointModelPY
from pinocchio.pinocchio_pywrap import JointModelPZ
from pinocchio.pinocchio_pywrap import JointModelPlanar
from pinocchio.pinocchio_pywrap import JointModelPrismaticUnaligned
from pinocchio.pinocchio_pywrap import JointModelRUBX
from pinocchio.pinocchio_pywrap import JointModelRUBY
from pinocchio.pinocchio_pywrap import JointModelRUBZ
from pinocchio.pinocchio_pywrap import JointModelRX
from pinocchio.pinocchio_pywrap import JointModelRY
from pinocchio.pinocchio_pywrap import JointModelRZ
from pinocchio.pinocchio_pywrap import JointModelRevoluteUnaligned
from pinocchio.pinocchio_pywrap import JointModelRevoluteUnboundedUnaligned
from pinocchio.pinocchio_pywrap import JointModelSpherical
from pinocchio.pinocchio_pywrap import JointModelSphericalZYX
from pinocchio.pinocchio_pywrap import JointModelTranslation
from pinocchio.pinocchio_pywrap import KinematicLevel
from pinocchio.pinocchio_pywrap import LieGroup
from pinocchio.pinocchio_pywrap import Model
from pinocchio.pinocchio_pywrap import ModelPool
from pinocchio.pinocchio_pywrap import Motion
from pinocchio.pinocchio_pywrap import Quaternion
from pinocchio.pinocchio_pywrap import ReferenceFrame
from pinocchio.pinocchio_pywrap import SE3
from pinocchio.pinocchio_pywrap import StdMap_String_VectorXd
from pinocchio.pinocchio_pywrap import StdVec_Bool
from pinocchio.pinocchio_pywrap import StdVec_CollisionPair
from pinocchio.pinocchio_pywrap import StdVec_Data
from pinocchio.pinocchio_pywrap import StdVec_Double
from pinocchio.pinocchio_pywrap import StdVec_Force
from pinocchio.pinocchio_pywrap import StdVec_Frame
from pinocchio.pinocchio_pywrap import StdVec_GeometryData
from pinocchio.pinocchio_pywrap import StdVec_GeometryModel
from pinocchio.pinocchio_pywrap import StdVec_GeometryObject
from pinocchio.pinocchio_pywrap import StdVec_Index
from pinocchio.pinocchio_pywrap import StdVec_IndexVector
from pinocchio.pinocchio_pywrap import StdVec_Inertia
from pinocchio.pinocchio_pywrap import StdVec_Int
from pinocchio.pinocchio_pywrap import StdVec_JointModelVector
from pinocchio.pinocchio_pywrap import StdVec_Matrix6x
from pinocchio.pinocchio_pywrap import StdVec_Motion
from pinocchio.pinocchio_pywrap import StdVec_SE3
from pinocchio.pinocchio_pywrap import StdVec_StdString
from pinocchio.pinocchio_pywrap import StdVec_Vector3
from pinocchio.pinocchio_pywrap import cholesky
from pinocchio.pinocchio_pywrap import liegroups
from pinocchio.pinocchio_pywrap import map_indexing_suite_StdMap_String_VectorXd_entry
from pinocchio.pinocchio_pywrap import rpy
from pinocchio.pinocchio_pywrap import serialization
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.shortcuts import buildModelsFromUrdf
from pinocchio.shortcuts import createDatas
from pinocchio.utils import npToTTuple
from pinocchio.utils import npToTuple
import sys as sys
from . import deprecation
from . import explog
from . import pinocchio_pywrap
from . import robot_wrapper
from . import shortcuts
from . import utils
from . import visualize
__all__: list[str] = ['ACCELERATION', 'ARG0', 'ARG1', 'ARG2', 'ARG3', 'ARG4', 'AngleAxis', 'ArgumentPosition', 'BODY', 'COLLISION', 'CachedMeshLoader', 'CollisionGeometry', 'CollisionPair', 'CollisionResult', 'Contact', 'Data', 'DeprecatedWarning', 'DistanceResult', 'Exception', 'FIXED_JOINT', 'Force', 'Frame', 'FrameType', 'GeometryData', 'GeometryModel', 'GeometryNoMaterial', 'GeometryObject', 'GeometryPhongMaterial', 'GeometryPool', 'GeometryType', 'Inertia', 'JOINT', 'JointDataComposite', 'JointDataFreeFlyer', 'JointDataMimic_JointDataRX', 'JointDataMimic_JointDataRY', 'JointDataMimic_JointDataRZ', 'JointDataPX', 'JointDataPY', 'JointDataPZ', 'JointDataPlanar', 'JointDataPrismaticUnaligned', 'JointDataRUBX', 'JointDataRUBY', 'JointDataRUBZ', 'JointDataRX', 'JointDataRY', 'JointDataRZ', 'JointDataRevoluteUnaligned', 'JointDataRevoluteUnboundedUnalignedTpl', 'JointDataSpherical', 'JointDataSphericalZYX', 'JointDataTranslation', 'JointModel', 'JointModelComposite', 'JointModelFreeFlyer', 'JointModelMimic_JointModelRX', 'JointModelMimic_JointModelRY', 'JointModelMimic_JointModelRZ', 'JointModelPX', 'JointModelPY', 'JointModelPZ', 'JointModelPlanar', 'JointModelPrismaticUnaligned', 'JointModelRUBX', 'JointModelRUBY', 'JointModelRUBZ', 'JointModelRX', 'JointModelRY', 'JointModelRZ', 'JointModelRevoluteUnaligned', 'JointModelRevoluteUnboundedUnaligned', 'JointModelSpherical', 'JointModelSphericalZYX', 'JointModelTranslation', 'KinematicLevel', 'LOCAL', 'LOCAL_WORLD_ALIGNED', 'LieGroup', 'MeshLoader', 'Model', 'ModelPool', 'Motion', 'OP_FRAME', 'PINOCCHIO_MAJOR_VERSION', 'PINOCCHIO_MINOR_VERSION', 'PINOCCHIO_PATCH_VERSION', 'POSITION', 'Quaternion', 'ReferenceFrame', 'RobotWrapper', 'SE3', 'SENSOR', 'StdMap_String_VectorXd', 'StdVec_Bool', 'StdVec_CollisionPair', 'StdVec_CollisionResult', 'StdVec_Contact', 'StdVec_Data', 'StdVec_DistanceResult', 'StdVec_Double', 'StdVec_Force', 'StdVec_Frame', 'StdVec_GeometryData', 'StdVec_GeometryModel', 'StdVec_GeometryObject', 'StdVec_Index', 'StdVec_IndexVector', 'StdVec_Inertia', 'StdVec_Int', 'StdVec_JointModelVector', 'StdVec_Matrix6x', 'StdVec_Motion', 'StdVec_SE3', 'StdVec_StdString', 'StdVec_Vector3', 'VELOCITY', 'VISUAL', 'WITH_CPPAD', 'WITH_HPP_FCL', 'WITH_HPP_FCL_BINDINGS', 'WITH_OPENMP', 'WITH_URDFDOM', 'WORLD', 'XYZQUATToSe3', 'buildGeomFromUrdf', 'buildModelsFromUrdf', 'cholesky', 'computeCentroidalDynamics', 'createDatas', 'deprecated', 'deprecation', 'exp', 'explog', 'forwardDynamics', 'frameJacobian', 'hppfcl', 'impulseDynamics', 'inspect', 'jointJacobian', 'kineticEnergy', 'liegroups', 'log', 'map_indexing_suite_StdMap_String_VectorXd_entry', 'module_info', 'npToTTuple', 'npToTuple', 'numpy', 'pin', 'pinocchio_pywrap', 'potentialEnergy', 'print_function', 'robot_wrapper', 'rpy', 'se3ToXYZQUAT', 'se3ToXYZQUATtuple', 'serialization', 'setGeometryMeshScales', 'shortcuts', 'submodules', 'sys', 'utils', 'visualize']
ACCELERATION: pinocchio_pywrap.KinematicLevel  # value = pinocchio.pinocchio_pywrap.KinematicLevel.ACCELERATION
ARG0: pinocchio_pywrap.ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG0
ARG1: pinocchio_pywrap.ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG1
ARG2: pinocchio_pywrap.ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG2
ARG3: pinocchio_pywrap.ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG3
ARG4: pinocchio_pywrap.ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG4
BODY: pinocchio_pywrap.FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.BODY
COLLISION: pinocchio_pywrap.GeometryType  # value = pinocchio.pinocchio_pywrap.GeometryType.COLLISION
FIXED_JOINT: pinocchio_pywrap.FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.FIXED_JOINT
JOINT: pinocchio_pywrap.FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.JOINT
LOCAL: pinocchio_pywrap.ReferenceFrame  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL
LOCAL_WORLD_ALIGNED: pinocchio_pywrap.ReferenceFrame  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED
OP_FRAME: pinocchio_pywrap.FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.OP_FRAME
PINOCCHIO_MAJOR_VERSION: int = 2
PINOCCHIO_MINOR_VERSION: int = 7
PINOCCHIO_PATCH_VERSION: int = 0
POSITION: pinocchio_pywrap.KinematicLevel  # value = pinocchio.pinocchio_pywrap.KinematicLevel.POSITION
SENSOR: pinocchio_pywrap.FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.SENSOR
VELOCITY: pinocchio_pywrap.KinematicLevel  # value = pinocchio.pinocchio_pywrap.KinematicLevel.VELOCITY
VISUAL: pinocchio_pywrap.GeometryType  # value = pinocchio.pinocchio_pywrap.GeometryType.VISUAL
WITH_CPPAD: bool = False
WITH_HPP_FCL: bool = True
WITH_HPP_FCL_BINDINGS: bool = True
WITH_OPENMP: bool = True
WITH_URDFDOM: bool = True
WORLD: pinocchio_pywrap.ReferenceFrame  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.WORLD
__raw_version__: str = '2.7.0'
__version__: str = '2.7.0'
module_info: tuple = ('serialization', pinocchio.pinocchio_pywrap.serialization)
print_function: __future__._Feature  # value = _Feature((2, 6, 0, 'alpha', 2), (3, 0, 0, 'alpha', 0), 1048576)
submodules: list = [('cholesky', pinocchio.pinocchio_pywrap.cholesky), ('liegroups', pinocchio.pinocchio_pywrap.liegroups), ('rpy', pinocchio.pinocchio_pywrap.rpy), ('serialization', pinocchio.pinocchio_pywrap.serialization)]
pin = pinocchio_pywrap
