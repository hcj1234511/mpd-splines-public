from __future__ import annotations
import typing
from . import cholesky
from . import liegroups
from . import rpy
from . import serialization
__all__: list[str] = ['ACCELERATION', 'ARG0', 'ARG1', 'ARG2', 'ARG3', 'ARG4', 'AngleAxis', 'ArgumentPosition', 'BODY', 'COLLISION', 'CollisionPair', 'Data', 'Exception', 'FIXED_JOINT', 'Force', 'Frame', 'FrameType', 'GeometryData', 'GeometryModel', 'GeometryNoMaterial', 'GeometryObject', 'GeometryPhongMaterial', 'GeometryPool', 'GeometryType', 'Hlog3', 'Inertia', 'JOINT', 'Jexp3', 'Jexp6', 'Jlog3', 'Jlog6', 'JointDataComposite', 'JointDataFreeFlyer', 'JointDataMimic_JointDataRX', 'JointDataMimic_JointDataRY', 'JointDataMimic_JointDataRZ', 'JointDataPX', 'JointDataPY', 'JointDataPZ', 'JointDataPlanar', 'JointDataPrismaticUnaligned', 'JointDataRUBX', 'JointDataRUBY', 'JointDataRUBZ', 'JointDataRX', 'JointDataRY', 'JointDataRZ', 'JointDataRevoluteUnaligned', 'JointDataRevoluteUnboundedUnalignedTpl', 'JointDataSpherical', 'JointDataSphericalZYX', 'JointDataTranslation', 'JointModel', 'JointModelComposite', 'JointModelFreeFlyer', 'JointModelMimic_JointModelRX', 'JointModelMimic_JointModelRY', 'JointModelMimic_JointModelRZ', 'JointModelPX', 'JointModelPY', 'JointModelPZ', 'JointModelPlanar', 'JointModelPrismaticUnaligned', 'JointModelRUBX', 'JointModelRUBY', 'JointModelRUBZ', 'JointModelRX', 'JointModelRY', 'JointModelRZ', 'JointModelRevoluteUnaligned', 'JointModelRevoluteUnboundedUnaligned', 'JointModelSpherical', 'JointModelSphericalZYX', 'JointModelTranslation', 'KinematicLevel', 'LOCAL', 'LOCAL_WORLD_ALIGNED', 'LieGroup', 'Model', 'ModelPool', 'Motion', 'OP_FRAME', 'PINOCCHIO_MAJOR_VERSION', 'PINOCCHIO_MINOR_VERSION', 'PINOCCHIO_PATCH_VERSION', 'POSITION', 'Quaternion', 'ReferenceFrame', 'SE3', 'SE3ToXYZQUAT', 'SE3ToXYZQUATtuple', 'SENSOR', 'StdMap_String_VectorXd', 'StdVec_Bool', 'StdVec_CollisionPair', 'StdVec_Data', 'StdVec_Double', 'StdVec_Force', 'StdVec_Frame', 'StdVec_GeometryData', 'StdVec_GeometryModel', 'StdVec_GeometryObject', 'StdVec_Index', 'StdVec_IndexVector', 'StdVec_Inertia', 'StdVec_Int', 'StdVec_JointModelVector', 'StdVec_Matrix6x', 'StdVec_Motion', 'StdVec_SE3', 'StdVec_StdString', 'StdVec_Vector3', 'VELOCITY', 'VISUAL', 'WITH_CPPAD', 'WITH_HPP_FCL', 'WITH_OPENMP', 'WITH_URDFDOM', 'WORLD', 'XYZQUATToSE3', 'aba', 'appendModel', 'bodyRegressor', 'buildGeomFromUrdf', 'buildGeomFromUrdfString', 'buildModelFromUrdf', 'buildModelFromXML', 'buildReducedModel', 'buildSampleGeometryModelHumanoid', 'buildSampleGeometryModelManipulator', 'buildSampleModelHumanoid', 'buildSampleModelHumanoidRandom', 'buildSampleModelManipulator', 'ccrba', 'centerOfMass', 'checkVersionAtLeast', 'cholesky', 'computeABADerivatives', 'computeAllTerms', 'computeBodyRadius', 'computeCentroidalDynamicsDerivatives', 'computeCentroidalMap', 'computeCentroidalMapTimeVariation', 'computeCentroidalMomentum', 'computeCentroidalMomentumTimeVariation', 'computeCollision', 'computeCollisions', 'computeCoriolisMatrix', 'computeDistance', 'computeDistances', 'computeForwardKinematicsDerivatives', 'computeFrameJacobian', 'computeFrameKinematicRegressor', 'computeGeneralizedGravity', 'computeGeneralizedGravityDerivatives', 'computeJointJacobian', 'computeJointJacobians', 'computeJointJacobiansTimeVariation', 'computeJointKinematicRegressor', 'computeJointTorqueRegressor', 'computeKKTContactDynamicMatrixInverse', 'computeKineticEnergy', 'computeMinverse', 'computePotentialEnergy', 'computeRNEADerivatives', 'computeStaticRegressor', 'computeStaticTorque', 'computeStaticTorqueDerivatives', 'computeSubtreeMasses', 'computeSupportedForceByFrame', 'computeSupportedInertiaByFrame', 'computeTotalMass', 'crba', 'dDifference', 'dIntegrate', 'dIntegrateTransport', 'dccrba', 'difference', 'distance', 'exp3', 'exp6', 'forwardDynamics', 'forwardKinematics', 'frameBodyRegressor', 'frameJacobianTimeVariation', 'framesForwardKinematics', 'getAcceleration', 'getCenterOfMassVelocityDerivatives', 'getCentroidalDynamicsDerivatives', 'getClassicalAcceleration', 'getCoriolisMatrix', 'getFrameAcceleration', 'getFrameAccelerationDerivatives', 'getFrameClassicalAcceleration', 'getFrameJacobian', 'getFrameJacobianTimeVariation', 'getFrameVelocity', 'getFrameVelocityDerivatives', 'getJacobianSubtreeCenterOfMass', 'getJointAccelerationDerivatives', 'getJointJacobian', 'getJointJacobianTimeVariation', 'getJointVelocityDerivatives', 'getKKTContactDynamicMatrixInverse', 'getVelocity', 'impulseDynamics', 'integrate', 'interpolate', 'isNormalized', 'isSameConfiguration', 'jacobianCenterOfMass', 'jacobianSubtreeCenterOfMass', 'jacobianSubtreeCoMJacobian', 'jointBodyRegressor', 'liegroups', 'loadReferenceConfigurations', 'loadReferenceConfigurationsFromXML', 'loadRotorParameters', 'log3', 'log6', 'map_indexing_suite_StdMap_String_VectorXd_entry', 'neutral', 'nonLinearEffects', 'normalize', 'omp_get_max_threads', 'printVersion', 'randomConfiguration', 'removeCollisionPairs', 'removeCollisionPairsFromXML', 'rnea', 'rpy', 'seed', 'serialization', 'sharedMemory', 'skew', 'skewSquare', 'squaredDistance', 'unSkew', 'updateFramePlacement', 'updateFramePlacements', 'updateGeometryPlacements', 'updateGlobalPlacements']
class AngleAxis(Boost.Python.instance):
    """
    AngleAxis representation of a rotation.
    
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (AngleAxis)arg1, (AngleAxis)arg2) -> bool
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor
        
        __init__( (object)self, (float)angle, (numpy.ndarray)axis) -> None :
            Initialize from angle and axis.
        
        __init__( (object)self, (numpy.ndarray)R) -> None :
            Initialize from a rotation matrix
        
        __init__( (object)self, (Quaternion)quaternion) -> None :
            Initialize from a quaternion.
        
        __init__( (object)self, (AngleAxis)copy) -> None :
            Copy constructor.
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        """
        
        __mul__( (AngleAxis)arg1, (numpy.ndarray)arg2) -> object
        
        __mul__( (AngleAxis)arg1, (Quaternion)arg2) -> object
        
        __mul__( (AngleAxis)arg1, (AngleAxis)arg2) -> object
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (AngleAxis)arg1, (AngleAxis)arg2) -> bool
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (AngleAxis)arg1) -> str
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (AngleAxis)arg1) -> str
        """
    @staticmethod
    def fromRotationMatrix(*args, **kwargs):
        """
        
        fromRotationMatrix( (AngleAxis)self, (numpy.ndarray)rotation matrix) -> AngleAxis :
            Sets *this from a 3x3 rotation matrix
        """
    @staticmethod
    def inverse(*args, **kwargs):
        """
        
        inverse( (AngleAxis)self) -> AngleAxis :
            Return the inverse rotation.
        """
    @staticmethod
    def isApprox(*args, **kwargs):
        """
        
        isApprox( (AngleAxis)self, (AngleAxis)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision determined by prec.
        """
    @staticmethod
    def matrix(*args, **kwargs):
        """
        
        matrix( (AngleAxis)self) -> numpy.ndarray :
            Returns an equivalent rotation matrix.
        """
    @staticmethod
    def toRotationMatrix(*args, **kwargs):
        """
        
        toRotationMatrix( (AngleAxis)arg1) -> numpy.ndarray :
            Constructs and returns an equivalent rotation matrix.
        """
    @property
    def angle(*args, **kwargs):
        """
        The rotation angle.
        """
    @angle.setter
    def angle(*args, **kwargs):
        ...
    @property
    def axis(*args, **kwargs):
        """
        The rotation axis.
        """
    @axis.setter
    def axis(*args, **kwargs):
        ...
class ArgumentPosition(Boost.Python.enum):
    ARG0: typing.ClassVar[ArgumentPosition]  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG0
    ARG1: typing.ClassVar[ArgumentPosition]  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG1
    ARG2: typing.ClassVar[ArgumentPosition]  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG2
    ARG3: typing.ClassVar[ArgumentPosition]  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG3
    ARG4: typing.ClassVar[ArgumentPosition]  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG4
    __slots__: typing.ClassVar[tuple] = tuple()
    names: typing.ClassVar[dict]  # value = {'ARG0': pinocchio.pinocchio_pywrap.ArgumentPosition.ARG0, 'ARG1': pinocchio.pinocchio_pywrap.ArgumentPosition.ARG1, 'ARG2': pinocchio.pinocchio_pywrap.ArgumentPosition.ARG2, 'ARG3': pinocchio.pinocchio_pywrap.ArgumentPosition.ARG3, 'ARG4': pinocchio.pinocchio_pywrap.ArgumentPosition.ARG4}
    values: typing.ClassVar[dict]  # value = {0: pinocchio.pinocchio_pywrap.ArgumentPosition.ARG0, 1: pinocchio.pinocchio_pywrap.ArgumentPosition.ARG1, 2: pinocchio.pinocchio_pywrap.ArgumentPosition.ARG2, 3: pinocchio.pinocchio_pywrap.ArgumentPosition.ARG3, 4: pinocchio.pinocchio_pywrap.ArgumentPosition.ARG4}
class CollisionPair(Boost.Python.instance):
    """
    Pair of ordered index defining a pair of collisions
    """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (CollisionPair)self) -> CollisionPair :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (CollisionPair)self, (dict)memo) -> CollisionPair :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (CollisionPair)arg1, (CollisionPair)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Empty constructor.
        
        __init__( (object)self, (int)index1, (int)index2) -> None :
            Initializer of collision pair.
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (CollisionPair)arg1, (CollisionPair)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (CollisionPair)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (CollisionPair)arg1) -> object
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (CollisionPair)self) -> CollisionPair :
            Returns a copy of *this.
        """
    @property
    def first(*args, **kwargs):
        ...
    @first.setter
    def first(*args, **kwargs):
        ...
    @property
    def second(*args, **kwargs):
        ...
    @second.setter
    def second(*args, **kwargs):
        ...
class Data(Boost.Python.instance):
    """
    Articulated rigid body data related to a Model.
    It contains all the data that can be modified by the Pinocchio algorithms.
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (Data)self) -> Data :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (Data)self, (dict)memo) -> Data :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (Data)arg1, (Data)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (Data)arg1) -> tuple
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (Data)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor.
        
        __init__( (object)arg1, (Model)model) -> None :
            Constructs a data structure from a given model.
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (Data)arg1, (Data)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (Data)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (Data)self) -> Data :
            Returns a copy of *this.
        """
    @staticmethod
    def loadFromBinary(*args, **kwargs):
        """
        
        loadFromBinary( (Data)self, (str)filename) -> None :
            Loads *this from a binary file.
        
        loadFromBinary( (Data)self, (StreamBuffer)buffer) -> None :
            Loads *this from a binary buffer.
        
        loadFromBinary( (Data)self, (StaticBuffer)buffer) -> None :
            Loads *this from a static binary buffer.
        """
    @staticmethod
    def loadFromString(*args, **kwargs):
        """
        
        loadFromString( (Data)self, (str)string) -> None :
            Parses from the input string the content of the current object.
        """
    @staticmethod
    def loadFromText(*args, **kwargs):
        """
        
        loadFromText( (Data)arg1, (str)filename) -> None :
            Loads *this from a text file.
        """
    @staticmethod
    def loadFromXML(*args, **kwargs):
        """
        
        loadFromXML( (Data)self, (str)filename, (str)tag_name) -> None :
            Loads *this from a XML file.
        """
    @staticmethod
    def saveToBinary(*args, **kwargs):
        """
        
        saveToBinary( (Data)self, (str)filename) -> None :
            Saves *this inside a binary file.
        
        saveToBinary( (Data)self, (StreamBuffer)buffer) -> None :
            Saves *this inside a binary buffer.
        
        saveToBinary( (Data)self, (StaticBuffer)buffer) -> None :
            Saves *this inside a static binary buffer.
        """
    @staticmethod
    def saveToString(*args, **kwargs):
        """
        
        saveToString( (Data)self) -> str :
            Parses the current object to a string.
        """
    @staticmethod
    def saveToText(*args, **kwargs):
        """
        
        saveToText( (Data)arg1, (str)filename) -> None :
            Saves *this inside a text file.
        """
    @staticmethod
    def saveToXML(*args, **kwargs):
        """
        
        saveToXML( (Data)arg1, (str)filename, (str)tag_name) -> None :
            Saves *this inside a XML file.
        """
    @property
    def Ag(*args, **kwargs):
        """
        Centroidal matrix which maps from joint velocity to the centroidal momentum.
        """
    @Ag.setter
    def Ag(*args, **kwargs):
        ...
    @property
    def B(*args, **kwargs):
        """
        Combined variations of the inertia matrix consistent with Christoffel symbols.
        """
    @B.setter
    def B(*args, **kwargs):
        ...
    @property
    def C(*args, **kwargs):
        """
        The Coriolis C(q,v) matrix such that the Coriolis effects are given by c(q,v) = C(q,v)v
        """
    @C.setter
    def C(*args, **kwargs):
        ...
    @property
    def D(*args, **kwargs):
        """
        Diagonal of UDUT inertia decomposition
        """
    @D.setter
    def D(*args, **kwargs):
        ...
    @property
    def Fcrb(*args, **kwargs):
        """
        Spatial forces set, used in CRBA
        """
    @Fcrb.setter
    def Fcrb(*args, **kwargs):
        ...
    @property
    def Ig(*args, **kwargs):
        """
        Centroidal Composite Rigid Body Inertia.
        """
    @Ig.setter
    def Ig(*args, **kwargs):
        ...
    @property
    def Ivx(*args, **kwargs):
        """
        Right variation of the inertia matrix.
        """
    @Ivx.setter
    def Ivx(*args, **kwargs):
        ...
    @property
    def J(*args, **kwargs):
        """
        Jacobian of joint placement
        """
    @J.setter
    def J(*args, **kwargs):
        ...
    @property
    def Jcom(*args, **kwargs):
        """
        Jacobian of center of mass.
        """
    @Jcom.setter
    def Jcom(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        """
        The joint space inertia matrix
        """
    @M.setter
    def M(*args, **kwargs):
        ...
    @property
    def Minv(*args, **kwargs):
        """
        The inverse of the joint space inertia matrix
        """
    @Minv.setter
    def Minv(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        """
        Joint Inertia square root (upper triangle)
        """
    @U.setter
    def U(*args, **kwargs):
        ...
    @property
    def Ycrb(*args, **kwargs):
        """
        Inertia of the sub-tree composit rigid body
        """
    @Ycrb.setter
    def Ycrb(*args, **kwargs):
        ...
    @property
    def a(*args, **kwargs):
        """
        Joint spatial acceleration
        """
    @a.setter
    def a(*args, **kwargs):
        ...
    @property
    def a_gf(*args, **kwargs):
        """
        Joint spatial acceleration containing also the contribution of the gravity acceleration
        """
    @a_gf.setter
    def a_gf(*args, **kwargs):
        ...
    @property
    def acom(*args, **kwargs):
        """
        CoM acceleration of the subtree starting at joint index i.
        """
    @acom.setter
    def acom(*args, **kwargs):
        ...
    @property
    def com(*args, **kwargs):
        """
        CoM position of the subtree starting at joint index i.
        """
    @com.setter
    def com(*args, **kwargs):
        ...
    @property
    def dAg(*args, **kwargs):
        """
        Time derivative of the centroidal momentum matrix Ag.
        """
    @dAg.setter
    def dAg(*args, **kwargs):
        ...
    @property
    def dJ(*args, **kwargs):
        """
        Time variation of the Jacobian of joint placement (data.J).
        """
    @dJ.setter
    def dJ(*args, **kwargs):
        ...
    @property
    def ddq(*args, **kwargs):
        """
        Joint accelerations (output of ABA)
        """
    @ddq.setter
    def ddq(*args, **kwargs):
        ...
    @property
    def ddq_dq(*args, **kwargs):
        """
        Partial derivative of the joint acceleration vector with respect to the joint configuration.
        """
    @ddq_dq.setter
    def ddq_dq(*args, **kwargs):
        ...
    @property
    def ddq_dv(*args, **kwargs):
        """
        Partial derivative of the joint acceleration vector with respect to the joint velocity.
        """
    @ddq_dv.setter
    def ddq_dv(*args, **kwargs):
        ...
    @property
    def dhg(*args, **kwargs):
        """
        Centroidal momentum time derivative (expressed in the frame centered at the CoM and aligned with the world frame).
        """
    @dhg.setter
    def dhg(*args, **kwargs):
        ...
    @property
    def dq_after(*args, **kwargs):
        """
        Generalized velocity after the impact.
        """
    @dq_after.setter
    def dq_after(*args, **kwargs):
        ...
    @property
    def dtau_dq(*args, **kwargs):
        """
        Partial derivative of the joint torque vector with respect to the joint configuration.
        """
    @dtau_dq.setter
    def dtau_dq(*args, **kwargs):
        ...
    @property
    def dtau_dv(*args, **kwargs):
        """
        Partial derivative of the joint torque vector with respect to the joint velocity.
        """
    @dtau_dv.setter
    def dtau_dv(*args, **kwargs):
        ...
    @property
    def f(*args, **kwargs):
        """
        Joint spatial force expresssed in the joint frame.
        """
    @f.setter
    def f(*args, **kwargs):
        ...
    @property
    def g(*args, **kwargs):
        """
        Vector of generalized gravity (dim model.nv).
        """
    @g.setter
    def g(*args, **kwargs):
        ...
    @property
    def h(*args, **kwargs):
        """
        Vector of spatial momenta expressed in the local frame of the joint.
        """
    @h.setter
    def h(*args, **kwargs):
        ...
    @property
    def hg(*args, **kwargs):
        """
        Centroidal momentum (expressed in the frame centered at the CoM and aligned with the world frame).
        """
    @hg.setter
    def hg(*args, **kwargs):
        ...
    @property
    def iMf(*args, **kwargs):
        """
        Body placement wrt to algorithm end effector.
        """
    @iMf.setter
    def iMf(*args, **kwargs):
        ...
    @property
    def impulse_c(*args, **kwargs):
        """
        Lagrange Multipliers linked to contact impulses
        """
    @impulse_c.setter
    def impulse_c(*args, **kwargs):
        ...
    @property
    def jointTorqueRegressor(*args, **kwargs):
        """
        Joint torque regressor.
        """
    @jointTorqueRegressor.setter
    def jointTorqueRegressor(*args, **kwargs):
        ...
    @property
    def kinetic_energy(*args, **kwargs):
        """
        Kinetic energy in [J] computed by computeKineticEnergy
        """
    @kinetic_energy.setter
    def kinetic_energy(*args, **kwargs):
        ...
    @property
    def lambda_c(*args, **kwargs):
        """
        Lagrange Multipliers linked to contact forces
        """
    @lambda_c.setter
    def lambda_c(*args, **kwargs):
        ...
    @property
    def lastChild(*args, **kwargs):
        """
        Index of the last child (for CRBA)
        """
    @lastChild.setter
    def lastChild(*args, **kwargs):
        ...
    @property
    def liMi(*args, **kwargs):
        """
        Body relative placement (wrt parent)
        """
    @liMi.setter
    def liMi(*args, **kwargs):
        ...
    @property
    def mass(*args, **kwargs):
        """
        Mass of the subtree starting at joint index i.
        """
    @mass.setter
    def mass(*args, **kwargs):
        ...
    @property
    def nle(*args, **kwargs):
        """
        Non Linear Effects (output of nle algorithm)
        """
    @nle.setter
    def nle(*args, **kwargs):
        ...
    @property
    def nvSubtree(*args, **kwargs):
        """
        Dimension of the subtree motion space (for CRBA)
        """
    @nvSubtree.setter
    def nvSubtree(*args, **kwargs):
        ...
    @property
    def nvSubtree_fromRow(*args, **kwargs):
        """
        Subtree of the current row index (used in Cholesky)
        """
    @nvSubtree_fromRow.setter
    def nvSubtree_fromRow(*args, **kwargs):
        ...
    @property
    def oMf(*args, **kwargs):
        """
        frames absolute placement (wrt world)
        """
    @oMf.setter
    def oMf(*args, **kwargs):
        ...
    @property
    def oMi(*args, **kwargs):
        """
        Body absolute placement (wrt world)
        """
    @oMi.setter
    def oMi(*args, **kwargs):
        ...
    @property
    def oa(*args, **kwargs):
        """
        Joint spatial acceleration expressed at the origin of the world frame.
        """
    @oa.setter
    def oa(*args, **kwargs):
        ...
    @property
    def oa_gf(*args, **kwargs):
        """
        Joint spatial acceleration containing also the contribution of the gravity acceleration, but expressed at the origin of the world frame.
        """
    @oa_gf.setter
    def oa_gf(*args, **kwargs):
        ...
    @property
    def of(*args, **kwargs):
        """
        Joint spatial force expresssed at the origin of the world frame.
        """
    @of.setter
    def of(*args, **kwargs):
        ...
    @property
    def ov(*args, **kwargs):
        """
        Joint spatial velocity expressed at the origin of the world frame.
        """
    @ov.setter
    def ov(*args, **kwargs):
        ...
    @property
    def parents_fromRow(*args, **kwargs):
        """
        First previous non-zero row in M (used in Cholesky)
        """
    @parents_fromRow.setter
    def parents_fromRow(*args, **kwargs):
        ...
    @property
    def potential_energy(*args, **kwargs):
        """
        Potential energy in [J] computed by computePotentialEnergy
        """
    @potential_energy.setter
    def potential_energy(*args, **kwargs):
        ...
    @property
    def staticRegressor(*args, **kwargs):
        """
        Static regressor.
        """
    @staticRegressor.setter
    def staticRegressor(*args, **kwargs):
        ...
    @property
    def tau(*args, **kwargs):
        """
        Joint torques (output of RNEA)
        """
    @tau.setter
    def tau(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        """
        Joint spatial velocity expressed in the joint frame.
        """
    @v.setter
    def v(*args, **kwargs):
        ...
    @property
    def vcom(*args, **kwargs):
        """
        CoM velocity of the subtree starting at joint index i.
        """
    @vcom.setter
    def vcom(*args, **kwargs):
        ...
    @property
    def vxI(*args, **kwargs):
        """
        Left variation of the inertia matrix.
        """
    @vxI.setter
    def vxI(*args, **kwargs):
        ...
class Exception(Boost.Python.instance):
    __instance_size__: typing.ClassVar[int] = 72
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1, (str)arg2) -> None
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @property
    def message(*args, **kwargs):
        ...
class Force(Boost.Python.instance):
    """
    Force vectors, in se3* == F^6.
    
    Supported operations ...
    """
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def Random(*args, **kwargs):
        """
        
        Random() -> Force :
            Returns a random Force.
        """
    @staticmethod
    def Zero(*args, **kwargs):
        """
        
        Zero() -> Force :
            Returns a zero Force.
        """
    @staticmethod
    def __add__(*args, **kwargs):
        """
        
        __add__( (Force)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __array__(*args, **kwargs):
        """
        
        __array__( (Force)arg1) -> object
        """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (Force)self) -> Force :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (Force)self, (dict)memo) -> Force :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (Force)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (Force)arg1) -> tuple
        """
    @staticmethod
    def __iadd__(*args, **kwargs):
        """
        
        __iadd__( (object)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor
        
        __init__( (object)self, (numpy.ndarray)linear, (numpy.ndarray)angular) -> None :
            Initialize from linear and angular components of a Wrench vector (don't mix the order).
        
        __init__( (object)self, (numpy.ndarray)array) -> None :
            Init from a vector 6 [force,torque]
        
        __init__( (object)self, (Force)other) -> None :
            Copy constructor.
        """
    @staticmethod
    def __isub__(*args, **kwargs):
        """
        
        __isub__( (object)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        """
        
        __mul__( (Force)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (Force)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __neg__(*args, **kwargs):
        """
        
        __neg__( (Force)arg1) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (Force)arg1) -> object
        """
    @staticmethod
    def __rmul__(*args, **kwargs):
        """
        
        __rmul__( (Force)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (Force)arg1) -> object
        """
    @staticmethod
    def __sub__(*args, **kwargs):
        """
        
        __sub__( (Force)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def __truediv__(*args, **kwargs):
        """
        
        __truediv__( (Force)arg1, (float)arg2) -> object
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (Force)self) -> Force :
            Returns a copy of *this.
        """
    @staticmethod
    def isApprox(*args, **kwargs):
        """
        
        isApprox( (Force)self, (Force)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    @staticmethod
    def isZero(*args, **kwargs):
        """
        
        isZero( (Force)self [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to the zero Force, within the precision given by prec.
        """
    @staticmethod
    def se3Action(*args, **kwargs):
        """
        
        se3Action( (Force)self, (SE3)M) -> Force :
            Returns the result of the dual action of M on *this.
        """
    @staticmethod
    def se3ActionInverse(*args, **kwargs):
        """
        
        se3ActionInverse( (Force)self, (SE3)M) -> Force :
            Returns the result of the dual action of the inverse of M on *this.
        """
    @staticmethod
    def setRandom(*args, **kwargs):
        """
        
        setRandom( (Force)self) -> None :
            Set the linear and angular components of *this to random values.
        """
    @staticmethod
    def setZero(*args, **kwargs):
        """
        
        setZero( (Force)self) -> None :
            Set the linear and angular components of *this to zero.
        """
    @property
    def angular(*args, **kwargs):
        """
        Angular part of a *this, corresponding to the angular velocity in case of a Spatial velocity.
        """
    @angular.setter
    def angular(*args, **kwargs):
        ...
    @property
    def linear(*args, **kwargs):
        """
        Linear part of a *this, corresponding to the linear velocity in case of a Spatial velocity.
        """
    @linear.setter
    def linear(*args, **kwargs):
        ...
    @property
    def np(*args, **kwargs):
        ...
    @property
    def vector(*args, **kwargs):
        """
        Returns the components of *this as a 6d vector.
        """
    @vector.setter
    def vector(*args, **kwargs):
        ...
class Frame(Boost.Python.instance):
    """
    A Plucker coordinate frame related to a parent joint inside a kinematic tree.
    
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (Frame)self) -> Frame :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (Frame)self, (dict)memo) -> Frame :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (Frame)arg1, (Frame)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (Frame)arg1) -> tuple
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (Frame)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor
        
        __init__( (object)self, (Frame)other) -> None :
            Copy constructor
        
        __init__( (object)arg1, (str)name, (int)parent_joint, (int)parent_frame, (SE3)placement, (FrameType)type [, (Inertia)inertia]) -> None :
            Initialize from a given name, type, parent joint index, parent frame index and placement wrt parent joint and an spatial inertia object.
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (Frame)arg1, (Frame)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (Frame)arg1) -> object
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (Frame)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (Frame)arg1) -> object
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (Frame)self) -> Frame :
            Returns a copy of *this.
        """
    @property
    def inertia(*args, **kwargs):
        """
        Inertia information attached to the frame.
        """
    @inertia.setter
    def inertia(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        """
        name  of the frame
        """
    @name.setter
    def name(*args, **kwargs):
        ...
    @property
    def parent(*args, **kwargs):
        """
        id of the parent joint
        """
    @parent.setter
    def parent(*args, **kwargs):
        ...
    @property
    def placement(*args, **kwargs):
        """
        placement in the parent joint local frame
        """
    @placement.setter
    def placement(*args, **kwargs):
        ...
    @property
    def previousFrame(*args, **kwargs):
        """
        id of the previous frame
        """
    @previousFrame.setter
    def previousFrame(*args, **kwargs):
        ...
    @property
    def type(*args, **kwargs):
        """
        Type of the frame
        """
    @type.setter
    def type(*args, **kwargs):
        ...
class FrameType(Boost.Python.enum):
    BODY: typing.ClassVar[FrameType]  # value = pinocchio.pinocchio_pywrap.FrameType.BODY
    FIXED_JOINT: typing.ClassVar[FrameType]  # value = pinocchio.pinocchio_pywrap.FrameType.FIXED_JOINT
    JOINT: typing.ClassVar[FrameType]  # value = pinocchio.pinocchio_pywrap.FrameType.JOINT
    OP_FRAME: typing.ClassVar[FrameType]  # value = pinocchio.pinocchio_pywrap.FrameType.OP_FRAME
    SENSOR: typing.ClassVar[FrameType]  # value = pinocchio.pinocchio_pywrap.FrameType.SENSOR
    __slots__: typing.ClassVar[tuple] = tuple()
    names: typing.ClassVar[dict]  # value = {'OP_FRAME': pinocchio.pinocchio_pywrap.FrameType.OP_FRAME, 'JOINT': pinocchio.pinocchio_pywrap.FrameType.JOINT, 'FIXED_JOINT': pinocchio.pinocchio_pywrap.FrameType.FIXED_JOINT, 'BODY': pinocchio.pinocchio_pywrap.FrameType.BODY, 'SENSOR': pinocchio.pinocchio_pywrap.FrameType.SENSOR}
    values: typing.ClassVar[dict]  # value = {1: pinocchio.pinocchio_pywrap.FrameType.OP_FRAME, 2: pinocchio.pinocchio_pywrap.FrameType.JOINT, 4: pinocchio.pinocchio_pywrap.FrameType.FIXED_JOINT, 8: pinocchio.pinocchio_pywrap.FrameType.BODY, 16: pinocchio.pinocchio_pywrap.FrameType.SENSOR}
class GeometryData(Boost.Python.instance):
    """
    Geometry data linked to a Geometry Model and a Data struct.
    """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (GeometryData)self) -> GeometryData :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (GeometryData)self, (dict)memo) -> GeometryData :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (GeometryData)arg1, (GeometryData)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self, (GeometryModel)geometry_model) -> None :
            Default constructor from a given GeometryModel
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (GeometryData)arg1, (GeometryData)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (GeometryData)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (GeometryData)arg1) -> object
        """
    @staticmethod
    def activateCollisionPair(*args, **kwargs):
        """
        
        activateCollisionPair( (GeometryData)self, (int)pair_id) -> None :
            Activate the collsion pair pair_id in geomModel.collisionPairs if it exists.
            note: Only active pairs are check for collision and distance computations.
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (GeometryData)self) -> GeometryData :
            Returns a copy of *this.
        """
    @staticmethod
    def deactivateAllCollisionPairs(*args, **kwargs):
        """
        
        deactivateAllCollisionPairs( (GeometryData)self) -> None :
            Deactivate all collision pairs.
        """
    @staticmethod
    def deactivateCollisionPair(*args, **kwargs):
        """
        
        deactivateCollisionPair( (GeometryData)self, (int)pair_id) -> None :
            Deactivate the collsion pair pair_id in geomModel.collisionPairs if it exists.
        """
    @staticmethod
    def fillInnerOuterObjectMaps(*args, **kwargs):
        """
        
        fillInnerOuterObjectMaps( (GeometryData)self, (GeometryModel)geometry_model) -> None :
            Fill inner and outer objects maps
        """
    @staticmethod
    def loadFromBinary(*args, **kwargs):
        """
        
        loadFromBinary( (GeometryData)self, (str)filename) -> None :
            Loads *this from a binary file.
        
        loadFromBinary( (GeometryData)self, (StreamBuffer)buffer) -> None :
            Loads *this from a binary buffer.
        
        loadFromBinary( (GeometryData)self, (StaticBuffer)buffer) -> None :
            Loads *this from a static binary buffer.
        """
    @staticmethod
    def loadFromString(*args, **kwargs):
        """
        
        loadFromString( (GeometryData)self, (str)string) -> None :
            Parses from the input string the content of the current object.
        """
    @staticmethod
    def loadFromText(*args, **kwargs):
        """
        
        loadFromText( (GeometryData)arg1, (str)filename) -> None :
            Loads *this from a text file.
        """
    @staticmethod
    def loadFromXML(*args, **kwargs):
        """
        
        loadFromXML( (GeometryData)self, (str)filename, (str)tag_name) -> None :
            Loads *this from a XML file.
        """
    @staticmethod
    def saveToBinary(*args, **kwargs):
        """
        
        saveToBinary( (GeometryData)self, (str)filename) -> None :
            Saves *this inside a binary file.
        
        saveToBinary( (GeometryData)self, (StreamBuffer)buffer) -> None :
            Saves *this inside a binary buffer.
        
        saveToBinary( (GeometryData)self, (StaticBuffer)buffer) -> None :
            Saves *this inside a static binary buffer.
        """
    @staticmethod
    def saveToString(*args, **kwargs):
        """
        
        saveToString( (GeometryData)self) -> str :
            Parses the current object to a string.
        """
    @staticmethod
    def saveToText(*args, **kwargs):
        """
        
        saveToText( (GeometryData)arg1, (str)filename) -> None :
            Saves *this inside a text file.
        """
    @staticmethod
    def saveToXML(*args, **kwargs):
        """
        
        saveToXML( (GeometryData)arg1, (str)filename, (str)tag_name) -> None :
            Saves *this inside a XML file.
        """
    @staticmethod
    def setActiveCollisionPairs(*args, **kwargs):
        """
        
        setActiveCollisionPairs( (GeometryData)self, (GeometryModel)geometry_model, (numpy.ndarray)collision_map [, (bool)upper]) -> None :
            Set the collision pair association from a given input array.
            Each entry of the input matrix defines the activation of a given collision pair.
        """
    @staticmethod
    def setGeometryCollisionStatus(*args, **kwargs):
        """
        
        setGeometryCollisionStatus( (GeometryData)self, (GeometryModel)geom_model, (int)geom_id, (bool)enable_collision) -> None :
            Enable or disable collision for the given geometry given by its geometry id with all the other geometries registered in the list of collision pairs.
        """
    @staticmethod
    def setSecurityMargins(*args, **kwargs):
        """
        
        setSecurityMargins( (GeometryData)self, (GeometryModel)geometry_model, (numpy.ndarray)security_margin_map [, (bool)upper]) -> None :
            Set the security margin of all the collision request in a row, according to the values stored in the associative map.
        """
    @property
    def activeCollisionPairs(*args, **kwargs):
        """
        Vector of active CollisionPairs
        """
    @property
    def collisionRequests(*args, **kwargs):
        """
        Defines which information should be computed by FCL for collision computations.
        
        Note: it is possible to define a security_margin and a break_distance for a collision request.
        Most likely, for robotics application, these thresholds will be different for each collision pairs
        (e.g. the two hands can have a large security margin while the two hips cannot.)
        """
    @property
    def collisionResults(*args, **kwargs):
        """
        Vector of collision results.
        """
    @property
    def distanceRequests(*args, **kwargs):
        """
        Defines which information should be computed by FCL for distance computations
        """
    @property
    def distanceResults(*args, **kwargs):
        """
        Vector of distance results.
        """
    @property
    def oMg(*args, **kwargs):
        """
        Vector of collision objects placement relative to the world frame.
        note: These quantities have to be updated by calling updateGeometryPlacements.
        """
    @property
    def radius(*args, **kwargs):
        """
        Vector of radius of bodies, i.e. the distance between the further point of the geometry object from the joint center.
        note: This radius information might be usuful in continuous collision checking
        """
class GeometryModel(Boost.Python.instance):
    """
    Geometry model containing the collision or visual geometries associated to a model.
    """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (GeometryModel)self) -> GeometryModel :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (GeometryModel)self, (dict)memo) -> GeometryModel :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (GeometryModel)arg1, (GeometryModel)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (GeometryModel)arg1, (GeometryModel)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (GeometryModel)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (GeometryModel)arg1) -> object
        """
    @staticmethod
    def addAllCollisionPairs(*args, **kwargs):
        """
        
        addAllCollisionPairs( (GeometryModel)arg1) -> None :
            Add all collision pairs.
            note : collision pairs between geometries having the same parent joint are not added.
        """
    @staticmethod
    def addCollisionPair(*args, **kwargs):
        """
        
        addCollisionPair( (GeometryModel)self, (CollisionPair)collision_pair) -> None :
            Add a collision pair given by the index of the two collision objects.
        """
    @staticmethod
    def addGeometryObject(*args, **kwargs):
        """
        
        addGeometryObject( (GeometryModel)self, (GeometryObject)geometry_object) -> int :
            Add a GeometryObject to a GeometryModel.
            Parameters
            	geometry_object : a GeometryObject
            
        
        addGeometryObject( (GeometryModel)self, (GeometryObject)geometry_object, (Model)model) -> int :
            Add a GeometryObject to a GeometryModel and set its parent joint by reading its value in the model.
            Parameters
            	geometry_object : a GeometryObject
            	model : a Model of the system
            
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (GeometryModel)self) -> GeometryModel :
            Returns a copy of *this.
        """
    @staticmethod
    def createData(*args, **kwargs):
        """
        
        createData( (GeometryModel)self) -> GeometryData :
            Create a GeometryData associated to the current model.
        """
    @staticmethod
    def existCollisionPair(*args, **kwargs):
        """
        
        existCollisionPair( (GeometryModel)self, (CollisionPair)collision_pair) -> bool :
            Check if a collision pair exists.
        """
    @staticmethod
    def existGeometryName(*args, **kwargs):
        """
        
        existGeometryName( (GeometryModel)self, (str)name) -> bool :
            Checks if a GeometryObject  given by its name exists.
        """
    @staticmethod
    def findCollisionPair(*args, **kwargs):
        """
        
        findCollisionPair( (GeometryModel)self, (CollisionPair)collision_pair) -> int :
            Return the index of a collision pair.
        """
    @staticmethod
    def getGeometryId(*args, **kwargs):
        """
        
        getGeometryId( (GeometryModel)self, (str)name) -> int :
            Returns the index of a GeometryObject given by its name.
        """
    @staticmethod
    def removeAllCollisionPairs(*args, **kwargs):
        """
        
        removeAllCollisionPairs( (GeometryModel)arg1) -> None :
            Remove all collision pairs.
        """
    @staticmethod
    def removeCollisionPair(*args, **kwargs):
        """
        
        removeCollisionPair( (GeometryModel)self, (CollisionPair)collision_pair) -> None :
            Remove a collision pair.
        """
    @staticmethod
    def removeGeometryObject(*args, **kwargs):
        """
        
        removeGeometryObject( (GeometryModel)self, (str)name) -> None :
            Remove a GeometryObject. Remove also the collision pairs that contain the object.
        """
    @staticmethod
    def setCollisionPairs(*args, **kwargs):
        """
        
        setCollisionPairs( (GeometryModel)self, (numpy.ndarray)collision_map [, (bool)upper]) -> None :
            Set the collision pairs from a given input array.
            Each entry of the input matrix defines the activation of a given collision pair(map[i,j] == True means that the pair (i,j) is active).
        """
    @property
    def collisionPairs(*args, **kwargs):
        """
        Vector of collision pairs.
        """
    @property
    def geometryObjects(*args, **kwargs):
        """
        Vector of geometries objects.
        """
    @property
    def ngeoms(*args, **kwargs):
        """
        Number of geometries contained in the Geometry Model.
        """
class GeometryNoMaterial(Boost.Python.instance):
    __instance_size__: typing.ClassVar[int] = 40
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        
        __init__( (object)arg1, (GeometryNoMaterial)arg2) -> None
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
class GeometryObject(Boost.Python.instance):
    """
    A wrapper on a collision geometry including its parent joint, parent frame, placement in parent joint's frame.
    
    """
    @staticmethod
    def CreateCapsule(*args, **kwargs):
        """
        
        CreateCapsule( (float)arg1, (float)arg2) -> GeometryObject
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (GeometryObject)arg1, (GeometryObject)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self, (str)name, (int)parent_frame, (int)parent_joint, (CollisionGeometry)collision_geometry, (SE3)placement [, (str)mesh_path [, (numpy.ndarray)mesh_scale [, (bool)override_material [, (numpy.ndarray)mesh_color [, (str)mesh_texture_path [, (object)mesh_material]]]]]]) -> None :
            Full constructor of a GeometryObject.
        
        __init__( (object)self, (str)name, (int)parent_joint, (CollisionGeometry)collision_geometry, (SE3)placement [, (str)mesh_path [, (numpy.ndarray)mesh_scale [, (bool)override_material [, (numpy.ndarray)mesh_color [, (str)mesh_texture_path [, (object)mesh_material]]]]]]) -> None :
            Reduced constructor of a GeometryObject. This constructor does not require to specify the parent frame index.
        
        __init__( (object)self, (GeometryObject)otherGeometryObject) -> None :
            Copy constructor
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (GeometryObject)arg1, (GeometryObject)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @property
    def disableCollision(*args, **kwargs):
        """
        If true, no collision or distance check will be done between the Geometry and any other geometry.
        """
    @disableCollision.setter
    def disableCollision(*args, **kwargs):
        ...
    @property
    def geometry(*args, **kwargs):
        """
        The FCL CollisionGeometry associated to the given GeometryObject.
        """
    @geometry.setter
    def geometry(*args, **kwargs):
        ...
    @property
    def meshColor(*args, **kwargs):
        """
        Color rgba of the mesh.
        """
    @meshColor.setter
    def meshColor(*args, **kwargs):
        ...
    @property
    def meshMaterial(*args, **kwargs):
        """
        Material associated to the mesh (applied only if overrideMaterial is True)
        """
    @meshMaterial.setter
    def meshMaterial(*args, **kwargs):
        ...
    @property
    def meshPath(*args, **kwargs):
        """
        Path to the mesh file.
        """
    @meshPath.setter
    def meshPath(*args, **kwargs):
        ...
    @property
    def meshScale(*args, **kwargs):
        """
        Scaling parameter of the mesh.
        """
    @meshScale.setter
    def meshScale(*args, **kwargs):
        ...
    @property
    def meshTexturePath(*args, **kwargs):
        """
        Path to the mesh texture file.
        """
    @meshTexturePath.setter
    def meshTexturePath(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        """
        Name associated to the given GeometryObject.
        """
    @name.setter
    def name(*args, **kwargs):
        ...
    @property
    def overrideMaterial(*args, **kwargs):
        """
        Boolean that tells whether material information is stored inside the given GeometryObject.
        """
    @overrideMaterial.setter
    def overrideMaterial(*args, **kwargs):
        ...
    @property
    def parentFrame(*args, **kwargs):
        """
        Index of the parent frame.
        """
    @parentFrame.setter
    def parentFrame(*args, **kwargs):
        ...
    @property
    def parentJoint(*args, **kwargs):
        """
        Index of the parent joint.
        """
    @parentJoint.setter
    def parentJoint(*args, **kwargs):
        ...
    @property
    def placement(*args, **kwargs):
        """
        Position of geometry object in parent joint's frame.
        """
    @placement.setter
    def placement(*args, **kwargs):
        ...
class GeometryPhongMaterial(Boost.Python.instance):
    __instance_size__: typing.ClassVar[int] = 112
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        
        __init__( (object)arg1, (GeometryPhongMaterial)arg2) -> None
        
        __init__( (object)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (float)arg4) -> None
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @property
    def meshEmissionColor(*args, **kwargs):
        """
        RGBA emission (ambient) color value of the mesh
        """
    @meshEmissionColor.setter
    def meshEmissionColor(*args, **kwargs):
        ...
    @property
    def meshShininess(*args, **kwargs):
        """
        Shininess associated to the specular lighting model (between 0 and 1)
        """
    @meshShininess.setter
    def meshShininess(*args, **kwargs):
        ...
    @property
    def meshSpecularColor(*args, **kwargs):
        """
        RGBA specular value of the mesh
        """
    @meshSpecularColor.setter
    def meshSpecularColor(*args, **kwargs):
        ...
class GeometryPool(ModelPool):
    """
    Pool containing a model + a geometry_model and several datas for parallel computations
    """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (GeometryPool)self) -> GeometryPool :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (GeometryPool)self, (dict)memo) -> GeometryPool :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self, (Model)model, (GeometryModel)geometry_model [, (int)size]) -> None :
            Default constructor.
        
        __init__( (object)self, (GeometryPool)other) -> None :
            Copy constructor.
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (GeometryPool)self) -> GeometryPool :
            Returns a copy of *this.
        """
    @staticmethod
    def geometry_data(*args, **kwargs):
        """
        
        geometry_data( (GeometryPool)self, (int)index) -> GeometryData :
            Return a specific geometry_data data.
        """
    @staticmethod
    def geometry_datas(*args, **kwargs):
        """
        
        geometry_datas( (GeometryPool)self) -> StdVec_GeometryData :
            Returns the geometry data vector.
        """
    @staticmethod
    def geometry_model(*args, **kwargs):
        """
        
        geometry_model( (GeometryPool)self) -> GeometryModel :
            Geometry model contained in the pool.
        """
    @staticmethod
    def update(*args, **kwargs):
        """
        
        update( (GeometryPool)self, (GeometryModel)geometry_model) -> None :
            Update the geometry model, meaning that all the datas will be refreshed accordingly.
        
        update( (GeometryPool)self, (GeometryData)geometry_data) -> None :
            Update all the geometry datas with the input geometry data value.
        
        update( (GeometryPool)self, (GeometryModel)geometry_model, (GeometryData)geometry_data) -> None :
            Update the geometry model and data together.
        """
class GeometryType(Boost.Python.enum):
    COLLISION: typing.ClassVar[GeometryType]  # value = pinocchio.pinocchio_pywrap.GeometryType.COLLISION
    VISUAL: typing.ClassVar[GeometryType]  # value = pinocchio.pinocchio_pywrap.GeometryType.VISUAL
    __slots__: typing.ClassVar[tuple] = tuple()
    names: typing.ClassVar[dict]  # value = {'VISUAL': pinocchio.pinocchio_pywrap.GeometryType.VISUAL, 'COLLISION': pinocchio.pinocchio_pywrap.GeometryType.COLLISION}
    values: typing.ClassVar[dict]  # value = {0: pinocchio.pinocchio_pywrap.GeometryType.VISUAL, 1: pinocchio.pinocchio_pywrap.GeometryType.COLLISION}
class Inertia(Boost.Python.instance):
    """
    This class represenses a sparse version of a Spatial Inertia and its is defined by its mass, its center of mass location and the rotational inertia expressed around this center of mass.
    
    Supported operations ...
    """
    __instance_size__: typing.ClassVar[int] = 112
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def FromBox(*args, **kwargs):
        """
        
        FromBox( (float)mass, (float)length_x, (float)length_y, (float)length_z) -> Inertia :
            Returns the Inertia of a box shape with a mass and of dimension the semi axis of length_{x,y,z}.
        """
    @staticmethod
    def FromCylinder(*args, **kwargs):
        """
        
        FromCylinder( (float)mass, (float)radius, (float)length) -> Inertia :
            Returns the Inertia of a cylinder defined by its mass, radius and length along the Z axis.
        """
    @staticmethod
    def FromDynamicParameters(*args, **kwargs):
        """
        
        FromDynamicParameters( (numpy.ndarray)dynamic_parameters) -> Inertia :
            Builds and inertia matrix from a vector of dynamic parameters.
            The parameters are given as dynamic_parameters = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T where I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter.
        """
    @staticmethod
    def FromEllipsoid(*args, **kwargs):
        """
        
        FromEllipsoid( (float)mass, (float)length_x, (float)length_y, (float)length_z) -> Inertia :
            Returns the Inertia of an ellipsoid shape defined by a mass and given dimensions the semi-axis of values length_{x,y,z}.
        """
    @staticmethod
    def FromSphere(*args, **kwargs):
        """
        
        FromSphere( (float)mass, (float)radius) -> Inertia :
            Returns the Inertia of a sphere defined by a given mass and radius.
        """
    @staticmethod
    def Identity(*args, **kwargs):
        """
        
        Identity() -> Inertia :
            Returns the identity Inertia.
        """
    @staticmethod
    def Random(*args, **kwargs):
        """
        
        Random() -> Inertia :
            Returns a random Inertia.
        """
    @staticmethod
    def Zero(*args, **kwargs):
        """
        
        Zero() -> Inertia :
            Returns the null Inertia.
        """
    @staticmethod
    def __add__(*args, **kwargs):
        """
        
        __add__( (Inertia)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    def __array__(*args, **kwargs):
        """
        
        __array__( (Inertia)arg1) -> numpy.ndarray
        """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (Inertia)self) -> Inertia :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (Inertia)self, (dict)memo) -> Inertia :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (Inertia)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (Inertia)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor.
        
        __init__( (object)arg1, (float)mass, (numpy.ndarray)lever, (numpy.ndarray)inertia) -> object :
            Initialize from mass, lever and 3d inertia.
        
        __init__( (object)self, (Inertia)other) -> None :
            Copy constructor.
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        """
        
        __mul__( (Inertia)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (Inertia)arg1, (Inertia)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (Inertia)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (Inertia)arg1) -> object
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (Inertia)self) -> Inertia :
            Returns a copy of *this.
        """
    @staticmethod
    def isApprox(*args, **kwargs):
        """
        
        isApprox( (Inertia)self, (Inertia)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    @staticmethod
    def isZero(*args, **kwargs):
        """
        
        isZero( (Inertia)self [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to the zero Inertia, within the precision given by prec.
        """
    @staticmethod
    def ivx(*args, **kwargs):
        """
        
        ivx( (Inertia)self, (Motion)v) -> numpy.ndarray :
            Returns the result of I vx, a 6x6 matrix.
        """
    @staticmethod
    def matrix(*args, **kwargs):
        """
        
        matrix( (Inertia)self) -> numpy.ndarray
        """
    @staticmethod
    def se3Action(*args, **kwargs):
        """
        
        se3Action( (Inertia)self, (SE3)M) -> Inertia :
            Returns the result of the action of M on *this.
        """
    @staticmethod
    def se3ActionInverse(*args, **kwargs):
        """
        
        se3ActionInverse( (Inertia)self, (SE3)M) -> Inertia :
            Returns the result of the action of the inverse of M on *this.
        """
    @staticmethod
    def setIdentity(*args, **kwargs):
        """
        
        setIdentity( (Inertia)self) -> None :
            Set *this to be the Identity inertia.
        """
    @staticmethod
    def setRandom(*args, **kwargs):
        """
        
        setRandom( (Inertia)self) -> None :
            Set all the components of *this to random values.
        """
    @staticmethod
    def setZero(*args, **kwargs):
        """
        
        setZero( (Inertia)self) -> None :
            Set all the components of *this to zero.
        """
    @staticmethod
    def toDynamicParameters(*args, **kwargs):
        """
        
        toDynamicParameters( (Inertia)self) -> numpy.ndarray :
            Returns the representation of the matrix as a vector of dynamic parameters.
            The parameters are given as v = [m, mc_x, mc_y, mc_z, I_{xx}, I_{xy}, I_{yy}, I_{xz}, I_{yz}, I_{zz}]^T where I = I_C + mS^T(c)S(c) and I_C has its origin at the barycenter
        """
    @staticmethod
    def variation(*args, **kwargs):
        """
        
        variation( (Inertia)self, (Motion)v) -> numpy.ndarray :
            Returns the time derivative of the inertia.
        """
    @staticmethod
    def vtiv(*args, **kwargs):
        """
        
        vtiv( (Inertia)self, (Motion)v) -> float :
            Returns the result of v.T * Iv.
        """
    @staticmethod
    def vxi(*args, **kwargs):
        """
        
        vxi( (Inertia)self, (Motion)v) -> numpy.ndarray :
            Returns the result of v x* I, a 6x6 matrix.
        """
    @staticmethod
    def vxiv(*args, **kwargs):
        """
        
        vxiv( (Inertia)self, (Motion)v) -> Force :
            Returns the result of v x Iv.
        """
    @property
    def inertia(*args, **kwargs):
        """
        Rotational part of the Spatial Inertia, i.e. a symmetric matrix representing the rotational inertia around the center of mass.
        """
    @inertia.setter
    def inertia(*args, **kwargs):
        ...
    @property
    def lever(*args, **kwargs):
        """
        Center of mass location of the Spatial Inertia. It corresponds to the location of the center of mass regarding to the frame where the Spatial Inertia is expressed.
        """
    @lever.setter
    def lever(*args, **kwargs):
        ...
    @property
    def mass(*args, **kwargs):
        """
        Mass of the Spatial Inertia.
        """
    @mass.setter
    def mass(*args, **kwargs):
        ...
    @property
    def np(*args, **kwargs):
        ...
class JointDataComposite(Boost.Python.instance):
    """
    JointDataComposite
    """
    __instance_size__: typing.ClassVar[int] = 400
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataComposite)arg1, (JointDataComposite)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        
        __init__( (object)arg1, (object)joint_data_vectors, (int)nq, (int)nv) -> None :
            Init JointDataComposite from a given collection of joint data
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataComposite)arg1, (JointDataComposite)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataComposite)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataComposite)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataComposite)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def StU(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def iMlast(*args, **kwargs):
        ...
    @property
    def joints(*args, **kwargs):
        ...
    @property
    def pjMi(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataFreeFlyer(Boost.Python.instance):
    """
    JointDataFreeFlyer
    """
    __instance_size__: typing.ClassVar[int] = 1072
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataFreeFlyer)arg1, (JointDataFreeFlyer)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataFreeFlyer)arg1, (JointDataFreeFlyer)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataFreeFlyer)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataFreeFlyer)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataFreeFlyer)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataMimic_JointDataRX(Boost.Python.instance):
    """
    JointDataMimic_JointDataRX
    """
    __instance_size__: typing.ClassVar[int] = 240
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataMimic_JointDataRX)arg1, (JointDataMimic_JointDataRX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataMimic_JointDataRX)arg1, (JointDataMimic_JointDataRX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataMimic_JointDataRX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataMimic_JointDataRX)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataMimic_JointDataRX)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataMimic_JointDataRY(Boost.Python.instance):
    """
    JointDataMimic_JointDataRY
    """
    __instance_size__: typing.ClassVar[int] = 240
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataMimic_JointDataRY)arg1, (JointDataMimic_JointDataRY)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataMimic_JointDataRY)arg1, (JointDataMimic_JointDataRY)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataMimic_JointDataRY)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataMimic_JointDataRY)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataMimic_JointDataRY)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataMimic_JointDataRZ(Boost.Python.instance):
    """
    JointDataMimic_JointDataRZ
    """
    __instance_size__: typing.ClassVar[int] = 240
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataMimic_JointDataRZ)arg1, (JointDataMimic_JointDataRZ)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataMimic_JointDataRZ)arg1, (JointDataMimic_JointDataRZ)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataMimic_JointDataRZ)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataMimic_JointDataRZ)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataMimic_JointDataRZ)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataPX(Boost.Python.instance):
    """
    JointDataPX
    """
    __instance_size__: typing.ClassVar[int] = 176
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataPX)arg1, (JointDataPX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataPX)arg1, (JointDataPX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataPX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataPX)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataPX)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataPY(Boost.Python.instance):
    """
    JointDataPY
    """
    __instance_size__: typing.ClassVar[int] = 176
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataPY)arg1, (JointDataPY)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataPY)arg1, (JointDataPY)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataPY)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataPY)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataPY)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataPZ(Boost.Python.instance):
    """
    JointDataPZ
    """
    __instance_size__: typing.ClassVar[int] = 176
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataPZ)arg1, (JointDataPZ)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataPZ)arg1, (JointDataPZ)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataPZ)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataPZ)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataPZ)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataPlanar(Boost.Python.instance):
    """
    JointDataPlanar
    """
    __instance_size__: typing.ClassVar[int] = 624
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataPlanar)arg1, (JointDataPlanar)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataPlanar)arg1, (JointDataPlanar)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataPlanar)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataPlanar)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataPlanar)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def StU(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataPrismaticUnaligned(Boost.Python.instance):
    """
    JointDataPrismaticUnaligned
    """
    __instance_size__: typing.ClassVar[int] = 240
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataPrismaticUnaligned)arg1, (JointDataPrismaticUnaligned)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        
        __init__( (object)arg1, (numpy.ndarray)axis) -> None :
            Init JointDataPrismaticUnaligned from an axis with x-y-z components
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataPrismaticUnaligned)arg1, (JointDataPrismaticUnaligned)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataPrismaticUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataPrismaticUnaligned)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataPrismaticUnaligned)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataRUBX(Boost.Python.instance):
    """
    JointDataRUBX
    """
    __instance_size__: typing.ClassVar[int] = 192
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataRUBX)arg1, (JointDataRUBX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataRUBX)arg1, (JointDataRUBX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataRUBX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataRUBX)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataRUBX)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataRUBY(Boost.Python.instance):
    """
    JointDataRUBY
    """
    __instance_size__: typing.ClassVar[int] = 192
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataRUBY)arg1, (JointDataRUBY)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataRUBY)arg1, (JointDataRUBY)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataRUBY)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataRUBY)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataRUBY)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataRUBZ(Boost.Python.instance):
    """
    JointDataRUBZ
    """
    __instance_size__: typing.ClassVar[int] = 192
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataRUBZ)arg1, (JointDataRUBZ)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataRUBZ)arg1, (JointDataRUBZ)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataRUBZ)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataRUBZ)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataRUBZ)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataRX(Boost.Python.instance):
    """
    JointDataRX
    """
    __instance_size__: typing.ClassVar[int] = 192
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataRX)arg1, (JointDataRX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataRX)arg1, (JointDataRX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataRX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataRX)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataRX)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataRY(Boost.Python.instance):
    """
    JointDataRY
    """
    __instance_size__: typing.ClassVar[int] = 192
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataRY)arg1, (JointDataRY)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataRY)arg1, (JointDataRY)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataRY)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataRY)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataRY)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataRZ(Boost.Python.instance):
    """
    JointDataRZ
    """
    __instance_size__: typing.ClassVar[int] = 192
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataRZ)arg1, (JointDataRZ)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataRZ)arg1, (JointDataRZ)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataRZ)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataRZ)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataRZ)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataRevoluteUnaligned(Boost.Python.instance):
    """
    JointDataRevoluteUnaligned
    """
    __instance_size__: typing.ClassVar[int] = 304
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataRevoluteUnaligned)arg1, (JointDataRevoluteUnaligned)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        
        __init__( (object)arg1, (numpy.ndarray)axis) -> None :
            Init JointDataRevoluteUnaligned from an axis with x-y-z components
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataRevoluteUnaligned)arg1, (JointDataRevoluteUnaligned)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataRevoluteUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataRevoluteUnaligned)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataRevoluteUnaligned)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataRevoluteUnboundedUnalignedTpl(Boost.Python.instance):
    """
    JointDataRevoluteUnboundedUnalignedTpl
    """
    __instance_size__: typing.ClassVar[int] = 304
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataRevoluteUnboundedUnalignedTpl)arg1, (JointDataRevoluteUnboundedUnalignedTpl)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataRevoluteUnboundedUnalignedTpl)arg1, (JointDataRevoluteUnboundedUnalignedTpl)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataRevoluteUnboundedUnalignedTpl)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataRevoluteUnboundedUnalignedTpl)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataRevoluteUnboundedUnalignedTpl)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataSpherical(Boost.Python.instance):
    """
    JointDataSpherical
    """
    __instance_size__: typing.ClassVar[int] = 544
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataSpherical)arg1, (JointDataSpherical)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataSpherical)arg1, (JointDataSpherical)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataSpherical)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataSpherical)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataSpherical)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataSphericalZYX(Boost.Python.instance):
    """
    JointDataSphericalZYX
    """
    __instance_size__: typing.ClassVar[int] = 704
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataSphericalZYX)arg1, (JointDataSphericalZYX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataSphericalZYX)arg1, (JointDataSphericalZYX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataSphericalZYX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataSphericalZYX)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataSphericalZYX)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def StU(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointDataTranslation(Boost.Python.instance):
    """
    JointDataTranslation
    """
    __instance_size__: typing.ClassVar[int] = 464
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointDataTranslation)arg1, (JointDataTranslation)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointDataTranslation)arg1, (JointDataTranslation)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointDataTranslation)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointDataTranslation)arg1) -> object
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointDataTranslation)arg1) -> str
        """
    @property
    def Dinv(*args, **kwargs):
        ...
    @property
    def M(*args, **kwargs):
        ...
    @property
    def S(*args, **kwargs):
        ...
    @property
    def U(*args, **kwargs):
        ...
    @property
    def UDinv(*args, **kwargs):
        ...
    @property
    def c(*args, **kwargs):
        ...
    @property
    def v(*args, **kwargs):
        ...
class JointModel(Boost.Python.instance):
    """
    Generic Joint Model
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModel)arg1, (JointModel)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self [, (JointModel)other]) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModel)arg1, (JointModel)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModel)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModel)arg1) -> object
        """
    @staticmethod
    def hasConfigurationLimit(*args, **kwargs):
        """
        
        hasConfigurationLimit( (JointModel)arg1) -> StdVec_Bool :
            Return vector of boolean if joint has configuration limits.
        """
    @staticmethod
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        
        hasConfigurationLimitInTangent( (JointModel)arg1) -> StdVec_Bool :
            Return vector of boolean if joint has configuration limits in tangent space.
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModel)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModel)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModel)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelComposite(Boost.Python.instance):
    """
    JointModelComposite
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelComposite)arg1, (JointModelComposite)arg2) -> object
        
        __eq__( (JointModelComposite)arg1, (JointModelComposite)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        
        __init__( (object)self, (int)size) -> None :
            Init JointModelComposite with a defined size
        
        __init__( (object)arg1, (JointModel)joint_model) -> object :
            Init JointModelComposite from a joint
        
        __init__( (object)arg1, (JointModel)joint_model, (SE3)joint_placement) -> object :
            Init JointModelComposite from a joint and a placement
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelComposite)arg1, (JointModelComposite)arg2) -> object
        
        __ne__( (JointModelComposite)arg1, (JointModelComposite)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelComposite)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelComposite)arg1) -> object
        """
    @staticmethod
    def addJoint(*args, **kwargs):
        """
        
        addJoint( (JointModelComposite)self, (JointModel)joint_model [, (SE3)joint_placement]) -> JointModelComposite :
            Add a joint to the vector of joints.
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelComposite)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelComposite)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelComposite)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def jointPlacements(*args, **kwargs):
        ...
    @property
    def joints(*args, **kwargs):
        ...
    @property
    def njoints(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelFreeFlyer(Boost.Python.instance):
    """
    JointModelFreeFlyer
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelFreeFlyer)arg1, (JointModelFreeFlyer)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelFreeFlyer)arg1, (JointModelFreeFlyer)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelFreeFlyer)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelFreeFlyer)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelFreeFlyer)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelFreeFlyer)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelFreeFlyer)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelMimic_JointModelRX(Boost.Python.instance):
    """
    JointModelMimic_JointModelRX
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelMimic_JointModelRX)arg1, (JointModelMimic_JointModelRX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelMimic_JointModelRX)arg1, (JointModelMimic_JointModelRX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelMimic_JointModelRX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelMimic_JointModelRX)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelMimic_JointModelRX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelMimic_JointModelRX)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelMimic_JointModelRX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelMimic_JointModelRY(Boost.Python.instance):
    """
    JointModelMimic_JointModelRY
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelMimic_JointModelRY)arg1, (JointModelMimic_JointModelRY)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelMimic_JointModelRY)arg1, (JointModelMimic_JointModelRY)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelMimic_JointModelRY)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelMimic_JointModelRY)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelMimic_JointModelRY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelMimic_JointModelRY)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelMimic_JointModelRY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelMimic_JointModelRZ(Boost.Python.instance):
    """
    JointModelMimic_JointModelRZ
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelMimic_JointModelRZ)arg1, (JointModelMimic_JointModelRZ)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelMimic_JointModelRZ)arg1, (JointModelMimic_JointModelRZ)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelMimic_JointModelRZ)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelMimic_JointModelRZ)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelMimic_JointModelRZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelMimic_JointModelRZ)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelMimic_JointModelRZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelPX(Boost.Python.instance):
    """
    JointModelPX
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelPX)arg1, (JointModelPX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelPX)arg1, (JointModelPX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelPX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelPX)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelPX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelPX)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelPX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelPY(Boost.Python.instance):
    """
    JointModelPY
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelPY)arg1, (JointModelPY)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelPY)arg1, (JointModelPY)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelPY)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelPY)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelPY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelPY)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelPY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelPZ(Boost.Python.instance):
    """
    JointModelPZ
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelPZ)arg1, (JointModelPZ)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelPZ)arg1, (JointModelPZ)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelPZ)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelPZ)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelPZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelPZ)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelPZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelPlanar(Boost.Python.instance):
    """
    JointModelPlanar
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelPlanar)arg1, (JointModelPlanar)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelPlanar)arg1, (JointModelPlanar)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelPlanar)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelPlanar)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelPlanar)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelPlanar)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelPlanar)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelPrismaticUnaligned(Boost.Python.instance):
    """
    JointModelPrismaticUnaligned
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelPrismaticUnaligned)arg1, (JointModelPrismaticUnaligned)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        
        __init__( (object)self, (float)x, (float)y, (float)z) -> None :
            Init JointModelPrismaticUnaligned from the components x, y, z of the axis
        
        __init__( (object)self, (numpy.ndarray)axis) -> None :
            Init JointModelPrismaticUnaligned from an axis with x-y-z components
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelPrismaticUnaligned)arg1, (JointModelPrismaticUnaligned)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelPrismaticUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelPrismaticUnaligned)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelPrismaticUnaligned)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelPrismaticUnaligned)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelPrismaticUnaligned)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def axis(*args, **kwargs):
        """
        Translation axis of the JointModelPrismaticUnaligned.
        """
    @axis.setter
    def axis(*args, **kwargs):
        ...
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelRUBX(Boost.Python.instance):
    """
    JointModelRUBX
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelRUBX)arg1, (JointModelRUBX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelRUBX)arg1, (JointModelRUBX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelRUBX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelRUBX)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelRUBX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelRUBX)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelRUBX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelRUBY(Boost.Python.instance):
    """
    JointModelRUBY
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelRUBY)arg1, (JointModelRUBY)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelRUBY)arg1, (JointModelRUBY)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelRUBY)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelRUBY)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelRUBY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelRUBY)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelRUBY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelRUBZ(Boost.Python.instance):
    """
    JointModelRUBZ
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelRUBZ)arg1, (JointModelRUBZ)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelRUBZ)arg1, (JointModelRUBZ)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelRUBZ)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelRUBZ)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelRUBZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelRUBZ)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelRUBZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelRX(Boost.Python.instance):
    """
    JointModelRX
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelRX)arg1, (JointModelRX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelRX)arg1, (JointModelRX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelRX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelRX)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelRX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelRX)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelRX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelRY(Boost.Python.instance):
    """
    JointModelRY
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelRY)arg1, (JointModelRY)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelRY)arg1, (JointModelRY)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelRY)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelRY)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelRY)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelRY)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelRY)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelRZ(Boost.Python.instance):
    """
    JointModelRZ
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelRZ)arg1, (JointModelRZ)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelRZ)arg1, (JointModelRZ)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelRZ)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelRZ)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelRZ)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelRZ)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelRZ)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelRevoluteUnaligned(Boost.Python.instance):
    """
    JointModelRevoluteUnaligned
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelRevoluteUnaligned)arg1, (JointModelRevoluteUnaligned)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        
        __init__( (object)self, (float)x, (float)y, (float)z) -> None :
            Init JointModelRevoluteUnaligned from the components x, y, z of the axis
        
        __init__( (object)self, (numpy.ndarray)axis) -> None :
            Init JointModelRevoluteUnaligned from an axis with x-y-z components
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelRevoluteUnaligned)arg1, (JointModelRevoluteUnaligned)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelRevoluteUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelRevoluteUnaligned)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelRevoluteUnaligned)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelRevoluteUnaligned)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelRevoluteUnaligned)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def axis(*args, **kwargs):
        """
        Rotation axis of the JointModelRevoluteUnaligned.
        """
    @axis.setter
    def axis(*args, **kwargs):
        ...
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelRevoluteUnboundedUnaligned(Boost.Python.instance):
    """
    JointModelRevoluteUnboundedUnaligned
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelRevoluteUnboundedUnaligned)arg1, (JointModelRevoluteUnboundedUnaligned)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelRevoluteUnboundedUnaligned)arg1, (JointModelRevoluteUnboundedUnaligned)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelRevoluteUnboundedUnaligned)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelRevoluteUnboundedUnaligned)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelRevoluteUnboundedUnaligned)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelRevoluteUnboundedUnaligned)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelRevoluteUnboundedUnaligned)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelSpherical(Boost.Python.instance):
    """
    JointModelSpherical
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelSpherical)arg1, (JointModelSpherical)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelSpherical)arg1, (JointModelSpherical)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelSpherical)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelSpherical)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelSpherical)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelSpherical)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelSpherical)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelSphericalZYX(Boost.Python.instance):
    """
    JointModelSphericalZYX
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelSphericalZYX)arg1, (JointModelSphericalZYX)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelSphericalZYX)arg1, (JointModelSphericalZYX)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelSphericalZYX)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelSphericalZYX)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelSphericalZYX)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelSphericalZYX)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelSphericalZYX)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class JointModelTranslation(Boost.Python.instance):
    """
    JointModelTranslation
    """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (JointModelTranslation)arg1, (JointModelTranslation)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (JointModelTranslation)arg1, (JointModelTranslation)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (JointModelTranslation)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (JointModelTranslation)arg1) -> object
        """
    @staticmethod
    def hasSameIndexes(*args, **kwargs):
        """
        
        hasSameIndexes( (JointModelTranslation)self, (object)other) -> bool :
            Check if this has same indexes than other.
        """
    @staticmethod
    def setIndexes(*args, **kwargs):
        """
        
        setIndexes( (JointModelTranslation)self, (int)id, (int)idx_q, (int)idx_v) -> None
        """
    @staticmethod
    def shortname(*args, **kwargs):
        """
        
        shortname( (JointModelTranslation)self) -> str :
            Returns string indicating the joint type (class name):
            	- JointModelR[*]: Revolute Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnaligned: Revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelRUB[*]: Unbounded revolute Joint (without position limits), with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelRevoluteUnboundedUnaligned: Unbounded revolute Joint, with rotation axis not aligned with X, Y, nor Z
            	- JointModelP[*]: Prismatic Joint, with rotation axis [*] ∈ [X,Y,Z]
            	- JointModelPlanar: Planar joint
            	- JointModelPrismaticUnaligned: Prismatic joint, with translation axis not aligned with X, Y, nor Z
            	- JointModelSphericalZYX: Spherical joint (3D rotation)
            	- JointModelTranslation: Translation joint (3D translation)
            	- JointModelFreeFlyer: Joint enabling 3D rotation and translations.
        """
    @property
    def hasConfigurationLimit(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits.
        """
    @property
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        Return vector of boolean if joint has configuration limits in tangent space.
        """
    @property
    def id(*args, **kwargs):
        ...
    @property
    def idx_q(*args, **kwargs):
        ...
    @property
    def idx_v(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class KinematicLevel(Boost.Python.enum):
    ACCELERATION: typing.ClassVar[KinematicLevel]  # value = pinocchio.pinocchio_pywrap.KinematicLevel.ACCELERATION
    POSITION: typing.ClassVar[KinematicLevel]  # value = pinocchio.pinocchio_pywrap.KinematicLevel.POSITION
    VELOCITY: typing.ClassVar[KinematicLevel]  # value = pinocchio.pinocchio_pywrap.KinematicLevel.VELOCITY
    __slots__: typing.ClassVar[tuple] = tuple()
    names: typing.ClassVar[dict]  # value = {'POSITION': pinocchio.pinocchio_pywrap.KinematicLevel.POSITION, 'VELOCITY': pinocchio.pinocchio_pywrap.KinematicLevel.VELOCITY, 'ACCELERATION': pinocchio.pinocchio_pywrap.KinematicLevel.ACCELERATION}
    values: typing.ClassVar[dict]  # value = {0: pinocchio.pinocchio_pywrap.KinematicLevel.POSITION, 1: pinocchio.pinocchio_pywrap.KinematicLevel.VELOCITY, 2: pinocchio.pinocchio_pywrap.KinematicLevel.ACCELERATION}
class LieGroup(Boost.Python.instance):
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (LieGroup)arg1, (LieGroup)arg2) -> object
        """
    @staticmethod
    def __imul__(*args, **kwargs):
        """
        
        __imul__( (object)arg1, (LieGroup)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None :
            Default constructor
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        """
        
        __mul__( (LieGroup)arg1, (LieGroup)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def dDifference(*args, **kwargs):
        """
        
        dDifference( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (ArgumentPosition)arg4) -> numpy.ndarray
        
        dDifference( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (ArgumentPosition)arg4, (numpy.ndarray)arg5, (int)arg6) -> numpy.ndarray
        
        dDifference( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (ArgumentPosition)arg4, (int)arg5, (numpy.ndarray)arg6) -> numpy.ndarray
        """
    @staticmethod
    def dIntegrate(*args, **kwargs):
        """
        
        dIntegrate( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (ArgumentPosition)arg4) -> numpy.ndarray
        """
    @staticmethod
    def dIntegrateTransport(*args, **kwargs):
        """
        
        dIntegrateTransport( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (numpy.ndarray)arg4, (ArgumentPosition)arg5) -> numpy.ndarray
        """
    @staticmethod
    def dIntegrate_dq(*args, **kwargs):
        """
        
        dIntegrate_dq( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        
        dIntegrate_dq( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (numpy.ndarray)arg4, (int)arg5) -> numpy.ndarray
        
        dIntegrate_dq( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (int)arg4, (numpy.ndarray)arg5) -> numpy.ndarray
        """
    @staticmethod
    def dIntegrate_dv(*args, **kwargs):
        """
        
        dIntegrate_dv( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        
        dIntegrate_dv( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (numpy.ndarray)arg4, (int)arg5) -> numpy.ndarray
        
        dIntegrate_dv( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (int)arg4, (numpy.ndarray)arg5) -> numpy.ndarray
        """
    @staticmethod
    def difference(*args, **kwargs):
        """
        
        difference( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        """
    @staticmethod
    def distance(*args, **kwargs):
        """
        
        distance( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> float
        """
    @staticmethod
    def integrate(*args, **kwargs):
        """
        
        integrate( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        """
    @staticmethod
    def interpolate(*args, **kwargs):
        """
        
        interpolate( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3, (float)arg4) -> numpy.ndarray
        """
    @staticmethod
    def normalize(*args, **kwargs):
        """
        
        normalize( (LieGroup)arg1, (numpy.ndarray)arg2) -> None
        """
    @staticmethod
    def random(*args, **kwargs):
        """
        
        random( (LieGroup)arg1) -> numpy.ndarray
        """
    @staticmethod
    def randomConfiguration(*args, **kwargs):
        """
        
        randomConfiguration( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> numpy.ndarray
        """
    @staticmethod
    def squaredDistance(*args, **kwargs):
        """
        
        squaredDistance( (LieGroup)arg1, (numpy.ndarray)arg2, (numpy.ndarray)arg3) -> float
        """
    @property
    def name(*args, **kwargs):
        ...
    @property
    def neutral(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
class Model(Boost.Python.instance):
    """
    Articulated Rigid Body model
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (Model)self) -> Model :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (Model)self, (dict)memo) -> Model :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (Model)arg1, (Model)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (Model)arg1) -> tuple
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (Model)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor. Constructs an empty model.
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (Model)arg1, (Model)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (Model)arg1) -> object
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (Model)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (Model)arg1) -> object
        """
    @staticmethod
    def addBodyFrame(*args, **kwargs):
        """
        
        addBodyFrame( (Model)self, (str)body_name, (int)parentJoint, (SE3)body_placement, (int)previous_frame) -> int :
            add a body to the frame tree
        """
    @staticmethod
    def addFrame(*args, **kwargs):
        """
        
        addFrame( (Model)self, (Frame)frame [, (bool)append_inertia=True]) -> int :
            Add a frame to the vector of frames. If append_inertia set to True, the inertia value contained in frame will be added to the inertia supported by the parent joint.
        """
    @staticmethod
    def addJoint(*args, **kwargs):
        """
        
        addJoint( (Model)self, (int)parent_id, (JointModel)joint_model, (SE3)joint_placement, (str)joint_name) -> int :
            Adds a joint to the kinematic tree. The joint is defined by its placement relative to its parent joint and its name.
        
        addJoint( (Model)self, (int)parent_id, (JointModel)joint_model, (SE3)joint_placement, (str)joint_name, (numpy.ndarray)max_effort, (numpy.ndarray)max_velocity, (numpy.ndarray)min_config, (numpy.ndarray)max_config) -> int :
            Adds a joint to the kinematic tree with given bounds. The joint is defined by its placement relative to its parent joint and its name.This signature also takes as input effort, velocity limits as well as the bounds on the joint configuration.
        
        addJoint( (Model)self, (int)parent_id, (JointModel)joint_model, (SE3)joint_placement, (str)joint_name, (numpy.ndarray)max_effort, (numpy.ndarray)max_velocity, (numpy.ndarray)min_config, (numpy.ndarray)max_config, (numpy.ndarray)friction, (numpy.ndarray)damping) -> int :
            Adds a joint to the kinematic tree with given bounds. The joint is defined by its placement relative to its parent joint and its name.
            This signature also takes as input effort, velocity limits as well as the bounds on the joint configuration.
            The user should also provide the friction and damping related to the joint.
        """
    @staticmethod
    def addJointFrame(*args, **kwargs):
        """
        
        addJointFrame( (Model)self, (int)joint_id [, (int)frame_id]) -> int :
            Add the joint provided by its joint_id as a frame to the frame tree.
            The frame_id may be optionally provided.
        """
    @staticmethod
    def appendBodyToJoint(*args, **kwargs):
        """
        
        appendBodyToJoint( (Model)self, (int)joint_id, (Inertia)body_inertia, (SE3)body_placement) -> None :
            Appends a body to the joint given by its index. The body is defined by its inertia, its relative placement regarding to the joint and its name.
        """
    @staticmethod
    def check(*args, **kwargs):
        """
        
        check( (Model)self, (Data)data) -> bool :
            Check consistency of data wrt model.
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (Model)self) -> Model :
            Returns a copy of *this.
        """
    @staticmethod
    def createData(*args, **kwargs):
        """
        
        createData( (Model)self) -> Data :
            Create a Data object for the given model.
        """
    @staticmethod
    def existBodyName(*args, **kwargs):
        """
        
        existBodyName( (Model)self, (str)name) -> bool :
            Check if a frame of type BODY exists, given its name
        """
    @staticmethod
    def existFrame(*args, **kwargs):
        """
        
        existFrame( (Model)self, (str)name [, (FrameType)type]) -> bool :
            Returns true if the frame given by its name exists inside the Model with the given type.
        """
    @staticmethod
    def existJointName(*args, **kwargs):
        """
        
        existJointName( (Model)self, (str)name) -> bool :
            Check if a joint given by its name exists
        """
    @staticmethod
    def getBodyId(*args, **kwargs):
        """
        
        getBodyId( (Model)self, (str)name) -> int :
            Return the index of a frame of type BODY given by its name
        """
    @staticmethod
    def getFrameId(*args, **kwargs):
        """
        
        getFrameId( (Model)self, (str)name [, (FrameType)type]) -> int :
            Returns the index of the frame given by its name and its type. If the frame is not in the frames vector, it returns the current size of the frames vector.
        """
    @staticmethod
    def getJointId(*args, **kwargs):
        """
        
        getJointId( (Model)self, (str)name) -> int :
            Return the index of a joint given by its name
        """
    @staticmethod
    def hasConfigurationLimit(*args, **kwargs):
        """
        
        hasConfigurationLimit( (Model)self) -> StdVec_Bool :
            Returns list of boolean if joints have configuration limit.
        """
    @staticmethod
    def hasConfigurationLimitInTangent(*args, **kwargs):
        """
        
        hasConfigurationLimitInTangent( (Model)self) -> StdVec_Bool :
            Returns list of boolean if joints have configuration limit in tangent space  .
        """
    @staticmethod
    def loadFromBinary(*args, **kwargs):
        """
        
        loadFromBinary( (Model)self, (str)filename) -> None :
            Loads *this from a binary file.
        
        loadFromBinary( (Model)self, (StreamBuffer)buffer) -> None :
            Loads *this from a binary buffer.
        
        loadFromBinary( (Model)self, (StaticBuffer)buffer) -> None :
            Loads *this from a static binary buffer.
        """
    @staticmethod
    def loadFromString(*args, **kwargs):
        """
        
        loadFromString( (Model)self, (str)string) -> None :
            Parses from the input string the content of the current object.
        """
    @staticmethod
    def loadFromText(*args, **kwargs):
        """
        
        loadFromText( (Model)arg1, (str)filename) -> None :
            Loads *this from a text file.
        """
    @staticmethod
    def loadFromXML(*args, **kwargs):
        """
        
        loadFromXML( (Model)self, (str)filename, (str)tag_name) -> None :
            Loads *this from a XML file.
        """
    @staticmethod
    def saveToBinary(*args, **kwargs):
        """
        
        saveToBinary( (Model)self, (str)filename) -> None :
            Saves *this inside a binary file.
        
        saveToBinary( (Model)self, (StreamBuffer)buffer) -> None :
            Saves *this inside a binary buffer.
        
        saveToBinary( (Model)self, (StaticBuffer)buffer) -> None :
            Saves *this inside a static binary buffer.
        """
    @staticmethod
    def saveToString(*args, **kwargs):
        """
        
        saveToString( (Model)self) -> str :
            Parses the current object to a string.
        """
    @staticmethod
    def saveToText(*args, **kwargs):
        """
        
        saveToText( (Model)arg1, (str)filename) -> None :
            Saves *this inside a text file.
        """
    @staticmethod
    def saveToXML(*args, **kwargs):
        """
        
        saveToXML( (Model)arg1, (str)filename, (str)tag_name) -> None :
            Saves *this inside a XML file.
        """
    @property
    def damping(*args, **kwargs):
        """
        Vector of joint damping parameters.
        """
    @damping.setter
    def damping(*args, **kwargs):
        ...
    @property
    def effortLimit(*args, **kwargs):
        """
        Joint max effort.
        """
    @effortLimit.setter
    def effortLimit(*args, **kwargs):
        ...
    @property
    def frames(*args, **kwargs):
        """
        Vector of frames contained in the model.
        """
    @frames.setter
    def frames(*args, **kwargs):
        ...
    @property
    def friction(*args, **kwargs):
        """
        Vector of joint friction parameters.
        """
    @friction.setter
    def friction(*args, **kwargs):
        ...
    @property
    def gravity(*args, **kwargs):
        """
        Motion vector corresponding to the gravity field expressed in the world Frame.
        """
    @gravity.setter
    def gravity(*args, **kwargs):
        ...
    @property
    def idx_qs(*args, **kwargs):
        ...
    @property
    def idx_vs(*args, **kwargs):
        ...
    @property
    def inertias(*args, **kwargs):
        ...
    @property
    def jointPlacements(*args, **kwargs):
        ...
    @property
    def joints(*args, **kwargs):
        ...
    @property
    def lowerPositionLimit(*args, **kwargs):
        """
        Limit for joint lower position.
        """
    @lowerPositionLimit.setter
    def lowerPositionLimit(*args, **kwargs):
        ...
    @property
    def name(*args, **kwargs):
        ...
    @name.setter
    def name(*args, **kwargs):
        ...
    @property
    def names(*args, **kwargs):
        ...
    @property
    def nbodies(*args, **kwargs):
        ...
    @property
    def nframes(*args, **kwargs):
        ...
    @property
    def njoints(*args, **kwargs):
        ...
    @property
    def nq(*args, **kwargs):
        ...
    @property
    def nqs(*args, **kwargs):
        ...
    @property
    def nv(*args, **kwargs):
        ...
    @property
    def nvs(*args, **kwargs):
        ...
    @property
    def parents(*args, **kwargs):
        ...
    @property
    def referenceConfigurations(*args, **kwargs):
        ...
    @referenceConfigurations.setter
    def referenceConfigurations(*args, **kwargs):
        ...
    @property
    def rotorGearRatio(*args, **kwargs):
        """
        Vector of rotor gear ratio parameters.
        """
    @rotorGearRatio.setter
    def rotorGearRatio(*args, **kwargs):
        ...
    @property
    def rotorInertia(*args, **kwargs):
        """
        Vector of rotor inertia parameters.
        """
    @rotorInertia.setter
    def rotorInertia(*args, **kwargs):
        ...
    @property
    def subtrees(*args, **kwargs):
        """
        Vector of subtrees. subtree[j] corresponds to the subtree supported by the joint j.
        """
    @subtrees.setter
    def subtrees(*args, **kwargs):
        ...
    @property
    def supports(*args, **kwargs):
        """
        Vector of supports. supports[j] corresponds to the list of joints on the path between
        the current *j* to the root of the kinematic tree.
        """
    @supports.setter
    def supports(*args, **kwargs):
        ...
    @property
    def upperPositionLimit(*args, **kwargs):
        """
        Limit for joint upper position.
        """
    @upperPositionLimit.setter
    def upperPositionLimit(*args, **kwargs):
        ...
    @property
    def velocityLimit(*args, **kwargs):
        """
        Joint max velocity.
        """
    @velocityLimit.setter
    def velocityLimit(*args, **kwargs):
        ...
class ModelPool(Boost.Python.instance):
    """
    Pool containing a model and several datas for parallel computations
    """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (ModelPool)self) -> ModelPool :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (ModelPool)self, (dict)memo) -> ModelPool :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self, (Model)model [, (int)size]) -> None :
            Default constructor.
        
        __init__( (object)self, (ModelPool)other) -> None :
            Copy constructor.
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (ModelPool)self) -> ModelPool :
            Returns a copy of *this.
        """
    @staticmethod
    def data(*args, **kwargs):
        """
        
        data( (ModelPool)self, (int)index) -> Data :
            Return a specific data.
        """
    @staticmethod
    def datas(*args, **kwargs):
        """
        
        datas( (ModelPool)self) -> StdVec_Data :
            Returns the data vectors.
        """
    @staticmethod
    def model(*args, **kwargs):
        """
        
        model( (ModelPool)self) -> Model :
            Model contained in the pool.
        """
    @staticmethod
    def resize(*args, **kwargs):
        """
        
        resize( (ModelPool)self, (int)new_size) -> None :
            Resize the pool.
        """
    @staticmethod
    def size(*args, **kwargs):
        """
        
        size( (ModelPool)self) -> int :
            Returns the size of the pool.
        """
    @staticmethod
    def update(*args, **kwargs):
        """
        
        update( (ModelPool)self, (Model)model) -> None :
            Update the model, meaning that all the datas will be refreshed accordingly.
        
        update( (ModelPool)self, (Data)data) -> None :
            Update all the datas with the input data value.
        
        update( (ModelPool)self, (Model)model, (Data)data) -> None :
            Update the model and data together.
        """
class Motion(Boost.Python.instance):
    """
    Motion vectors, in se3 == M^6.
    
    Supported operations ...
    """
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def Random(*args, **kwargs):
        """
        
        Random() -> Motion :
            Returns a random Motion.
        """
    @staticmethod
    def Zero(*args, **kwargs):
        """
        
        Zero() -> Motion :
            Returns a zero Motion.
        """
    @staticmethod
    def __add__(*args, **kwargs):
        """
        
        __add__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __array__(*args, **kwargs):
        """
        
        __array__( (Motion)arg1) -> object
        """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (Motion)self) -> Motion :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (Motion)self, (dict)memo) -> Motion :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (Motion)arg1) -> tuple
        """
    @staticmethod
    def __iadd__(*args, **kwargs):
        """
        
        __iadd__( (object)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor
        
        __init__( (object)self, (numpy.ndarray)linear, (numpy.ndarray)angular) -> None :
            Initialize from linear and angular components of a Motion vector (don't mix the order).
        
        __init__( (object)self, (numpy.ndarray)array) -> None :
            Init from a vector 6 [linear velocity, angular velocity]
        
        __init__( (object)self, (Motion)other) -> None :
            Copy constructor.
        """
    @staticmethod
    def __isub__(*args, **kwargs):
        """
        
        __isub__( (object)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        """
        
        __mul__( (Motion)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __neg__(*args, **kwargs):
        """
        
        __neg__( (Motion)arg1) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (Motion)arg1) -> object
        """
    @staticmethod
    def __rmul__(*args, **kwargs):
        """
        
        __rmul__( (Motion)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (Motion)arg1) -> object
        """
    @staticmethod
    def __sub__(*args, **kwargs):
        """
        
        __sub__( (Motion)arg1, (Motion)arg2) -> object
        """
    @staticmethod
    def __truediv__(*args, **kwargs):
        """
        
        __truediv__( (Motion)arg1, (float)arg2) -> object
        """
    @staticmethod
    def __xor__(*args, **kwargs):
        """
        
        __xor__( (Motion)arg1, (Motion)arg2) -> object
        
        __xor__( (Motion)arg1, (Force)arg2) -> object
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (Motion)self) -> Motion :
            Returns a copy of *this.
        """
    @staticmethod
    def cross(*args, **kwargs):
        """
        
        cross( (Motion)self, (Motion)m) -> Motion :
            Action of *this onto another Motion m. Returns ¨*this x m.
        
        cross( (Motion)self, (Force)f) -> Force :
            Dual action of *this onto a Force f. Returns *this x* f.
        """
    @staticmethod
    def isApprox(*args, **kwargs):
        """
        
        isApprox( (Motion)self, (Motion)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    @staticmethod
    def isZero(*args, **kwargs):
        """
        
        isZero( (Motion)self [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to the zero Motion, within the precision given by prec.
        """
    @staticmethod
    def se3Action(*args, **kwargs):
        """
        
        se3Action( (Motion)self, (SE3)M) -> Motion :
            Returns the result of the action of M on *this.
        """
    @staticmethod
    def se3ActionInverse(*args, **kwargs):
        """
        
        se3ActionInverse( (Motion)self, (SE3)M) -> Motion :
            Returns the result of the action of the inverse of M on *this.
        """
    @staticmethod
    def setRandom(*args, **kwargs):
        """
        
        setRandom( (Motion)self) -> None :
            Set the linear and angular components of *this to random values.
        """
    @staticmethod
    def setZero(*args, **kwargs):
        """
        
        setZero( (Motion)self) -> None :
            Set the linear and angular components of *this to zero.
        """
    @property
    def action(*args, **kwargs):
        """
        Returns the action matrix of *this (acting on Motion).
        """
    @property
    def angular(*args, **kwargs):
        """
        Angular part of a *this, corresponding to the angular velocity in case of a Spatial velocity.
        """
    @angular.setter
    def angular(*args, **kwargs):
        ...
    @property
    def dualAction(*args, **kwargs):
        """
        Returns the dual action matrix of *this (acting on Force).
        """
    @property
    def homogeneous(*args, **kwargs):
        """
        Equivalent homogeneous representation of the Motion vector
        """
    @property
    def linear(*args, **kwargs):
        """
        Linear part of a *this, corresponding to the linear velocity in case of a Spatial velocity.
        """
    @linear.setter
    def linear(*args, **kwargs):
        ...
    @property
    def np(*args, **kwargs):
        ...
    @property
    def vector(*args, **kwargs):
        """
        Returns the components of *this as a 6d vector.
        """
    @vector.setter
    def vector(*args, **kwargs):
        ...
class Quaternion(Boost.Python.instance):
    """
    Quaternion representing rotation.
    
    Supported operations ('q is a Quaternion, 'v' is a Vector3): 'q*q' (rotation composition), 'q*=q', 'q*v' (rotating 'v' by 'q'), 'q==q', 'q!=q', 'q[0..3]'.
    """
    @staticmethod
    def FromTwoVectors(*args, **kwargs):
        """
        
        FromTwoVectors( (numpy.ndarray)a, (numpy.ndarray)b) -> Quaternion :
            Returns the quaternion which transforms a into b through a rotation.
        """
    @staticmethod
    def Identity(*args, **kwargs):
        """
        
        Identity() -> Quaternion :
            Returns a quaternion representing an identity rotation.
        """
    @staticmethod
    def __abs__(*args, **kwargs):
        """
        
        __abs__( (Quaternion)arg1) -> float
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (Quaternion)arg1, (Quaternion)arg2) -> bool
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (Quaternion)arg1, (int)arg2) -> float
        """
    @staticmethod
    def __imul__(*args, **kwargs):
        """
        
        __imul__( (object)arg1, (Quaternion)arg2) -> object
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1, (numpy.ndarray)R) -> object :
            Initialize from rotation matrix.
            	R : a rotation matrix 3x3.
        
        __init__( (object)arg1, (AngleAxis)aa) -> object :
            Initialize from an angle axis.
            	aa: angle axis object.
        
        __init__( (object)arg1, (Quaternion)quat) -> object :
            Copy constructor.
            	quat: a quaternion.
        
        __init__( (object)arg1, (numpy.ndarray)u, (numpy.ndarray)v) -> object :
            Initialize from two vectors u and v
        
        __init__( (object)arg1, (numpy.ndarray)vec4) -> object :
            Initialize from a vector 4D.
            	vec4 : a 4D vector representing quaternion coefficients in the order xyzw.
        
        __init__( (object)arg1) -> object :
            Default constructor
        
        __init__( (object)arg1, (float)w, (float)x, (float)y, (float)z) -> object :
            Initialize from coefficients.
            
            ... note:: The order of coefficients is *w*, *x*, *y*, *z*. The [] operator numbers them differently, 0...4 for *x* *y* *z* *w*!
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__() -> int
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        """
        
        __mul__( (Quaternion)arg1, (Quaternion)arg2) -> object
        
        __mul__( (Quaternion)arg1, (numpy.ndarray)arg2) -> object
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (Quaternion)arg1, (Quaternion)arg2) -> bool
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (Quaternion)arg1) -> str
        """
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (Quaternion)arg1, (int)arg2, (float)arg3) -> None
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (Quaternion)arg1) -> str
        """
    @staticmethod
    def _transformVector(*args, **kwargs):
        """
        
        _transformVector( (Quaternion)self, (numpy.ndarray)vector) -> numpy.ndarray :
            Rotation of a vector by a quaternion.
        """
    @staticmethod
    def angularDistance(*args, **kwargs):
        """
        
        angularDistance( (Quaternion)arg1, (Quaternion)arg2) -> float :
            Returns the angle (in radian) between two rotations.
        """
    @staticmethod
    def assign(*args, **kwargs):
        """
        
        assign( (Quaternion)self, (Quaternion)quat) -> Quaternion :
            Set *this from an quaternion quat and returns a reference to *this.
        
        assign( (Quaternion)self, (AngleAxis)aa) -> Quaternion :
            Set *this from an angle-axis aa and returns a reference to *this.
        """
    @staticmethod
    def coeffs(*args, **kwargs):
        """
        
        coeffs( (Quaternion)self) -> object :
            Returns a vector of the coefficients (x,y,z,w)
        """
    @staticmethod
    def conjugate(*args, **kwargs):
        """
        
        conjugate( (Quaternion)self) -> Quaternion :
            Returns the conjugated quaternion.
            The conjugate of a quaternion represents the opposite rotation.
        """
    @staticmethod
    def dot(*args, **kwargs):
        """
        
        dot( (Quaternion)self, (Quaternion)other) -> float :
            Returns the dot product of *this with an other Quaternion.
            Geometrically speaking, the dot product of two unit quaternions corresponds to the cosine of half the angle between the two rotations.
        """
    @staticmethod
    def inverse(*args, **kwargs):
        """
        
        inverse( (Quaternion)self) -> Quaternion :
            Returns the quaternion describing the inverse rotation.
        """
    @staticmethod
    def isApprox(*args, **kwargs):
        """
        
        isApprox( (Quaternion)self, (Quaternion)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision determined by prec.
        """
    @staticmethod
    def matrix(*args, **kwargs):
        """
        
        matrix( (Quaternion)self) -> numpy.ndarray :
            Returns an equivalent 3x3 rotation matrix. Similar to toRotationMatrix.
        """
    @staticmethod
    def norm(*args, **kwargs):
        """
        
        norm( (Quaternion)self) -> float :
            Returns the norm of the quaternion's coefficients.
        """
    @staticmethod
    def normalize(*args, **kwargs):
        """
        
        normalize( (Quaternion)self) -> Quaternion :
            Normalizes the quaternion *this.
        """
    @staticmethod
    def normalized(*args, **kwargs):
        """
        
        normalized( (Quaternion)self) -> Quaternion :
            Returns a normalized copy of *this.
        """
    @staticmethod
    def setFromTwoVectors(*args, **kwargs):
        """
        
        setFromTwoVectors( (Quaternion)self, (numpy.ndarray)a, (numpy.ndarray)b) -> Quaternion :
            Set *this to be the quaternion which transforms a into b through a rotation.
        """
    @staticmethod
    def setIdentity(*args, **kwargs):
        """
        
        setIdentity( (Quaternion)self) -> Quaternion :
            Set *this to the identity rotation.
        """
    @staticmethod
    def slerp(*args, **kwargs):
        """
        
        slerp( (Quaternion)self, (float)t, (Quaternion)other) -> Quaternion :
            Returns the spherical linear interpolation between the two quaternions *this and other at the parameter t in [0;1].
        """
    @staticmethod
    def squaredNorm(*args, **kwargs):
        """
        
        squaredNorm( (Quaternion)self) -> float :
            Returns the squared norm of the quaternion's coefficients.
        """
    @staticmethod
    def toRotationMatrix(*args, **kwargs):
        """
        
        toRotationMatrix( (Quaternion)arg1) -> numpy.ndarray :
            Returns an equivalent rotation matrix.
        """
    @staticmethod
    def vec(*args, **kwargs):
        """
        
        vec( (Quaternion)self) -> numpy.ndarray :
            Returns a vector expression of the imaginary part (x,y,z).
        """
    @property
    def w(*args, **kwargs):
        """
        The w coefficient.
        """
    @w.setter
    def w(*args, **kwargs):
        ...
    @property
    def x(*args, **kwargs):
        """
        The x coefficient.
        """
    @x.setter
    def x(*args, **kwargs):
        ...
    @property
    def y(*args, **kwargs):
        """
        The y coefficient.
        """
    @y.setter
    def y(*args, **kwargs):
        ...
    @property
    def z(*args, **kwargs):
        """
        The z coefficient.
        """
    @z.setter
    def z(*args, **kwargs):
        ...
class ReferenceFrame(Boost.Python.enum):
    LOCAL: typing.ClassVar[ReferenceFrame]  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL
    LOCAL_WORLD_ALIGNED: typing.ClassVar[ReferenceFrame]  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED
    WORLD: typing.ClassVar[ReferenceFrame]  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.WORLD
    __slots__: typing.ClassVar[tuple] = tuple()
    names: typing.ClassVar[dict]  # value = {'WORLD': pinocchio.pinocchio_pywrap.ReferenceFrame.WORLD, 'LOCAL': pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL, 'LOCAL_WORLD_ALIGNED': pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED}
    values: typing.ClassVar[dict]  # value = {0: pinocchio.pinocchio_pywrap.ReferenceFrame.WORLD, 1: pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL, 2: pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED}
class SE3(Boost.Python.instance):
    """
    SE3 transformation defined by a 3d vector and a rotation matrix.
    """
    __instance_size__: typing.ClassVar[int] = 120
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def Identity(*args, **kwargs):
        """
        
        Identity() -> SE3 :
            Returns the identity transformation.
        """
    @staticmethod
    def Interpolate(*args, **kwargs):
        """
        
        Interpolate( (SE3)A, (SE3)B, (float)alpha) -> SE3 :
            Linear interpolation on the SE3 manifold.
            
            This method computes the linear interpolation between A and B, such that the result C = A + (B-A)*t if it would be applied on classic Euclidian space.
            This operation is very similar to the SLERP operation on Rotations.
            Parameters:
            	A: Initial transformation
            	B: Target transformation
            	alpha: Interpolation factor
        """
    @staticmethod
    def Random(*args, **kwargs):
        """
        
        Random() -> SE3 :
            Returns a random transformation.
        """
    @staticmethod
    def __array__(*args, **kwargs):
        """
        
        __array__( (SE3)arg1) -> numpy.ndarray
        """
    @staticmethod
    def __copy__(*args, **kwargs):
        """
        
        __copy__( (SE3)self) -> SE3 :
            Returns a copy of *this.
        """
    @staticmethod
    def __deepcopy__(*args, **kwargs):
        """
        
        __deepcopy__( (SE3)self, (dict)memo) -> SE3 :
            Returns a deep copy of *this.
        """
    @staticmethod
    def __eq__(*args, **kwargs):
        """
        
        __eq__( (SE3)arg1, (SE3)arg2) -> object
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (SE3)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor.
        
        __init__( (object)self, (numpy.ndarray)rotation, (numpy.ndarray)translation) -> None :
            Initialize from a rotation matrix and a translation vector.
        
        __init__( (object)self, (Quaternion)quat, (numpy.ndarray)translation) -> None :
            Initialize from a quaternion and a translation vector.
        
        __init__( (object)self, (int)int) -> None :
            Init to identity.
        
        __init__( (object)self, (SE3)other) -> None :
            Copy constructor.
        
        __init__( (object)self, (numpy.ndarray)array) -> None :
            Initialize from an homogeneous matrix.
        """
    @staticmethod
    def __invert__(*args, **kwargs):
        """
        
        __invert__( (SE3)arg1) -> SE3 :
            Returns the inverse of *this.
        """
    @staticmethod
    def __mul__(*args, **kwargs):
        """
        
        __mul__( (SE3)arg1, (SE3)arg2) -> object
        
        __mul__( (SE3)arg1, (Motion)arg2) -> Motion
        
        __mul__( (SE3)arg1, (Force)arg2) -> Force
        
        __mul__( (SE3)arg1, (Inertia)arg2) -> Inertia
        
        __mul__( (SE3)arg1, (numpy.ndarray)arg2) -> numpy.ndarray
        """
    @staticmethod
    def __ne__(*args, **kwargs):
        """
        
        __ne__( (SE3)arg1, (SE3)arg2) -> object
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (SE3)arg1) -> object
        """
    @staticmethod
    def __str__(*args, **kwargs):
        """
        
        __str__( (SE3)arg1) -> object
        """
    @staticmethod
    def act(*args, **kwargs):
        """
        
        act( (SE3)self, (numpy.ndarray)point) -> numpy.ndarray :
            Returns a point which is the result of the entry point transforms by *this.
        
        act( (SE3)self, (SE3)M) -> SE3 :
            Returns the result of *this * M.
        
        act( (SE3)self, (Motion)motion) -> Motion :
            Returns the result action of *this onto a Motion.
        
        act( (SE3)self, (Force)force) -> Force :
            Returns the result of *this onto a Force.
        
        act( (SE3)self, (Inertia)inertia) -> Inertia :
            Returns the result of *this onto a Force.
        """
    @staticmethod
    def actInv(*args, **kwargs):
        """
        
        actInv( (SE3)self, (numpy.ndarray)point) -> numpy.ndarray :
            Returns a point which is the result of the entry point by the inverse of *this.
        
        actInv( (SE3)self, (SE3)M) -> SE3 :
            Returns the result of the inverse of *this times M.
        
        actInv( (SE3)self, (Motion)motion) -> Motion :
            Returns the result of the inverse of *this onto a Motion.
        
        actInv( (SE3)self, (Force)force) -> Force :
            Returns the result of the inverse of *this onto an Inertia.
        
        actInv( (SE3)self, (Inertia)inertia) -> Inertia :
            Returns the result of the inverse of *this onto an Inertia.
        """
    @staticmethod
    def copy(*args, **kwargs):
        """
        
        copy( (SE3)self) -> SE3 :
            Returns a copy of *this.
        """
    @staticmethod
    def inverse(*args, **kwargs):
        """
        
        inverse( (SE3)self) -> SE3 :
            Returns the inverse transform
        """
    @staticmethod
    def isApprox(*args, **kwargs):
        """
        
        isApprox( (SE3)self, (SE3)other [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to other, within the precision given by prec.
        """
    @staticmethod
    def isIdentity(*args, **kwargs):
        """
        
        isIdentity( (SE3)self [, (float)prec]) -> bool :
            Returns true if *this is approximately equal to the identity placement, within the precision given by prec.
        """
    @staticmethod
    def setIdentity(*args, **kwargs):
        """
        
        setIdentity( (SE3)self) -> None :
            Set *this to the identity placement.
        """
    @staticmethod
    def setRandom(*args, **kwargs):
        """
        
        setRandom( (SE3)self) -> None :
            Set *this to a random placement.
        """
    @staticmethod
    def toActionMatrix(*args, **kwargs):
        """
        
        toActionMatrix( (SE3)self) -> numpy.ndarray :
            Returns the related action matrix (acting on Motion).
        """
    @staticmethod
    def toActionMatrixInverse(*args, **kwargs):
        """
        
        toActionMatrixInverse( (SE3)self) -> numpy.ndarray :
            Returns the inverse of the action matrix (acting on Motion).
            This is equivalent to do m.inverse().toActionMatrix()
        """
    @staticmethod
    def toDualActionMatrix(*args, **kwargs):
        """
        
        toDualActionMatrix( (SE3)self) -> numpy.ndarray :
            Returns the related dual action matrix (acting on Force).
        """
    @property
    def action(*args, **kwargs):
        """
        Returns the related action matrix (acting on Motion).
        """
    @property
    def actionInverse(*args, **kwargs):
        """
        Returns the inverse of the action matrix (acting on Motion).
        This is equivalent to do m.inverse().action
        """
    @property
    def dualAction(*args, **kwargs):
        """
        Returns the related dual action matrix (acting on Force).
        """
    @property
    def homogeneous(*args, **kwargs):
        """
        Returns the equivalent homegeneous matrix (acting on SE3).
        """
    @property
    def np(*args, **kwargs):
        ...
    @property
    def rotation(*args, **kwargs):
        """
        The rotation part of the transformation.
        """
    @rotation.setter
    def rotation(*args, **kwargs):
        ...
    @property
    def translation(*args, **kwargs):
        """
        The translation part of the transformation.
        """
    @translation.setter
    def translation(*args, **kwargs):
        ...
class StdMap_String_VectorXd(Boost.Python.instance):
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 72
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdMap_String_VectorXd)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdMap_String_VectorXd)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdMap_String_VectorXd)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdMap_String_VectorXd)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdMap_String_VectorXd)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
class StdVec_Bool(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 72
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Bool)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Bool)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Bool)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Bool)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Bool)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Bool)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Bool)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Bool)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_CollisionPair(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_CollisionPair)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_CollisionPair)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_CollisionPair)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_CollisionPair)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_CollisionPair)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_CollisionPair)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_CollisionPair)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_CollisionPair)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_Data(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Data)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Data)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Data)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Data)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Data)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Data)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Data)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Data)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_Double(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Double)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Double)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Double)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Double)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Double)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Double)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Double)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Double)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_Force(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Force)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Force)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Force)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Force)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Force)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Force)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Force)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Force)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class StdVec_Frame(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Frame)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Frame)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Frame)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Frame)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Frame)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Frame)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Frame)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Frame)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class StdVec_GeometryData(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_GeometryData)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_GeometryData)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_GeometryData)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_GeometryData)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_GeometryData)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_GeometryData)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_GeometryData)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_GeometryData)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_GeometryModel(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_GeometryModel)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_GeometryModel)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_GeometryModel)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_GeometryModel)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_GeometryModel)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_GeometryModel)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_GeometryModel)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_GeometryModel)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_GeometryObject(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_GeometryObject)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_GeometryObject)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_GeometryObject)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_GeometryObject)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_GeometryObject)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_GeometryObject)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_GeometryObject)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_GeometryObject)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class StdVec_Index(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Index)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Index)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Index)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Index)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Index)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Index)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Index)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Index)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_IndexVector(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_IndexVector)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_IndexVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_IndexVector)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_IndexVector)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_IndexVector)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_IndexVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_IndexVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_IndexVector)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_Inertia(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Inertia)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Inertia)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Inertia)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Inertia)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Inertia)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Inertia)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Inertia)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Inertia)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class StdVec_Int(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Int)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Int)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Int)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Int)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Int)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Int)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Int)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Int)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_JointModelVector(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_JointModelVector)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_JointModelVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_JointModelVector)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_JointModelVector)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_JointModelVector)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_JointModelVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_JointModelVector)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_JointModelVector)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class StdVec_Matrix6x(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Matrix6x)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Matrix6x)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Matrix6x)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Matrix6x)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Matrix6x)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Matrix6x)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Matrix6x)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Matrix6x)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class StdVec_Motion(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Motion)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Motion)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Motion)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Motion)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Motion)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Motion)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Motion)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Motion)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class StdVec_SE3(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_SE3)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_SE3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_SE3)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_SE3)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_SE3)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_SE3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_SE3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_SE3)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class StdVec_StdString(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_StdString)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_StdString)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_StdString)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_StdString)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_StdString)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_StdString)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_StdString)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_StdString)self) -> list :
            Returns the std::vector as a Python list.
        """
class StdVec_Vector3(Boost.Python.instance):
    """
    """
    __getstate_manages_dict__: typing.ClassVar[bool] = True
    __instance_size__: typing.ClassVar[int] = 56
    __safe_for_unpickling__: typing.ClassVar[bool] = True
    @staticmethod
    def __contains__(*args, **kwargs):
        """
        
        __contains__( (StdVec_Vector3)arg1, (object)arg2) -> bool
        """
    @staticmethod
    def __delitem__(*args, **kwargs):
        """
        
        __delitem__( (StdVec_Vector3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def __getinitargs__(*args, **kwargs):
        """
        
        __getinitargs__( (StdVec_Vector3)arg1) -> tuple
        """
    @staticmethod
    def __getitem__(*args, **kwargs):
        """
        
        __getitem__( (object)arg1, (object)arg2) -> object
        
        __getitem__( (object)arg1, (object)arg2) -> object
        """
    @staticmethod
    def __getstate__(*args, **kwargs):
        """
        
        __getstate__( (object)arg1) -> tuple
        """
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __iter__(*args, **kwargs):
        """
        
        __iter__( (object)arg1) -> object
        """
    @staticmethod
    def __len__(*args, **kwargs):
        """
        
        __len__( (StdVec_Vector3)arg1) -> int
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setitem__(*args, **kwargs):
        """
        
        __setitem__( (StdVec_Vector3)arg1, (object)arg2, (object)arg3) -> None
        """
    @staticmethod
    def __setstate__(*args, **kwargs):
        """
        
        __setstate__( (object)arg1, (tuple)arg2) -> None
        """
    @staticmethod
    def append(*args, **kwargs):
        """
        
        append( (StdVec_Vector3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def extend(*args, **kwargs):
        """
        
        extend( (StdVec_Vector3)arg1, (object)arg2) -> None
        """
    @staticmethod
    def tolist(*args, **kwargs):
        """
        
        tolist( (StdVec_Vector3)self) -> list :
            Returns the aligned_vector as a Python list.
        """
class map_indexing_suite_StdMap_String_VectorXd_entry(Boost.Python.instance):
    __instance_size__: typing.ClassVar[int] = 72
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)arg1) -> None
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __repr__(*args, **kwargs):
        """
        
        __repr__( (map_indexing_suite_StdMap_String_VectorXd_entry)arg1) -> object
        """
    @staticmethod
    def data(*args, **kwargs):
        """
        
        data( (map_indexing_suite_StdMap_String_VectorXd_entry)arg1) -> object
        """
    @staticmethod
    def key(*args, **kwargs):
        """
        
        key( (map_indexing_suite_StdMap_String_VectorXd_entry)arg1) -> str
        """
def Hlog3(*args, **kwargs):
    """
    
    Hlog3( (numpy.ndarray)R, (numpy.ndarray)v) -> numpy.ndarray :
        Vector v to be multiplied to the hessian
    """
def Jexp3(*args, **kwargs):
    """
    
    Jexp3( (numpy.ndarray)v) -> numpy.ndarray :
        Jacobian of exp(R) which maps from the tangent of SO(3) at exp(v) to the tangent of SO(3) at Identity.
    """
def Jexp6(*args, **kwargs):
    """
    
    Jexp6( (Motion)v) -> numpy.ndarray :
        Jacobian of exp(v) which maps from the tangent of SE(3) at exp(v) to the tangent of SE(3) at Identity.
    
    Jexp6( (numpy.ndarray)v) -> numpy.ndarray :
        Jacobian of exp(v) which maps from the tangent of SE(3) at exp(v) to the tangent of SE(3) at Identity.
    """
def Jlog3(*args, **kwargs):
    """
    
    Jlog3( (numpy.ndarray)R) -> numpy.ndarray :
        Jacobian of log(R) which maps from the tangent of SO(3) at R to the tangent of SO(3) at Identity.
    """
def Jlog6(*args, **kwargs):
    """
    
    Jlog6( (SE3)M) -> numpy.ndarray :
        Jacobian of log(M) which maps from the tangent of SE(3) at M to the tangent of SE(3) at Identity.
    """
def SE3ToXYZQUAT(*args, **kwargs):
    """
    
    SE3ToXYZQUAT( (SE3)arg1) -> numpy.ndarray :
        M
    """
def SE3ToXYZQUATtuple(*args, **kwargs):
    """
    
    SE3ToXYZQUATtuple( (SE3)arg1) -> tuple :
        M
    """
def XYZQUATToSE3(*args, **kwargs):
    """
    
    XYZQUATToSE3( (tuple)tuple) -> SE3 :
        Reverse function of SE3ToXYZQUAT: convert [X,Y,Z,x,y,z,w] to an SE3 element.
    
    XYZQUATToSE3( (list)list) -> SE3 :
        Reverse function of SE3ToXYZQUAT: convert [X,Y,Z,x,y,z,w] to an SE3 element.
    
    XYZQUATToSE3( (numpy.ndarray)array) -> SE3 :
        Reverse function of SE3ToXYZQUAT: convert [X,Y,Z,x,y,z,w] to an SE3 element.
    """
def aba(*args, **kwargs):
    """
    
    aba( (Model)Model, (Data)Data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau) -> numpy.ndarray :
        Compute ABA, store the result in Data::ddq and return it.
    
    aba( (Model)Model, (Data)Data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau, (StdVec_Force)f_ext) -> numpy.ndarray :
        Compute ABA with external forces, store the result in Data::ddq and return it.
    
    aba( (int)num_thread, (ModelPool)pool, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> numpy.ndarray :
        Computes in parallel the ABA and returns the result.
        
        Parameters:
        	num_thread: number of threads used for the computation
        	pool: pool of model/data
        	q: the joint configuration vector (size model.nq x batch_size)
        	v: the joint velocity vector (size model.nv x batch_size)
        	tau: the joint torque vector (size model.nv x batch_size)
        
    
    aba( (int)num_thread, (ModelPool)pool, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a, (numpy.ndarray)tau) -> None :
        Computes in parallel the ABA, store the result in a.
        
        Parameters:
        	num_thread: number of threads used for the computation
        	pool: pool of model/data
        	q: the joint configuration vector (size model.nq x batch_size)
        	v: the joint velocity vector (size model.nv x batch_size)
        	tau: the joint torque vector (size model.nv x batch_size)
        	a: the resulting joint acceleration vectors (size model.nv x batch_size)
        
    """
def appendModel(*args, **kwargs):
    """
    
    appendModel( (Model)modelA, (Model)modelB, (int)frame_in_modelA, (SE3)aMb) -> Model :
        Append a child model into a parent model, after a specific frame given by its index.
        
        Parameters:
        	modelA: the parent model
        	modelB: the child model
        	frameInModelA:  index of the frame of modelA where to append modelB
        	aMb: pose of modelB universe joint (index 0) in frameInModelA
        
    
    appendModel( (Model)modelA, (Model)modelB, (GeometryModel)geomModelA, (GeometryModel)geomModelB, (int)frame_in_modelA, (SE3)aMb) -> tuple :
        Append a child (geometry) model into a parent (geometry) model, after a specific frame given by its index.
        
        Parameters:
        	modelA: the parent model
        	modelB: the child model
        	geomModelA: the parent geometry model
        	geomModelB: the child geometry model
        	frameInModelA:  index of the frame of modelA where to append modelB
        	aMb: pose of modelB universe joint (index 0) in frameInModelA
        
    """
def bodyRegressor(*args, **kwargs):
    """
    
    bodyRegressor( (Motion)velocity, (Motion)acceleration) -> numpy.ndarray :
        Computes the regressor for the dynamic parameters of a single rigid body.
        The result is such that Ia + v x Iv = bodyRegressor(v,a) * I.toDynamicParameters()
        
        Parameters:
        	velocity: spatial velocity of the rigid body
        	acceleration: spatial acceleration of the rigid body
        
    """
def buildGeomFromUrdf(*args, **kwargs):
    """
    
    buildGeomFromUrdf( (Model)model, (str)urdf_filename, (GeometryType)geom_type [, (object)geometry_model=None [, (object)package_dirs=None [, (object)mesh_loader=None]]]) -> GeometryModel :
        Parse the URDF file given as input looking for the geometry of the given input model and
        and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in a GeometryModel object.
        Parameters:
        	model: model of the robot
        
        urdf_filename: path to the URDF file containing the model of the robot
        	geom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).
        	geometry_model: if provided, this geometry model will be used to store the parsed information instead of creating a new one
        	package_dirs: either a single path or a vector of paths pointing to folders containing the model of the robot
        	mesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).
        
        Retuns:
        	a new GeometryModel if `geometry_model` is None else `geometry_model` (that has been updated).
        
    """
def buildGeomFromUrdfString(*args, **kwargs):
    """
    
    buildGeomFromUrdfString( (Model)model, (str)urdf_string, (GeometryType)geom_type [, (object)geometry_model=None [, (object)package_dirs=None [, (object)mesh_loader=None]]]) -> GeometryModel :
        Parse the URDF file given as input looking for the geometry of the given input model and
        and store either the collision geometries (GeometryType.COLLISION) or the visual geometries (GeometryType.VISUAL) in a GeometryModel object.
        Parameters:
        	model: model of the robot
        
        urdf_string: a string containing the URDF model of the robot
        	geom_type: type of geometry to extract from the URDF file (either the VISUAL for display or the COLLISION for collision detection).
        	geometry_model: if provided, this geometry model will be used to store the parsed information instead of creating a new one
        	package_dirs: either a single path or a vector of paths pointing to folders containing the model of the robot
        	mesh_loader: an hpp-fcl mesh loader (to load only once the related geometries).
        
        Retuns:
        	a new GeometryModel if `geometry_model` is None else `geometry_model` (that has been updated).
        
    """
def buildModelFromUrdf(*args, **kwargs):
    """
    
    buildModelFromUrdf( (str)urdf_filename, (JointModel)root_joint) -> Model :
        Parse the URDF file given in input and return a pinocchio Model starting with the given root joint.
    
    buildModelFromUrdf( (str)urdf_filename) -> Model :
        Parse the URDF file given in input and return a pinocchio Model.
    
    buildModelFromUrdf( (str)urdf_filename, (Model)model) -> Model :
        Append to a given model a URDF structure given by its filename.
    
    buildModelFromUrdf( (str)urdf_filename, (JointModel)root_joint, (Model)model) -> Model :
        Append to a given model a URDF structure given by its filename and the root joint.
        Remark: In the URDF format, a joint of type fixed can be defined. For efficiency reasons,it is treated as operational frame and not as a joint of the model.
    """
def buildModelFromXML(*args, **kwargs):
    """
    
    buildModelFromXML( (str)urdf_xml_stream, (JointModel)root_joint) -> Model :
        Parse the URDF XML stream given in input and return a pinocchio Model starting with the given root joint.
    
    buildModelFromXML( (str)urdf_xml_stream, (JointModel)root_joint, (Model)model) -> Model :
        Parse the URDF XML stream given in input and append it to the input model with the given interfacing joint.
    
    buildModelFromXML( (str)urdf_xml_stream) -> Model :
        Parse the URDF XML stream given in input and return a pinocchio Model.
    
    buildModelFromXML( (str)urdf_xml_stream, (Model)model) -> Model :
        Parse the URDF XML stream given in input and append it to the input model.
    """
def buildReducedModel(*args, **kwargs):
    """
    
    buildReducedModel( (Model)model, (StdVec_Index)list_of_joints_to_lock, (numpy.ndarray)reference_configuration) -> Model :
        Build a reduce model from a given input model and a list of joint to lock.
        
        Parameters:
        	model: input kinematic modell to reduce
        	list_of_joints_to_lock: list of joint indexes to lock
        	reference_configuration: reference configuration to compute the placement of the lock joints
        
    
    buildReducedModel( (Model)model, (GeometryModel)geom_model, (StdVec_Index)list_of_joints_to_lock, (numpy.ndarray)reference_configuration) -> tuple :
        Build a reduced model and a reduced geometry model from a given input model,an input geometry model and a list of joints to lock.
        
        Parameters:
        	model: input kinematic model to reduce
        	geom_model: input geometry model to reduce
        	list_of_joints_to_lock: list of joint indexes to lock
        	reference_configuration: reference configuration to compute the placement of the locked joints
        
    
    buildReducedModel( (Model)model, (StdVec_GeometryModel)list_of_geom_models, (StdVec_Index)list_of_joints_to_lock, (numpy.ndarray)reference_configuration) -> tuple :
        Build a reduced model and the related reduced geometry models from a given input model,a list of input geometry models and a list of joints to lock.
        
        Parameters:
        	model: input kinematic model to reduce
        	list_of_geom_models: input geometry models to reduce
        	list_of_joints_to_lock: list of joint indexes to lock
        	reference_configuration: reference configuration to compute the placement of the locked joints
        
    """
def buildSampleGeometryModelHumanoid(*args, **kwargs):
    """
    
    buildSampleGeometryModelHumanoid( (Model)model) -> GeometryModel :
        Generate a (hard-coded) geometry model of a simple humanoid.
    """
def buildSampleGeometryModelManipulator(*args, **kwargs):
    """
    
    buildSampleGeometryModelManipulator( (Model)model) -> GeometryModel :
        Generate a (hard-coded) geometry model of a simple manipulator.
    """
def buildSampleModelHumanoid(*args, **kwargs):
    """
    
    buildSampleModelHumanoid() -> Model :
        Generate a (hard-coded) model of a simple humanoid.
    
    buildSampleModelHumanoid( (bool)using_free_flyer) -> Model :
        Generate a (hard-coded) model of a simple humanoid.
    """
def buildSampleModelHumanoidRandom(*args, **kwargs):
    """
    
    buildSampleModelHumanoidRandom() -> Model :
        Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint placements.
        Only meant for unit tests.
    
    buildSampleModelHumanoidRandom( (bool)using_free_flyer) -> Model :
        Generate a (hard-coded) model of a humanoid robot with 6-DOF limbs and random joint placements.
        Only meant for unit tests.
    """
def buildSampleModelManipulator(*args, **kwargs):
    """
    
    buildSampleModelManipulator() -> Model :
        Generate a (hard-coded) model of a simple manipulator.
    """
def ccrba(*args, **kwargs):
    """
    
    ccrba( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the centroidal mapping, the centroidal momentum and the Centroidal Composite Rigid Body Inertia, puts the result in Data and returns the centroidal mapping.For the same price, it also computes the total joint jacobians (data.J).
    """
def centerOfMass(*args, **kwargs):
    """
    
    centerOfMass( (Model)model, (Data)data, (numpy.ndarray)q [, (bool)compute_subtree_coms]) -> numpy.ndarray :
        Compute the center of mass, putting the result in Data and return it.If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    
    centerOfMass( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v [, (bool)compute_subtree_coms]) -> numpy.ndarray :
        Computes the center of mass position and velocity by storing the result in Data. It returns the center of mass position expressed in the WORLD frame.
        If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    
    centerOfMass( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a [, (bool)compute_subtree_coms]) -> numpy.ndarray :
        Computes the center of mass position, velocity and acceleration by storing the result in Data. It returns the center of mass position expressed in the WORLD frame.
        If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    
    centerOfMass( (Model)Model, (Data)Data, (int)kinematic_level [, (bool)computeSubtreeComs If true, the algorithm computes also the center of mass of the subtrees]) -> None :
        Computes the center of mass position, velocity and acceleration of a given model according to the current kinematic values contained in data and the requested kinematic_level.
        If kinematic_level = 0, computes the CoM position, if kinematic_level = 1, also computes the CoM velocity and if kinematic_level = 2, it also computes the CoM acceleration.
    
    centerOfMass( (Model)model, (Data)data, (KinematicLevel)kinematic_level [, (bool)compute_subtree_coms]) -> numpy.ndarray :
        Computes the center of mass position, velocity or acceleration of a given model according to the current kinematic values contained in data and the requested kinematic_level.
        If kinematic_level = POSITION, computes the CoM position, if kinematic_level = VELOCITY, also computes the CoM velocity and if kinematic_level = ACCELERATION, it also computes the CoM acceleration.
        If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    
    centerOfMass( (Model)model, (Data)data [, (bool)compute_subtree_coms]) -> numpy.ndarray :
        Computes the center of mass position, velocity and acceleration of a given model according to the current kinematic values contained in data.
        If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    """
def checkVersionAtLeast(*args, **kwargs):
    """
    
    checkVersionAtLeast( (int)major, (int)minor, (int)patch) -> bool :
        Checks if the current version of Pinocchio is at least the version provided by the input arguments.
    """
def computeABADerivatives(*args, **kwargs):
    """
    
    computeABADerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau) -> tuple :
        Computes the ABA derivatives, store the result in data.ddq_dq, data.ddq_dv and data.Minv (aka ddq_dtau)
        which correspond to the partial derivatives of the joint acceleration vector output with respect to the joint configuration,
        velocity and torque vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	tau: the joint torque vector (size model.nv)
        
        Returns: (ddq_dq, ddq_dv, ddq_da)
    
    computeABADerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau, (StdVec_Force)fext) -> tuple :
        Computes the ABA derivatives with external contact foces,
        store the result in data.ddq_dq, data.ddq_dv and data.Minv (aka ddq_dtau)
        which correspond to the partial derivatives of the acceleration output with respect to the joint configuration,
        velocity and torque vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	tau: the joint torque vector (size model.nv)
        	fext: list of external forces expressed in the local frame of the joints (size model.njoints)
        
        Returns: (ddq_dq, ddq_dv, ddq_da)
    """
def computeAllTerms(*args, **kwargs):
    """
    
    computeAllTerms( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> None :
        Compute all the terms M, non linear effects, center of mass quantities, centroidal quantities and Jacobians inin the same loop and store the results in data.
        This algorithm is equivalent to calling:
        	- forwardKinematics
        	- crba
        	- nonLinearEffects
        	- computeJointJacobians
        	- centerOfMass
        	- jacobianCenterOfMass
        	- ccrba
        	- computeKineticEnergy
        	- computePotentialEnergy
        	- computeGeneralizedGravity
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """
def computeBodyRadius(*args, **kwargs):
    """
    
    computeBodyRadius( (Model)model, (GeometryModel)geometry_model, (GeometryData)geometry_data) -> None :
        Compute the radius of the geometry volumes attached to every joints.
    """
def computeCentroidalDynamicsDerivatives(*args, **kwargs):
    """
    
    computeCentroidalDynamicsDerivatives( (Model)Model, (Data)Data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> tuple :
        Computes the analytical derivatives of the centroidal dynamics
        with respect to the joint configuration vector, velocity and acceleration.
    """
def computeCentroidalMap(*args, **kwargs):
    """
    
    computeCentroidalMap( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes the centroidal mapping, puts the result in Data.Ag and returns the centroidal mapping.
        For the same price, it also computes the total joint jacobians (data.J).
    """
def computeCentroidalMapTimeVariation(*args, **kwargs):
    """
    
    computeCentroidalMapTimeVariation( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the time derivative of the centroidal momentum matrix Ag, puts the result in Data.Ag and returns the centroidal mapping.
        For the same price, it also computes the centroidal momentum matrix (data.Ag), the total joint jacobians (data.J) and the related joint jacobians time derivative (data.dJ)
    """
def computeCentroidalMomentum(*args, **kwargs):
    """
    
    computeCentroidalMomentum( (Model)model, (Data)data) -> Force :
        Computes the Centroidal momentum, a.k.a. the total momentum of the system expressed around the center of mass.
    
    computeCentroidalMomentum( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> Force :
        Computes the Centroidal momentum, a.k.a. the total momentum of the system expressed around the center of mass.
    """
def computeCentroidalMomentumTimeVariation(*args, **kwargs):
    """
    
    computeCentroidalMomentumTimeVariation( (Model)model, (Data)data) -> Force :
        Computes the Centroidal momentum and its time derivatives, a.k.a. the total momentum of the system and its time derivative expressed around the center of mass.
    
    computeCentroidalMomentumTimeVariation( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> Force :
        Computes the Centroidal momentum and its time derivatives, a.k.a. the total momentum of the system and its time derivative expressed around the center of mass.
    """
def computeCollision(*args, **kwargs):
    """
    
    computeCollision( (GeometryModel)geometry_model, (GeometryData)geometry_data, (int)pair_index) -> bool :
        Check if the collision objects of a collision pair for a given Geometry Model and Data are in collision.
        The collision pair is given by the two index of the collision objects.
    """
def computeCollisions(*args, **kwargs):
    """
    
    computeCollisions( (GeometryModel)geometry_model, (GeometryData)geometry_data, (bool)stop_at_first_collision) -> bool :
        Determine if collision pairs are effectively in collision.
    
    computeCollisions( (Model)model, (Data)data, (GeometryModel)geometry_model, (GeometryData)geometry_data, (numpy.ndarray)q, (bool)stop_at_first_collision) -> bool :
        Update the geometry for a given configuration and determine if all collision pairs are effectively in collision or not.
    
    computeCollisions( (int)num_thread, (GeometryModel)geometry_model, (GeometryData)geometry_data [, (bool)stop_at_first_collision]) -> bool :
        Evaluates in parallel the collisions for a single data and returns the result.
        
        Parameters:
        	num_thread: number of threads used for the computation
        	geometry_model: the geometry model
        	geometry_data: the geometry data
        	stop_at_first_collision: if set to true, stops when encountering the first collision.
        
    
    computeCollisions( (int)num_thread, (Model)model, (Data)data, (GeometryModel)geometry_model, (GeometryData)geometry_data, (numpy.ndarray)q [, (bool)stop_at_first_collision]) -> bool :
        Evaluates in parallel the collisions for a single data and returns the result.
        
        Parameters:
        	num_thread: number of threads used for the computation
        	model: the kinematic model
        	data: the data associated to the model
        	geometry_model: the geometry model
        	geometry_data: the geometry data associated to the tgeometry_model
        	q: the joint configuration vector (size model.nq)
        	stop_at_first_collision: if set to true, stops when encountering the first collision.
        
    
    computeCollisions( (int)num_thread, (GeometryPool)pool, (numpy.ndarray)q [, (bool)stop_at_first_collision]) -> numpy.ndarray :
        Evaluates in parallel the collisions and returns the result.
        
        Parameters:
        	num_thread: number of threads used for the computation
        	pool: pool of geometry model/ geometry data
        	q: the joint configuration vector (size model.nq x batch_size)
        	stop_at_first_collision: if set to true, stop when encountering the first collision in a batch element.
        
    
    computeCollisions( (int)num_thread, (GeometryPool)pool, (numpy.ndarray)q, (numpy.ndarray)res [, (bool)stop_at_first_collision]) -> None :
        Evaluates in parallel the collisions and stores the result in res.
        
        Parameters:
        	num_thread: number of threads used for the computation
        	pool: pool of geometry model/ geometry data
        	q: the joint configuration vector (size model.nq x batch_size)
        	res: the resulting collision vector (batch_size)
        	stop_at_first_collision: if set to true, stop when encountering the first collision in a batch element.
        
    """
def computeCoriolisMatrix(*args, **kwargs):
    """
    
    computeCoriolisMatrix( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Compute the Coriolis Matrix C(q,v) of the Lagrangian dynamics, store the result in data.C and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """
def computeDistance(*args, **kwargs):
    """
    
    computeDistance( (GeometryModel)geometry_model, (GeometryData)geometry_data, (int)pair_index) -> DistanceResult :
        Compute the distance between the two geometry objects of a given collision pair for a GeometryModel and associated GeometryData.
    """
def computeDistances(*args, **kwargs):
    """
    
    computeDistances( (GeometryModel)geometry_model, (GeometryData)geometry_data) -> int :
        Compute the distance between each collision pair for a given GeometryModel and associated GeometryData.
    
    computeDistances( (Model)model, (Data)data, (GeometryModel)geometry_model, (GeometryData)geometry_data, (numpy.ndarray)q) -> int :
        Update the geometry for a given configuration and compute the distance between each collision pair
    """
def computeForwardKinematicsDerivatives(*args, **kwargs):
    """
    
    computeForwardKinematicsDerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> None :
        Computes all the terms required to compute the derivatives of the placement, spatial velocity and acceleration
        for any joint of the model.
        The results are stored in data.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
    """
def computeFrameJacobian(*args, **kwargs):
    """
    
    computeFrameJacobian( (Model)model, (Data)data, (numpy.ndarray)q, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the Jacobian of the frame given by its frame_id in the coordinate system given by reference_frame.
        
    
    computeFrameJacobian( (Model)model, (Data)data, (numpy.ndarray)q, (int)frame_id) -> numpy.ndarray :
        Computes the Jacobian of the frame given by its frame_id.
        The columns of the Jacobian are expressed in the coordinates system of the Frame itself.
        In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,where v is the joint velocity.
    """
def computeFrameKinematicRegressor(*args, **kwargs):
    """
    
    computeFrameKinematicRegressor( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the kinematic regressor that links the joint placement variations of the whole kinematic tree to the placement variation of the frame given as input.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	frame_id: index of the frame
        	reference_frame: reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD)
        
    """
def computeGeneralizedGravity(*args, **kwargs):
    """
    
    computeGeneralizedGravity( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Compute the generalized gravity contribution g(q) of the Lagrangian dynamics, store the result in data.g and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    """
def computeGeneralizedGravityDerivatives(*args, **kwargs):
    """
    
    computeGeneralizedGravityDerivatives( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes the partial derivative of the generalized gravity contribution
        with respect to the joint configuration.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        Returns: dtau_statique_dq
        
    """
def computeJointJacobian(*args, **kwargs):
    """
    
    computeJointJacobian( (Model)model, (Data)data, (numpy.ndarray)q, (int)joint_id) -> numpy.ndarray :
        Computes the Jacobian of a specific joint frame expressed in the local frame of the joint according to the given input configuration.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	joint_id: index of the joint
        
    """
def computeJointJacobians(*args, **kwargs):
    """
    
    computeJointJacobians( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes the full model Jacobian, i.e. the stack of all the motion subspaces expressed in the coordinate world frame.
        The result is accessible through data.J. This function computes also the forward kinematics of the model.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    
    computeJointJacobians( (Model)model, (Data)data) -> numpy.ndarray :
        Computes the full model Jacobian, i.e. the stack of all motion subspace expressed in the world frame.
        The result is accessible through data.J. This function assumes that forward kinematics (pinocchio.forwardKinematics) has been called first.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        
    """
def computeJointJacobiansTimeVariation(*args, **kwargs):
    """
    
    computeJointJacobiansTimeVariation( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the full model Jacobian variations with respect to time. It corresponds to dJ/dt which depends both on q and v. It also computes the joint Jacobian of the model (similar to computeJointJacobians).The result is accessible through data.dJ and data.J.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """
def computeJointKinematicRegressor(*args, **kwargs):
    """
    
    computeJointKinematicRegressor( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame, (SE3)placement) -> numpy.ndarray :
        Computes the kinematic regressor that links the joint placements variations of the whole kinematic tree to the placement variation of the frame rigidly attached to the joint and given by its placement w.r.t. to the joint frame.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD)
        	placement: relative placement to the joint frame
        
    
    computeJointKinematicRegressor( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the kinematic regressor that links the joint placement variations of the whole kinematic tree to the placement variation of the joint given as input.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the result is expressed (LOCAL, LOCAL_WORLD_ALIGNED or WORLD)
        
    """
def computeJointTorqueRegressor(*args, **kwargs):
    """
    
    computeJointTorqueRegressor( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> numpy.ndarray :
        Compute the joint torque regressor that links the joint torque to the dynamic parameters of each link according to the current the robot motion,
        store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
    """
def computeKKTContactDynamicMatrixInverse(*args, **kwargs):
    """
    
    computeKKTContactDynamicMatrixInverse( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)J [, (float)damping]) -> numpy.ndarray :
        Computes the inverse of the constraint matrix [[M J^T], [J 0]].
    """
def computeKineticEnergy(*args, **kwargs):
    """
    
    computeKineticEnergy( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> float :
        Computes the forward kinematics and the kinematic energy of the model for the given joint configuration and velocity given as input. The result is accessible through data.kinetic_energy.
    
    computeKineticEnergy( (Model)model, (Data)data) -> float :
        Computes the kinematic energy of the model for the given joint placement and velocity stored in data. The result is accessible through data.kinetic_energy.
    """
def computeMinverse(*args, **kwargs):
    """
    
    computeMinverse( (Model)Model, (Data)Data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes the inverse of the joint space inertia matrix using a variant of the Articulated Body algorithm.
        The result is stored in data.Minv.
    """
def computePotentialEnergy(*args, **kwargs):
    """
    
    computePotentialEnergy( (Model)model, (Data)data, (numpy.ndarray)q) -> float :
        Computes the potential energy of the model for the given the joint configuration given as input. The result is accessible through data.potential_energy.
    
    computePotentialEnergy( (Model)model, (Data)data) -> float :
        Computes the potential energy of the model for the given joint placement stored in data. The result is accessible through data.potential_energy.
    """
def computeRNEADerivatives(*args, **kwargs):
    """
    
    computeRNEADerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> tuple :
        Computes the RNEA partial derivatives, store the result in data.dtau_dq, data.dtau_dv and data.M (aka dtau_da)
        which correspond to the partial derivatives of the torque output with respect to the joint configuration,
        velocity and acceleration vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
        Returns: (dtau_dq, dtau_dv, dtau_da)
        
    
    computeRNEADerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a, (StdVec_Force)fext) -> tuple :
        Computes the RNEA partial derivatives with external contact foces,
        store the result in data.dtau_dq, data.dtau_dv and data.M (aka dtau_da)
        which correspond to the partial derivatives of the torque output with respect to the joint configuration,
        velocity and acceleration vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        	fext: list of external forces expressed in the local frame of the joints (size model.njoints)
        
        Returns: (dtau_dq, dtau_dv, dtau_da)
        
    """
def computeStaticRegressor(*args, **kwargs):
    """
    
    computeStaticRegressor( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Compute the static regressor that links the inertia parameters of the system to its center of mass position,
        store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    """
def computeStaticTorque(*args, **kwargs):
    """
    
    computeStaticTorque( (Model)model, (Data)data, (numpy.ndarray)q, (StdVec_Force)fext) -> numpy.ndarray :
        Computes the generalized static torque contribution g(q) - J.T f_ext of the Lagrangian dynamics, store the result in data.tau and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	fext: list of external forces expressed in the local frame of the joints (size model.njoints)
        
    """
def computeStaticTorqueDerivatives(*args, **kwargs):
    """
    
    computeStaticTorqueDerivatives( (Model)model, (Data)data, (numpy.ndarray)q, (StdVec_Force)fext) -> numpy.ndarray :
        Computes the partial derivative of the generalized gravity and external forces contributions (a.k.a static torque vector)
        with respect to the joint configuration.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	fext: list of external forces expressed in the local frame of the joints (size model.njoints)
        Returns: dtau_statique_dq
        
    """
def computeSubtreeMasses(*args, **kwargs):
    """
    
    computeSubtreeMasses( (Model)model, (Data)data) -> None :
        Compute the mass of each kinematic subtree and store it in the vector data.mass.
    """
def computeSupportedForceByFrame(*args, **kwargs):
    """
    
    computeSupportedForceByFrame( (Model)model, (Data)data, (int)frame_id) -> Force :
        Computes the supported force of the frame (given by frame_id) and returns it.
        The supported force corresponds to the sum of all the forces experienced after the given frame.
        You must first call pinocchio::rnea to update placement values in data structure.
    """
def computeSupportedInertiaByFrame(*args, **kwargs):
    """
    
    computeSupportedInertiaByFrame( (Model)model, (Data)data, (int)frame_id, (bool)with_subtree) -> Inertia :
        Computes the supported inertia by the frame (given by frame_id) and returns it.
        The supported inertia corresponds to the sum of the inertias of all the child frames (that belongs to the same joint body) and the child joints, if with_subtree=True.
        You must first call pinocchio::forwardKinematics to update placement values in data structure.
    """
def computeTotalMass(*args, **kwargs):
    """
    
    computeTotalMass( (Model)model) -> float :
        Compute the total mass of the model and return it.
    
    computeTotalMass( (Model)model, (Data)data) -> float :
        Compute the total mass of the model, put it in data.mass[0] and return it.
    """
def crba(*args, **kwargs):
    """
    
    crba( (Model)model, (Data)data, (numpy.ndarray)q) -> numpy.ndarray :
        Computes CRBA, store the result in Data and return it.
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    """
def dDifference(*args, **kwargs):
    """
    
    dDifference( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2) -> tuple :
        Computes the partial derivatives of the difference function with respect to the first and the second argument, and returns the two Jacobians as a tuple.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        
    
    dDifference( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2, (ArgumentPosition)argument_position) -> numpy.ndarray :
        Computes the partial derivatives of the difference function with respect to the first (arg == ARG0) or the second argument (arg == ARG1).
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        	argument_position: either pinocchio.ArgumentPosition.ARG0 or pinocchio.ArgumentPosition.ARG1, depending on the desired Jacobian value.
        
    """
def dIntegrate(*args, **kwargs):
    """
    
    dIntegrate( (Model)model, (numpy.ndarray)q, (numpy.ndarray)v) -> tuple :
        Computes the partial derivatives of the integrate function with respect to the first and the second argument, and returns the two Jacobians as a tuple.
        
        Parameters:
        	model: model of the kinematic tree
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    
    dIntegrate( (Model)model, (numpy.ndarray)q, (numpy.ndarray)v, (ArgumentPosition)argument_position) -> numpy.ndarray :
        Computes the partial derivatives of the integrate function with respect to the first (arg == ARG0) or the second argument (arg == ARG1).
        
        Parameters:
        	model: model of the kinematic tree
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	argument_position: either pinocchio.ArgumentPosition.ARG0 or pinocchio.ArgumentPosition.ARG1, depending on the desired Jacobian value.
        
    """
def dIntegrateTransport(*args, **kwargs):
    """
    
    dIntegrateTransport( (Model)model, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)Jin, (ArgumentPosition)argument_position) -> numpy.ndarray :
        Takes a matrix expressed at q (+) v and uses parallel transport to express it in the tangent space at q.	This operation does the product of the matrix by the Jacobian of the integration operation, but more efficiently.Parameters:
        	model: model of the kinematic tree
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	Jin: the input matrix (row size model.nv)	argument_position: either pinocchio.ArgumentPosition.ARG0 (q) or pinocchio.ArgumentPosition.ARG1 (v), depending on the desired Jacobian value.
        
    """
def dccrba(*args, **kwargs):
    """
    
    dccrba( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the time derivative of the centroidal momentum matrix Ag in terms of q and v.
        For the same price, it also computes the centroidal momentum matrix (data.Ag), the total joint jacobians (data.J) and the related joint jacobians time derivative (data.dJ)
    """
def difference(*args, **kwargs):
    """
    
    difference( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2) -> numpy.ndarray :
        Difference between two joint configuration vectors, i.e. the tangent vector that must be integrated during one unit timeto go from q1 to q2.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        
    """
def distance(*args, **kwargs):
    """
    
    distance( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2) -> float :
        Distance between two joint configuration vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        
    """
def exp3(*args, **kwargs):
    """
    
    exp3( (numpy.ndarray)v) -> numpy.ndarray :
        Exp: so3 -> SO3. Return the integral of the input angular velocity during time 1.
    """
def exp6(*args, **kwargs):
    """
    
    exp6( (Motion)v) -> SE3 :
        Exp: se3 -> SE3. Return the integral of the input spatial velocity during time 1.
    
    exp6( (numpy.ndarray)v) -> SE3 :
        Exp: se3 -> SE3. Return the integral of the input spatial velocity during time 1.
    """
def forwardDynamics(*args, **kwargs):
    """
    
    forwardDynamics( (Model)Model, (Data)Data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)tau, (numpy.ndarray)J, (numpy.ndarray)gamma [, (float)damping]) -> numpy.ndarray :
        Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c. Internally, pinocchio.computeAllTerms is called.
    
    forwardDynamics( (Model)Model, (Data)Data, (numpy.ndarray)tau, (numpy.ndarray)J, (numpy.ndarray)gamma [, (float)damping]) -> numpy.ndarray :
        Solves the forward dynamics problem with contacts, puts the result in Data::ddq and return it. The contact forces are stored in data.lambda_c. Assumes pinocchio.computeAllTerms has been called.
    """
def forwardKinematics(*args, **kwargs):
    """
    
    forwardKinematics( (Model)model, (Data)data, (numpy.ndarray)q) -> None :
        Compute the global placements of all the joints of the kinematic tree and store the results in data.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        
    
    forwardKinematics( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> None :
        Compute the global placements and local spatial velocities of all the joints of the kinematic tree and store the results in data.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    
    forwardKinematics( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> None :
        Compute the global placements, local spatial velocities and spatial accelerations of all the joints of the kinematic tree and store the results in data.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
    """
def frameBodyRegressor(*args, **kwargs):
    """
    
    frameBodyRegressor( (Model)model, (Data)data, (int)frame_id) -> numpy.ndarray :
        Computes the regressor for the dynamic parameters of a rigid body attached to a given frame.
        This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	frame_id: index of the frame
        
    """
def frameJacobianTimeVariation(*args, **kwargs):
    """
    
    frameJacobianTimeVariation( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the Jacobian Time Variation of the frame given by its frame_id either in the reference frame provided by reference_frame.
        
    """
def framesForwardKinematics(*args, **kwargs):
    """
    
    framesForwardKinematics( (Model)model, (Data)data, (numpy.ndarray)q) -> None :
        Calls first the forwardKinematics(model,data,q) and then update the Frame placement quantities (data.oMf).
    """
def getAcceleration(*args, **kwargs):
    """
    
    getAcceleration( (Model)model, (Data)data, (int)joint_id [, (ReferenceFrame)reference_frame]) -> Motion :
        Returns the spatial acceleration of the joint expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a .
    """
def getCenterOfMassVelocityDerivatives(*args, **kwargs):
    """
    
    getCenterOfMassVelocityDerivatives( (Model)model, (Data)data) -> numpy.ndarray :
        Computes the partial derivaties of the center of mass velocity with respect to
        the joint configuration.
        You must first call computeAllTerms(model,data,q,v) or centerOfMass(model,data,q,v) before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        
    """
def getCentroidalDynamicsDerivatives(*args, **kwargs):
    """
    
    getCentroidalDynamicsDerivatives( (Model)Model, (Data)Data) -> tuple :
        Retrive the analytical derivatives of the centroidal dynamics
        from the RNEA derivatives.
        pinocchio.computeRNEADerivatives should have been called first.
    """
def getClassicalAcceleration(*args, **kwargs):
    """
    
    getClassicalAcceleration( (Model)model, (Data)data, (int)joint_id [, (ReferenceFrame)reference_frame]) -> Motion :
        Returns the "classical" acceleration of the joint expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a .
    """
def getCoriolisMatrix(*args, **kwargs):
    """
    
    getCoriolisMatrix( (Model)model, (Data)data) -> numpy.ndarray :
        Retrives the Coriolis Matrix C(q,v) of the Lagrangian dynamics after calling one of the derivative algorithms, store the result in data.C and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        
    """
def getFrameAcceleration(*args, **kwargs):
    """
    
    getFrameAcceleration( (Model)model, (Data)data, (int)frame_id [, (ReferenceFrame)reference_frame]) -> Motion :
        Returns the spatial acceleration of the frame expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a .
    """
def getFrameAccelerationDerivatives(*args, **kwargs):
    """
    
    getFrameAccelerationDerivatives( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the spatial acceleration of a given frame with respect to
        the joint configuration, velocity and acceleration and returns them as a tuple.
        The Jacobians can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	frame_id: index of the frame
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """
def getFrameClassicalAcceleration(*args, **kwargs):
    """
    
    getFrameClassicalAcceleration( (Model)model, (Data)data, (int)frame_id [, (ReferenceFrame)reference_frame]) -> Motion :
        Returns the "classical" acceleration of the frame expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v,a) should be called first to compute the joint spatial acceleration stored in data.a .
    """
def getFrameJacobian(*args, **kwargs):
    """
    
    getFrameJacobian( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the Jacobian of the frame given by its ID either in the local or the world frames.
        The columns of the Jacobian are expressed in the LOCAL frame coordinates system.
        In other words, the velocity of the frame vF expressed in the local coordinate is given by J*v,where v is the joint velocity.
        computeJointJacobians(model,data,q) and updateFramePlacements(model,data) must have been called first.
    """
def getFrameJacobianTimeVariation(*args, **kwargs):
    """
    
    getFrameJacobianTimeVariation( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Returns the Jacobian time variation of the frame given by its frame_id either in the reference frame provided by reference_frame.
        You have to call computeJointJacobiansTimeVariation(model,data,q,v) and updateFramePlacements(model,data) first.
    """
def getFrameVelocity(*args, **kwargs):
    """
    
    getFrameVelocity( (Model)model, (Data)data, (int)frame_id [, (ReferenceFrame)reference_frame]) -> Motion :
        Returns the spatial velocity of the frame expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint spatial velocity stored in data.v
    """
def getFrameVelocityDerivatives(*args, **kwargs):
    """
    
    getFrameVelocityDerivatives( (Model)model, (Data)data, (int)frame_id, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the spatial velocity of a given frame with respect to
        the joint configuration and velocity and returns them as a tuple.
        The Jacobians can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	frame_id: index of the frame
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """
def getJacobianSubtreeCenterOfMass(*args, **kwargs):
    """
    
    getJacobianSubtreeCenterOfMass( (Model)model, (Data)data, (int)subtree_root_joint_id) -> numpy.ndarray :
        Get the Jacobian of the CoM of the given subtree expressed in the world frame, according to the given entries in data. It assumes that jacobianCenterOfMass has been called first.
    """
def getJointAccelerationDerivatives(*args, **kwargs):
    """
    
    getJointAccelerationDerivatives( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the spatial acceleration of a given joint with respect to
        the joint configuration, velocity and acceleration and returns them as a tuple.
        The Jacobians can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """
def getJointJacobian(*args, **kwargs):
    """
    
    getJointJacobian( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the jacobian of a given given joint according to the given entries in data.
        If reference_frame is set to LOCAL, it returns the Jacobian expressed in the local coordinate system of the joint.
        If reference_frame is set to LOCAL_WORLD_ALIGNED, it returns the Jacobian expressed in the coordinate system of the frame centered on the joint, but aligned with the WORLD axes.
        If reference_frame is set to WORLD, it returns the Jacobian expressed in the coordinate system of the frame associated to the WORLD.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """
def getJointJacobianTimeVariation(*args, **kwargs):
    """
    
    getJointJacobianTimeVariation( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> numpy.ndarray :
        Computes the Jacobian time variation of a specific joint expressed in the requested frame provided by the value of reference_frame.You have to call computeJointJacobiansTimeVariation first. This function also computes the full model Jacobian contained in data.J.
        If reference_frame is set to LOCAL, it returns the Jacobian expressed in the local coordinate system of the joint.
        If reference_frame is set to LOCAL_WORLD_ALIGNED, it returns the Jacobian expressed in the coordinate system of the frame centered on the joint, but aligned with the WORLD axes.
        If reference_frame is set to WORLD, it returns the Jacobian expressed in the coordinate system of the frame associated to the WORLD.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """
def getJointVelocityDerivatives(*args, **kwargs):
    """
    
    getJointVelocityDerivatives( (Model)model, (Data)data, (int)joint_id, (ReferenceFrame)reference_frame) -> tuple :
        Computes the partial derivatives of the spatial velocity of a given joint with respect to
        the joint configuration and velocity and returns them as a tuple.
        The Jacobians can be either expressed in the LOCAL frame of the joint, in the LOCAL_WORLD_ALIGNED frame or in the WORLD coordinate frame depending on the value of reference_frame.
        You must first call computeForwardKinematicsDerivatives before calling this function.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        	reference_frame: reference frame in which the resulting derivatives are expressed
        
    """
def getKKTContactDynamicMatrixInverse(*args, **kwargs):
    """
    
    getKKTContactDynamicMatrixInverse( (Model)Model, (Data)Data, (numpy.ndarray)J) -> numpy.ndarray :
        Computes the inverse of the constraint matrix [[M JT], [J 0]]. forward/impulseDynamics must be called first. The jacobian should be the same that was provided to forward/impulseDynamics.
    """
def getVelocity(*args, **kwargs):
    """
    
    getVelocity( (Model)model, (Data)data, (int)joint_id [, (ReferenceFrame)reference_frame]) -> Motion :
        Returns the spatial velocity of the joint expressed in the coordinate system given by reference_frame.
        forwardKinematics(model,data,q,v[,a]) should be called first to compute the joint spatial velocity stored in data.v
    """
def impulseDynamics(*args, **kwargs):
    """
    
    impulseDynamics( (Model)Model, (Data)Data, (numpy.ndarray)q, (numpy.ndarray)v_before, (numpy.ndarray)J [, (float)r_coeff [, (float)damping]]) -> numpy.ndarray :
        Solves the impact dynamics problem with contacts, store the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c. Internally, pinocchio.crba is called.
    
    impulseDynamics( (Model)Model, (Data)Data, (numpy.ndarray)v_before, (numpy.ndarray)J [, (float)r_coeff [, (float)damping]]) -> numpy.ndarray :
        Solves the impact dynamics problem with contacts, store the result in Data::dq_after and return it. The contact impulses are stored in data.impulse_c. Assumes pinocchio.crba has been called.
    """
def integrate(*args, **kwargs):
    """
    
    integrate( (Model)model, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Integrate the joint configuration vector q with a tangent vector v during one unit time.
        This is the canonical integrator of a Configuration Space composed of Lie groups, such as most robots.
        
        Parameters:
        	model: model of the kinematic tree
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """
def interpolate(*args, **kwargs):
    """
    
    interpolate( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2, (float)alpha) -> numpy.ndarray :
        Interpolate between two given joint configuration vectors q1 and q2.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        	alpha: the interpolation coefficient in [0,1]
        
    """
def isNormalized(*args, **kwargs):
    """
    
    isNormalized( (Model)model, (numpy.ndarray)q [, (float)prec]) -> bool :
        Check whether a configuration vector is normalized within the given precision provided by prec.
        
        Parameters:
        	model: model of the kinematic tree
        	q: a joint configuration vector (size model.nq)
        	prec: requested accuracy for the check
        
    """
def isSameConfiguration(*args, **kwargs):
    """
    
    isSameConfiguration( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2, (float)prec) -> bool :
        Return true if two configurations are equivalent within the given precision provided by prec.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: a joint configuration vector (size model.nq)
        	q2: a joint configuration vector (size model.nq)
        	prec: requested accuracy for the comparison
        
    """
def jacobianCenterOfMass(*args, **kwargs):
    """
    
    jacobianCenterOfMass( (Model)model, (Data)data, (numpy.ndarray)q [, (bool)compute_subtree_coms]) -> numpy.ndarray :
        Computes the Jacobian of the center of mass, puts the result in Data and return it.
        If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    
    jacobianCenterOfMass( (Model)model, (Data)data [, (bool)compute_subtree_coms]) -> numpy.ndarray :
        Computes the Jacobian of the center of mass, puts the result in Data and return it.
        If compute_subtree_coms is True, the algorithm also computes the center of mass of the subtrees.
    """
def jacobianSubtreeCenterOfMass(*args, **kwargs):
    """
    
    jacobianSubtreeCenterOfMass( (Model)model, (Data)data, (numpy.ndarray)q, (int)subtree_root_joint_id) -> numpy.ndarray :
        Computes the Jacobian of the CoM of the given subtree (subtree_root_joint_id) expressed in the WORLD frame, according to the given joint configuration.
    
    jacobianSubtreeCenterOfMass( (Model)model, (Data)data, (int)subtree_root_joint_id) -> numpy.ndarray :
        Computes the Jacobian of the CoM of the given subtree (subtree_root_joint_id) expressed in the WORLD frame, according to the given entries in data.
    """
def jacobianSubtreeCoMJacobian(*args, **kwargs):
    """
    
    jacobianSubtreeCoMJacobian( (Model)model, (Data)data, (numpy.ndarray)q, (int)subtree_root_joint_id) -> numpy.ndarray :
        Computes the Jacobian of the CoM of the given subtree expressed in the world frame, according to the given joint configuration.
    
    jacobianSubtreeCoMJacobian( (Model)model, (Data)data, (int)subtree_root_joint_id) -> numpy.ndarray :
        Computes the Jacobian of the CoM of the given subtree expressed in the world frame, according to the given entries in data.
    """
def jointBodyRegressor(*args, **kwargs):
    """
    
    jointBodyRegressor( (Model)model, (Data)data, (int)joint_id) -> numpy.ndarray :
        Compute the regressor for the dynamic parameters of a rigid body attached to a given joint.
        This algorithm assumes RNEA has been run to compute the acceleration and gravitational effects.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	joint_id: index of the joint
        
    """
def loadReferenceConfigurations(*args, **kwargs):
    """
    
    loadReferenceConfigurations( (Model)model, (str)srdf_filename [, (bool)verbose]) -> None :
        Retrieve all the reference configurations of a given model from the SRDF file.
        Parameters:
        	model: model of the robot
        	srdf_filename: path to the SRDF file containing the reference configurations
        	verbose: [optional] display to the current terminal some internal information
    """
def loadReferenceConfigurationsFromXML(*args, **kwargs):
    """
    
    loadReferenceConfigurationsFromXML( (Model)model, (str)srdf_xml_stream [, (bool)verbose]) -> None :
        Retrieve all the reference configurations of a given model from the SRDF file.
        Parameters:
        	model: model of the robot
        	srdf_xml_stream: XML stream containing the SRDF information with the reference configurations
        	verbose: [optional] display to the current terminal some internal information
    """
def loadRotorParameters(*args, **kwargs):
    """
    
    loadRotorParameters( (Model)model, (str)srdf_filename [, (bool)verbose]) -> bool :
        Load the rotor parameters of a given model from a SRDF file.
        Results are stored in model.rotorInertia and model.rotorGearRatio.Parameters:
        	model: model of the robot
        	srdf_filename: path to the SRDF file containing the rotor parameters
        	verbose: [optional] display to the current terminal some internal information
    """
def log3(*args, **kwargs):
    """
    
    log3( (numpy.ndarray)R) -> numpy.ndarray :
        Log: SO3 -> so3. Pseudo-inverse of log from SO3 -> { v in so3, ||v|| < 2pi }.Exp: so3 -> SO3.
    """
def log6(*args, **kwargs):
    """
    
    log6( (SE3)M) -> Motion :
        Log: SE3 -> se3. Pseudo-inverse of exp from SE3 -> { v,w in se3, ||w|| < 2pi }.
    
    log6( (numpy.ndarray)H) -> Motion :
        Log: SE3 -> se3. Pseudo-inverse of exp from SE3 -> { v,w in se3, ||w|| < 2pi }.
    """
def neutral(*args, **kwargs):
    """
    
    neutral( (Model)model) -> numpy.ndarray :
        Returns the neutral configuration vector associated to the model.
        
        Parameters:
        	model: model of the kinematic tree
        
    """
def nonLinearEffects(*args, **kwargs):
    """
    
    nonLinearEffects( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v) -> numpy.ndarray :
        Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        
    """
def normalize(*args, **kwargs):
    """
    
    normalize( (Model)model, (numpy.ndarray)q) -> numpy.ndarray :
        Returns the configuration normalized.
        For instance, when the configuration vectors contains some quaternion values, it must be required to renormalize these components to keep orthonormal rotation values.
        
        Parameters:
        	model: model of the kinematic tree
        	q: a joint configuration vector to normalize (size model.nq)
        
    """
def omp_get_max_threads(*args, **kwargs):
    """
    
    omp_get_max_threads() -> int :
        Returns an upper bound on the number of threads that could be used.
    """
def printVersion(*args, **kwargs):
    """
    
    printVersion([  (str)delimiter]) -> str :
        Returns the current version of Pinocchio as a string.
        The user may specify the delimiter between the different semantic numbers.
    """
def randomConfiguration(*args, **kwargs):
    """
    
    randomConfiguration( (Model)model) -> numpy.ndarray :
        Generate a random configuration in the bounds given by the lower and upper limits contained in model.
        
        Parameters:
        	model: model of the kinematic tree
        
    
    randomConfiguration( (Model)model, (numpy.ndarray)lower_bound, (numpy.ndarray)upper_bound) -> numpy.ndarray :
        Generate a random configuration in the bounds given by the Joint lower and upper limits arguments.
        
        Parameters:
        	model: model of the kinematic tree
        	lower_bound: the lower bound on the joint configuration vectors (size model.nq)
        	upper_bound: the upper bound on the joint configuration vectors (size model.nq)
        
    """
def removeCollisionPairs(*args, **kwargs):
    """
    
    removeCollisionPairs( (Model)model, (GeometryModel)geom_model, (str)srdf_filename [, (bool)verbose]) -> None :
        Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.
        Parameters:
        Parameters:
        	model: model of the robot
        	geom_model: geometry model of the robot
        	srdf_filename: path to the SRDF file containing the collision pairs to remove
        	verbose: [optional] display to the current terminal some internal information
    """
def removeCollisionPairsFromXML(*args, **kwargs):
    """
    
    removeCollisionPairsFromXML( (Model)model, (GeometryModel)geom_model, (str)srdf_xml_stream [, (bool)verbose]) -> None :
        Parse an SRDF file in order to remove some collision pairs for a specific GeometryModel.
        Parameters:
        Parameters:
        	model: model of the robot
        	geom_model: geometry model of the robot
        	srdf_xml_stream: XML stream containing the SRDF information with the collision pairs to remove
        	verbose: [optional] display to the current terminal some internal information
    """
def rnea(*args, **kwargs):
    """
    
    rnea( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> numpy.ndarray :
        Compute the RNEA, store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        
    
    rnea( (Model)model, (Data)data, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a, (StdVec_Force)fext) -> numpy.ndarray :
        Compute the RNEA with external forces, store the result in Data and return it.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        	q: the joint configuration vector (size model.nq)
        	v: the joint velocity vector (size model.nv)
        	a: the joint acceleration vector (size model.nv)
        	fext: list of external forces expressed in the local frame of the joints (size model.njoints)
        
    
    rnea( (int)num_thread, (ModelPool)pool, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a) -> numpy.ndarray :
        Computes in parallel the RNEA and returns the result.
        
        Parameters:
        	num_thread: number of threads used for the computation
        	pool: pool of model/data
        	q: the joint configuration vector (size model.nq x batch_size)
        	v: the joint velocity vector (size model.nv x batch_size)
        	a: the joint acceleration vector (size model.nv x batch_size)
        
    
    rnea( (int)num_thread, (ModelPool)pool, (numpy.ndarray)q, (numpy.ndarray)v, (numpy.ndarray)a, (numpy.ndarray)tau) -> None :
        Computes in parallel the RNEA and stores the result in tau.
        
        Parameters:
        	num_thread: number of threads used for the computation
        	pool: pool of model/data
        	q: the joint configuration vector (size model.nq x batch_size)
        	v: the joint velocity vector (size model.nv x batch_size)
        	a: the joint acceleration vector (size model.nv x batch_size)
        	tau: the resulting joint torque vectors (size model.nv x batch_size)
        
    """
def seed(*args, **kwargs):
    """
    
    seed( (int)seed_value) -> None :
        Initialize the pseudo-random number generator with the argument seed_value.
    """
def sharedMemory(*args, **kwargs):
    """
    
    sharedMemory( (bool)value) -> None :
        Share the memory when converting from Eigen to Numpy.
    
    sharedMemory() -> bool :
        Status of the shared memory when converting from Eigen to Numpy.
        If True, the memory is shared when converting an Eigen::Matrix to a numpy.array.
        Otherwise, a deep copy of the Eigen::Matrix is performed.
    """
def skew(*args, **kwargs):
    """
    
    skew( (numpy.ndarray)u) -> numpy.ndarray :
        Computes the skew representation of a given 3d vector, i.e. the antisymmetric matrix representation of the cross product operator, aka U = [u]x.
        Parameters:
        	u: the input vector of dimension 3
    """
def skewSquare(*args, **kwargs):
    """
    
    skewSquare( (numpy.ndarray)u, (numpy.ndarray)v) -> numpy.ndarray :
        Computes the skew square representation of two given 3d vectors, i.e. the antisymmetric matrix representation of the chained cross product operator, u x (v x w), where w is another 3d vector.
        Parameters:
        	u: the first input vector of dimension 3
        	v: the second input vector of dimension 3
    """
def squaredDistance(*args, **kwargs):
    """
    
    squaredDistance( (Model)model, (numpy.ndarray)q1, (numpy.ndarray)q2) -> numpy.ndarray :
        Squared distance vector between two joint configuration vectors.
        
        Parameters:
        	model: model of the kinematic tree
        	q1: the initial joint configuration vector (size model.nq)
        	q2: the terminal joint configuration vector (size model.nq)
        
    """
def unSkew(*args, **kwargs):
    """
    
    unSkew( (numpy.ndarray)U) -> numpy.ndarray :
        Inverse of skew operator. From a given skew symmetric matrix U (i.e U = -U.T)of dimension 3x3, it extracts the supporting vector, i.e. the entries of U.
        Mathematically speacking, it computes v such that U.dot(x) = cross(u, x).
        Parameters:
        	U: the input skew symmetric matrix of dimension 3x3.
    """
def updateFramePlacement(*args, **kwargs):
    """
    
    updateFramePlacement( (Model)model, (Data)data, (int)frame_id) -> SE3 :
        Computes the placement of the given operational frame (frame_id) according to the current joint placement stored in data, stores the results in data and returns it.
    """
def updateFramePlacements(*args, **kwargs):
    """
    
    updateFramePlacements( (Model)model, (Data)data) -> None :
        Computes the placements of all the operational frames according to the current joint placement stored in dataand puts the results in data.
    """
def updateGeometryPlacements(*args, **kwargs):
    """
    
    updateGeometryPlacements( (Model)model, (Data)data, (GeometryModel)geometry_model, (GeometryData)geometry_data, (numpy.ndarray)q) -> None :
        Update the placement of the collision objects according to the current configuration.
        The algorithm also updates the current placement of the joint in Data.
    
    updateGeometryPlacements( (Model)model, (Data)data, (GeometryModel)geometry_model, (GeometryData)geometry_data) -> None :
        Update the placement of the collision objects according to the current joint placement stored in data.
    """
def updateGlobalPlacements(*args, **kwargs):
    """
    
    updateGlobalPlacements( (Model)model, (Data)data) -> None :
        Updates the global placements of all joint frames of the kinematic tree and store the results in data according to the relative placements of the joints.
        
        Parameters:
        	model: model of the kinematic tree
        	data: data related to the model
        
    """
ACCELERATION: KinematicLevel  # value = pinocchio.pinocchio_pywrap.KinematicLevel.ACCELERATION
ARG0: ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG0
ARG1: ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG1
ARG2: ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG2
ARG3: ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG3
ARG4: ArgumentPosition  # value = pinocchio.pinocchio_pywrap.ArgumentPosition.ARG4
BODY: FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.BODY
COLLISION: GeometryType  # value = pinocchio.pinocchio_pywrap.GeometryType.COLLISION
FIXED_JOINT: FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.FIXED_JOINT
JOINT: FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.JOINT
LOCAL: ReferenceFrame  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL
LOCAL_WORLD_ALIGNED: ReferenceFrame  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.LOCAL_WORLD_ALIGNED
OP_FRAME: FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.OP_FRAME
PINOCCHIO_MAJOR_VERSION: int = 2
PINOCCHIO_MINOR_VERSION: int = 7
PINOCCHIO_PATCH_VERSION: int = 0
POSITION: KinematicLevel  # value = pinocchio.pinocchio_pywrap.KinematicLevel.POSITION
SENSOR: FrameType  # value = pinocchio.pinocchio_pywrap.FrameType.SENSOR
VELOCITY: KinematicLevel  # value = pinocchio.pinocchio_pywrap.KinematicLevel.VELOCITY
VISUAL: GeometryType  # value = pinocchio.pinocchio_pywrap.GeometryType.VISUAL
WITH_CPPAD: bool = False
WITH_HPP_FCL: bool = True
WITH_OPENMP: bool = True
WITH_URDFDOM: bool = True
WORLD: ReferenceFrame  # value = pinocchio.pinocchio_pywrap.ReferenceFrame.WORLD
__raw_version__: str = '2.7.0'
__version__: str = '2.7.0'
