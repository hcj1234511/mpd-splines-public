from __future__ import annotations
from pinocchio import pinocchio_pywrap as pin
__all__: list[str] = ['WITH_HPP_FCL', 'WITH_HPP_FCL_BINDINGS', 'buildModelsFromUrdf', 'createDatas', 'pin']
def buildModelsFromUrdf(filename, package_dirs = None, root_joint = None, verbose = False, meshLoader = None, geometry_types = ...):
    """
    Parse the URDF file given in input and return a Pinocchio Model followed by corresponding GeometryModels of types specified by geometry_types, in the same order as listed.
        Examples of usage:
            # load model, collision model, and visual model, in this order (default)
            model, collision_model, visual_model = buildModelsFromUrdf(filename[, ...], geometry_types=[pin.GeometryType.COLLISION,pin.GeometryType.VISUAL])
            model, collision_model, visual_model = buildModelsFromUrdf(filename[, ...]) # same as above
    
            model, collision_model = buildModelsFromUrdf(filename[, ...], geometry_types=[pin.GeometryType.COLLISION]) # only load the model and the collision model
            model, collision_model = buildModelsFromUrdf(filename[, ...], geometry_types=pin.GeometryType.COLLISION)   # same as above
            model, visual_model    = buildModelsFromUrdf(filename[, ...], geometry_types=pin.GeometryType.VISUAL)      # only load the model and the visual model
    
            model = buildModelsFromUrdf(filename[, ...], geometry_types=[])  # equivalent to buildModelFromUrdf(filename[, root_joint])
    
        Remark:
            Remark: In the URDF format, a joint of type fixed can be defined.
            For efficiency reasons, it is treated as operational frame and not as a joint of the model.
        
    """
def createDatas(*models):
    """
    Call createData() on each Model or GeometryModel in input and return the results in a tuple.
        If one of the models is None, the corresponding data object in the result is also None.
        
    """
WITH_HPP_FCL: bool = True
WITH_HPP_FCL_BINDINGS: bool = True
