import __future__
from __future__ import annotations
import numpy as np
from numpy import linalg as npl
from pinocchio.deprecation import deprecated
from pinocchio import pinocchio_pywrap as pin
import sys as sys
__all__: list = ['np', 'npl', 'eye', 'zero', 'rand', 'isapprox', 'mprint', 'skew', 'cross', 'npToTTuple', 'npToTuple', 'rotate', 'rpyToMatrix', 'matrixToRpy', 'se3ToXYZQUAT', 'XYZQUATToSe3', 'fromListToVectorOfString']
def XYZQUATToSe3(*args, **kwargs):
    """
    Deprecated: Now useless. You can directly have access to this function from the main scope of Pinocchio
    """
def cross(*args, **kwargs):
    """
    Deprecated: Please use numpy.cross(a, b) or numpy.cross(a, b, axis=0).
    """
def eye(n):
    ...
def fromListToVectorOfString(items):
    ...
def isapprox(a, b, epsilon = 1e-06):
    ...
def mprint(M, name = 'ans', eps = 1e-15):
    """
    
        Matlab-style pretty matrix print.
        
    """
def npToTTuple(M):
    ...
def npToTuple(M):
    ...
def rand(n):
    ...
def se3ToXYZQUAT(*args, **kwargs):
    """
    Deprecated: Now useless. You can directly have access to this function from the main scope of Pinocchio
    """
def skew(*args, **kwargs):
    """
    Deprecated: Now useless. You can directly have access to this function from the main scope of Pinocchio
    """
def zero(n):
    ...
division: __future__._Feature  # value = _Feature((2, 2, 0, 'alpha', 2), (3, 0, 0, 'alpha', 0), 131072)
print_function: __future__._Feature  # value = _Feature((2, 6, 0, 'alpha', 2), (3, 0, 0, 'alpha', 0), 1048576)
