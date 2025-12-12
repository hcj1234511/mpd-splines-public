from __future__ import annotations
import functools as functools
import warnings as warnings
__all__: list[str] = ['DeprecatedWarning', 'deprecated', 'functools', 'warnings']
class DeprecatedWarning(UserWarning):
    pass
def deprecated(instructions):
    """
    Flags a method as deprecated.
        Args:
            instructions: A human-friendly string of instructions, such
                as: 'Please migrate to add_proxy() ASAP.'
        
    """
