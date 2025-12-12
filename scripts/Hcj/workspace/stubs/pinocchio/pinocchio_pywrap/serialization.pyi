from __future__ import annotations
import typing
__all__: list[str] = ['StaticBuffer', 'StreamBuffer', 'buffer_copy', 'loadFromBinary', 'saveToBinary']
class StaticBuffer(Boost.Python.instance):
    """
    Static buffer to save/load serialized objects in binary mode with pre-allocated memory.
    """
    __instance_size__: typing.ClassVar[int] = 56
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self, (int)size) -> None :
            Default constructor from a given size capacity.
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def reserve(*args, **kwargs):
        """
        
        reserve( (StaticBuffer)arg1, (int)new_size) -> None :
            Increase the capacity of the vector to a value that's greater or equal to new_size.
        """
    @staticmethod
    def size(*args, **kwargs):
        """
        
        size( (StaticBuffer)self) -> int :
            Get the size of the input sequence.
        """
class StreamBuffer(Boost.Python.instance):
    """
    Stream buffer to save/load serialized objects in binary mode.
    """
    __instance_size__: typing.ClassVar[int] = 120
    @staticmethod
    def __init__(*args, **kwargs):
        """
        
        __init__( (object)self) -> None :
            Default constructor.
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def max_size(*args, **kwargs):
        """
        
        max_size( (StreamBuffer)arg1) -> int :
            Get the maximum size of the StreamBuffer.
        """
    @staticmethod
    def prepare(*args, **kwargs):
        """
        
        prepare( (StreamBuffer)arg1, (int)arg2) -> StreamBuffer :
            Reserve data.
        """
    @staticmethod
    def size(*args, **kwargs):
        """
        
        size( (StreamBuffer)arg1) -> int :
            Get the size of the input sequence.
        """
def buffer_copy(*args, **kwargs):
    """
    
    buffer_copy( (StreamBuffer)dest, (StreamBuffer)source) -> None :
        Copy bytes from a source buffer to a target buffer.
    """
def loadFromBinary(*args, **kwargs):
    """
    
    loadFromBinary( (StdVec_SE3)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_SE3)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Force)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Force)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Motion)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Motion)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Inertia)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Inertia)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Index)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Index)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_IndexVector)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_IndexVector)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_StdString)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_StdString)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Bool)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Bool)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Double)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Double)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Model)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Model)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Frame)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Frame)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Data)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Data)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Vector3)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Vector3)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Matrix6x)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Matrix6x)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_Int)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_Int)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (StdVec_CollisionPair)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (StdVec_CollisionPair)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (GeometryData)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (GeometryData)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (TriangleP)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (TriangleP)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Sphere)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Sphere)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Capsule)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Capsule)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Box)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Box)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Cone)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Cone)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Cylinder)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Cylinder)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Plane)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Plane)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (Halfspace)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (Halfspace)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (BVHModelOBB)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (BVHModelOBB)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (object)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (object)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    
    loadFromBinary( (BVHModelOBBRSS)object, (StreamBuffer)stream_buffer) -> None :
        Load an object from a binary buffer.
    
    loadFromBinary( (BVHModelOBBRSS)object, (StaticBuffer)static_buffer) -> None :
        Load an object from a static binary buffer.
    """
def saveToBinary(*args, **kwargs):
    """
    
    saveToBinary( (StdVec_SE3)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_SE3)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Force)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Force)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Motion)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Motion)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Inertia)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Inertia)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Index)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Index)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_IndexVector)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_IndexVector)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_StdString)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_StdString)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Bool)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Bool)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Double)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Double)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Model)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Model)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Frame)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Frame)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Data)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Data)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Vector3)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Vector3)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Matrix6x)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Matrix6x)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_Int)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_Int)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (StdVec_CollisionPair)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (StdVec_CollisionPair)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (GeometryData)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (GeometryData)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (TriangleP)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (TriangleP)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Sphere)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Sphere)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Capsule)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Capsule)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Box)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Box)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Cone)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Cone)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Cylinder)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Cylinder)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Plane)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Plane)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (Halfspace)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (Halfspace)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (BVHModelOBB)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (BVHModelOBB)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (object)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (object)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    
    saveToBinary( (BVHModelOBBRSS)object, (StreamBuffer)stream_buffer) -> None :
        Save an object to a binary buffer.
    
    saveToBinary( (BVHModelOBBRSS)object, (StaticBuffer)static_buffer) -> None :
        Save an object to a static binary buffer.
    """
