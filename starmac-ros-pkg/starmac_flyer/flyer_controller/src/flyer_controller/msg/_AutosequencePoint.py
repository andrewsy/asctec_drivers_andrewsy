"""autogenerated by genmsg_py from AutosequencePoint.msg. Do not edit."""
import roslib.message
import struct

import flyer_controller.msg

class AutosequencePoint(roslib.message.Message):
  _md5sum = "c397fd5685c6ff6ca05d78fa239709a5"
  _type = "flyer_controller/AutosequencePoint"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """flyer_controller/HoverPoint hover_point
bool pause
================================================================================
MSG: flyer_controller/HoverPoint
string name
float64 x # [m] (North)
float64 y # [m] (East)
float64 alt # [m]
float64 yaw # [deg]
float64 vx # [m/s]
float64 vy # [m/s]
"""
  __slots__ = ['hover_point','pause']
  _slot_types = ['flyer_controller/HoverPoint','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       hover_point,pause
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(AutosequencePoint, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.hover_point is None:
        self.hover_point = flyer_controller.msg.HoverPoint()
      if self.pause is None:
        self.pause = False
    else:
      self.hover_point = flyer_controller.msg.HoverPoint()
      self.pause = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self.hover_point.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_6dB.pack(_x.hover_point.x, _x.hover_point.y, _x.hover_point.alt, _x.hover_point.yaw, _x.hover_point.vx, _x.hover_point.vy, _x.pause))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.hover_point is None:
        self.hover_point = flyer_controller.msg.HoverPoint()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.hover_point.name = str[start:end]
      _x = self
      start = end
      end += 49
      (_x.hover_point.x, _x.hover_point.y, _x.hover_point.alt, _x.hover_point.yaw, _x.hover_point.vx, _x.hover_point.vy, _x.pause,) = _struct_6dB.unpack(str[start:end])
      self.pause = bool(self.pause)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self.hover_point.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_6dB.pack(_x.hover_point.x, _x.hover_point.y, _x.hover_point.alt, _x.hover_point.yaw, _x.hover_point.vx, _x.hover_point.vy, _x.pause))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.hover_point is None:
        self.hover_point = flyer_controller.msg.HoverPoint()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.hover_point.name = str[start:end]
      _x = self
      start = end
      end += 49
      (_x.hover_point.x, _x.hover_point.y, _x.hover_point.alt, _x.hover_point.yaw, _x.hover_point.vx, _x.hover_point.vy, _x.pause,) = _struct_6dB.unpack(str[start:end])
      self.pause = bool(self.pause)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_6dB = struct.Struct("<6dB")
