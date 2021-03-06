"""autogenerated by genmsg_py from Autosequence.msg. Do not edit."""
import roslib.message
import struct

import flyer_controller.msg

class Autosequence(roslib.message.Message):
  _md5sum = "2fc3d91e94190d44de5f18ae9fa72ffd"
  _type = "flyer_controller/Autosequence"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string name
uint32 num_points
flyer_controller/AutosequencePoint[] points
================================================================================
MSG: flyer_controller/AutosequencePoint
flyer_controller/HoverPoint hover_point
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
  __slots__ = ['name','num_points','points']
  _slot_types = ['string','uint32','flyer_controller/AutosequencePoint[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       name,num_points,points
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Autosequence, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.name is None:
        self.name = ''
      if self.num_points is None:
        self.num_points = 0
      if self.points is None:
        self.points = []
    else:
      self.name = ''
      self.num_points = 0
      self.points = []

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
      _x = self.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_I.pack(self.num_points))
      length = len(self.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.points:
        _v1 = val1.hover_point
        _x = _v1.name
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = _v1
        buff.write(_struct_6d.pack(_x.x, _x.y, _x.alt, _x.yaw, _x.vx, _x.vy))
        buff.write(_struct_B.pack(val1.pause))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.name = str[start:end]
      start = end
      end += 4
      (self.num_points,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.points = []
      for i in xrange(0, length):
        val1 = flyer_controller.msg.AutosequencePoint()
        _v2 = val1.hover_point
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        _v2.name = str[start:end]
        _x = _v2
        start = end
        end += 48
        (_x.x, _x.y, _x.alt, _x.yaw, _x.vx, _x.vy,) = _struct_6d.unpack(str[start:end])
        start = end
        end += 1
        (val1.pause,) = _struct_B.unpack(str[start:end])
        val1.pause = bool(val1.pause)
        self.points.append(val1)
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
      _x = self.name
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_I.pack(self.num_points))
      length = len(self.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.points:
        _v3 = val1.hover_point
        _x = _v3.name
        length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = _v3
        buff.write(_struct_6d.pack(_x.x, _x.y, _x.alt, _x.yaw, _x.vx, _x.vy))
        buff.write(_struct_B.pack(val1.pause))
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
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.name = str[start:end]
      start = end
      end += 4
      (self.num_points,) = _struct_I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.points = []
      for i in xrange(0, length):
        val1 = flyer_controller.msg.AutosequencePoint()
        _v4 = val1.hover_point
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        _v4.name = str[start:end]
        _x = _v4
        start = end
        end += 48
        (_x.x, _x.y, _x.alt, _x.yaw, _x.vx, _x.vy,) = _struct_6d.unpack(str[start:end])
        start = end
        end += 1
        (val1.pause,) = _struct_B.unpack(str[start:end])
        val1.pause = bool(val1.pause)
        self.points.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B = struct.Struct("<B")
_struct_6d = struct.Struct("<6d")
