"""autogenerated by genmsg_py from EulerAngles.msg. Do not edit."""
import roslib.message
import struct


class EulerAngles(roslib.message.Message):
  _md5sum = "b41bea25ef0825fa6d2799746a51460f"
  _type = "starmac_msgs/EulerAngles"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string sequence # as per transformations.py, e.g. 'rzyx' for standard yaw-pitch-roll
bool angles_in_degrees # true if ai, aj, ak are in degrees, otherwise they are in radians
# rotations about 1st, 2nd, 3rd axis:
float64 ai
float64 aj
float64 ak
"""
  __slots__ = ['sequence','angles_in_degrees','ai','aj','ak']
  _slot_types = ['string','bool','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       sequence,angles_in_degrees,ai,aj,ak
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(EulerAngles, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.sequence is None:
        self.sequence = ''
      if self.angles_in_degrees is None:
        self.angles_in_degrees = False
      if self.ai is None:
        self.ai = 0.
      if self.aj is None:
        self.aj = 0.
      if self.ak is None:
        self.ak = 0.
    else:
      self.sequence = ''
      self.angles_in_degrees = False
      self.ai = 0.
      self.aj = 0.
      self.ak = 0.

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
      _x = self.sequence
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B3d.pack(_x.angles_in_degrees, _x.ai, _x.aj, _x.ak))
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
      self.sequence = str[start:end]
      _x = self
      start = end
      end += 25
      (_x.angles_in_degrees, _x.ai, _x.aj, _x.ak,) = _struct_B3d.unpack(str[start:end])
      self.angles_in_degrees = bool(self.angles_in_degrees)
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
      _x = self.sequence
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B3d.pack(_x.angles_in_degrees, _x.ai, _x.aj, _x.ak))
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
      self.sequence = str[start:end]
      _x = self
      start = end
      end += 25
      (_x.angles_in_degrees, _x.ai, _x.aj, _x.ak,) = _struct_B3d.unpack(str[start:end])
      self.angles_in_degrees = bool(self.angles_in_degrees)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B3d = struct.Struct("<B3d")
