"""autogenerated by genmsg_py from EulerAnglesStamped.msg. Do not edit."""
import roslib.message
import struct

import starmac_msgs.msg
import std_msgs.msg

class EulerAnglesStamped(roslib.message.Message):
  _md5sum = "08ed1513cf62c3aa9d3f21c206745853"
  _type = "starmac_msgs/EulerAnglesStamped"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
EulerAngles angles
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: starmac_msgs/EulerAngles
string sequence # as per transformations.py, e.g. 'rzyx' for standard yaw-pitch-roll
bool angles_in_degrees # true if ai, aj, ak are in degrees, otherwise they are in radians
# rotations about 1st, 2nd, 3rd axis:
float64 ai
float64 aj
float64 ak
"""
  __slots__ = ['header','angles']
  _slot_types = ['Header','starmac_msgs/EulerAngles']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,angles
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(EulerAnglesStamped, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.angles is None:
        self.angles = starmac_msgs.msg.EulerAngles()
    else:
      self.header = std_msgs.msg._Header.Header()
      self.angles = starmac_msgs.msg.EulerAngles()

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
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.angles.sequence
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B3d.pack(_x.angles.angles_in_degrees, _x.angles.ai, _x.angles.aj, _x.angles.ak))
    except struct.error, se: self._check_types(se)
    except TypeError, te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.angles is None:
        self.angles = starmac_msgs.msg.EulerAngles()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.angles.sequence = str[start:end]
      _x = self
      start = end
      end += 25
      (_x.angles.angles_in_degrees, _x.angles.ai, _x.angles.aj, _x.angles.ak,) = _struct_B3d.unpack(str[start:end])
      self.angles.angles_in_degrees = bool(self.angles.angles_in_degrees)
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
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.angles.sequence
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_B3d.pack(_x.angles.angles_in_degrees, _x.angles.ai, _x.angles.aj, _x.angles.ak))
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
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.angles is None:
        self.angles = starmac_msgs.msg.EulerAngles()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.angles.sequence = str[start:end]
      _x = self
      start = end
      end += 25
      (_x.angles.angles_in_degrees, _x.angles.ai, _x.angles.aj, _x.angles.ak,) = _struct_B3d.unpack(str[start:end])
      self.angles.angles_in_degrees = bool(self.angles.angles_in_degrees)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_B3d = struct.Struct("<B3d")
_struct_3I = struct.Struct("<3I")
