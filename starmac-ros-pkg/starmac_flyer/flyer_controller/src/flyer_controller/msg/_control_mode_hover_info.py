"""autogenerated by genmsg_py from control_mode_hover_info.msg. Do not edit."""
import roslib.message
import struct

import std_msgs.msg

class control_mode_hover_info(roslib.message.Message):
  _md5sum = "67aa3c03432b61b4cc2a5316a0d1458e"
  _type = "flyer_controller/control_mode_hover_info"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
string hover_point

float64 north_cmd
float64 east_cmd
float64 north_vel_cmd
float64 east_vel_cmd
float64 yaw_cmd

float64 alt_override

float64 north_err
float64 east_err
float64 north_vel_err
float64 east_vel_err
float64 yaw_err
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

"""
  __slots__ = ['header','hover_point','north_cmd','east_cmd','north_vel_cmd','east_vel_cmd','yaw_cmd','alt_override','north_err','east_err','north_vel_err','east_vel_err','yaw_err']
  _slot_types = ['Header','string','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,hover_point,north_cmd,east_cmd,north_vel_cmd,east_vel_cmd,yaw_cmd,alt_override,north_err,east_err,north_vel_err,east_vel_err,yaw_err
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(control_mode_hover_info, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.hover_point is None:
        self.hover_point = ''
      if self.north_cmd is None:
        self.north_cmd = 0.
      if self.east_cmd is None:
        self.east_cmd = 0.
      if self.north_vel_cmd is None:
        self.north_vel_cmd = 0.
      if self.east_vel_cmd is None:
        self.east_vel_cmd = 0.
      if self.yaw_cmd is None:
        self.yaw_cmd = 0.
      if self.alt_override is None:
        self.alt_override = 0.
      if self.north_err is None:
        self.north_err = 0.
      if self.east_err is None:
        self.east_err = 0.
      if self.north_vel_err is None:
        self.north_vel_err = 0.
      if self.east_vel_err is None:
        self.east_vel_err = 0.
      if self.yaw_err is None:
        self.yaw_err = 0.
    else:
      self.header = std_msgs.msg._Header.Header()
      self.hover_point = ''
      self.north_cmd = 0.
      self.east_cmd = 0.
      self.north_vel_cmd = 0.
      self.east_vel_cmd = 0.
      self.yaw_cmd = 0.
      self.alt_override = 0.
      self.north_err = 0.
      self.east_err = 0.
      self.north_vel_err = 0.
      self.east_vel_err = 0.
      self.yaw_err = 0.

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
      _x = self.hover_point
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_11d.pack(_x.north_cmd, _x.east_cmd, _x.north_vel_cmd, _x.east_vel_cmd, _x.yaw_cmd, _x.alt_override, _x.north_err, _x.east_err, _x.north_vel_err, _x.east_vel_err, _x.yaw_err))
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
      self.hover_point = str[start:end]
      _x = self
      start = end
      end += 88
      (_x.north_cmd, _x.east_cmd, _x.north_vel_cmd, _x.east_vel_cmd, _x.yaw_cmd, _x.alt_override, _x.north_err, _x.east_err, _x.north_vel_err, _x.east_vel_err, _x.yaw_err,) = _struct_11d.unpack(str[start:end])
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
      _x = self.hover_point
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_11d.pack(_x.north_cmd, _x.east_cmd, _x.north_vel_cmd, _x.east_vel_cmd, _x.yaw_cmd, _x.alt_override, _x.north_err, _x.east_err, _x.north_vel_err, _x.east_vel_err, _x.yaw_err))
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
      self.hover_point = str[start:end]
      _x = self
      start = end
      end += 88
      (_x.north_cmd, _x.east_cmd, _x.north_vel_cmd, _x.east_vel_cmd, _x.yaw_cmd, _x.alt_override, _x.north_err, _x.east_err, _x.north_vel_err, _x.east_vel_err, _x.yaw_err,) = _struct_11d.unpack(str[start:end])
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_11d = struct.Struct("<11d")
_struct_3I = struct.Struct("<3I")
