"""autogenerated by genmsg_py from controller_status.msg. Do not edit."""
import roslib.message
import struct

import std_msgs.msg

class controller_status(roslib.message.Message):
  _md5sum = "c2d88358f45aae41a821a82436eef1ed"
  _type = "flyer_controller/controller_status"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """int8 STATE_ERROR=0
int8 STATE_OFF=1
int8 STATE_INITIALIZING=2
int8 STATE_STANDBY=3
int8 STATE_OPERATIONAL=4

Header header
int8 state
string info
string active_mode
string[] standby_modes
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
  # Pseudo-constants
  STATE_ERROR = 0
  STATE_OFF = 1
  STATE_INITIALIZING = 2
  STATE_STANDBY = 3
  STATE_OPERATIONAL = 4

  __slots__ = ['header','state','info','active_mode','standby_modes']
  _slot_types = ['Header','int8','string','string','string[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       header,state,info,active_mode,standby_modes
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(controller_status, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg._Header.Header()
      if self.state is None:
        self.state = 0
      if self.info is None:
        self.info = ''
      if self.active_mode is None:
        self.active_mode = ''
      if self.standby_modes is None:
        self.standby_modes = []
    else:
      self.header = std_msgs.msg._Header.Header()
      self.state = 0
      self.info = ''
      self.active_mode = ''
      self.standby_modes = []

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
      buff.write(_struct_b.pack(self.state))
      _x = self.info
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.active_mode
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.standby_modes)
      buff.write(_struct_I.pack(length))
      for val1 in self.standby_modes:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
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
      end += 1
      (self.state,) = _struct_b.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.info = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.active_mode = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.standby_modes = []
      for i in xrange(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.standby_modes.append(val1)
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
      buff.write(_struct_b.pack(self.state))
      _x = self.info
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.active_mode
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.standby_modes)
      buff.write(_struct_I.pack(length))
      for val1 in self.standby_modes:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
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
      end += 1
      (self.state,) = _struct_b.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.info = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.active_mode = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.standby_modes = []
      for i in xrange(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.standby_modes.append(val1)
      return self
    except struct.error, e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_3I = struct.Struct("<3I")
_struct_b = struct.Struct("<b")
