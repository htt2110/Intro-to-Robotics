# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from state_estimator/SensorData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import state_estimator.msg
import std_msgs.msg

class SensorData(genpy.Message):
  _md5sum = "216a9c8ffac46cf3c39b26f16980385c"
  _type = "state_estimator/SensorData"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# Message header
std_msgs/Header header

# The forward translational velocity commanded to the robot
float64 vel_trans

# The rotational velocity commanded to the robot
float64 vel_ang

# The readings of landmarks with the range of the robot's sensors
# Can be empty, if no landmarks are nearby
state_estimator/LandmarkReading[] readings

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: state_estimator/LandmarkReading
# The landmark for which this reading was created
state_estimator/Landmark landmark

# The range from the robot's location to the landmark above
float64 range

# The bearing of this landmark relative to the robot's orientation
float64 bearing

================================================================================
MSG: state_estimator/Landmark
# The x coordinate of this landmark
float64 x

# The y coordinate of this landmark
float64 y

"""
  __slots__ = ['header','vel_trans','vel_ang','readings']
  _slot_types = ['std_msgs/Header','float64','float64','state_estimator/LandmarkReading[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,vel_trans,vel_ang,readings

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SensorData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.vel_trans is None:
        self.vel_trans = 0.
      if self.vel_ang is None:
        self.vel_ang = 0.
      if self.readings is None:
        self.readings = []
    else:
      self.header = std_msgs.msg.Header()
      self.vel_trans = 0.
      self.vel_ang = 0.
      self.readings = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2d().pack(_x.vel_trans, _x.vel_ang))
      length = len(self.readings)
      buff.write(_struct_I.pack(length))
      for val1 in self.readings:
        _v1 = val1.landmark
        _x = _v1
        buff.write(_get_struct_2d().pack(_x.x, _x.y))
        _x = val1
        buff.write(_get_struct_2d().pack(_x.range, _x.bearing))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.readings is None:
        self.readings = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.vel_trans, _x.vel_ang,) = _get_struct_2d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.readings = []
      for i in range(0, length):
        val1 = state_estimator.msg.LandmarkReading()
        _v2 = val1.landmark
        _x = _v2
        start = end
        end += 16
        (_x.x, _x.y,) = _get_struct_2d().unpack(str[start:end])
        _x = val1
        start = end
        end += 16
        (_x.range, _x.bearing,) = _get_struct_2d().unpack(str[start:end])
        self.readings.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2d().pack(_x.vel_trans, _x.vel_ang))
      length = len(self.readings)
      buff.write(_struct_I.pack(length))
      for val1 in self.readings:
        _v3 = val1.landmark
        _x = _v3
        buff.write(_get_struct_2d().pack(_x.x, _x.y))
        _x = val1
        buff.write(_get_struct_2d().pack(_x.range, _x.bearing))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.readings is None:
        self.readings = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 16
      (_x.vel_trans, _x.vel_ang,) = _get_struct_2d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.readings = []
      for i in range(0, length):
        val1 = state_estimator.msg.LandmarkReading()
        _v4 = val1.landmark
        _x = _v4
        start = end
        end += 16
        (_x.x, _x.y,) = _get_struct_2d().unpack(str[start:end])
        _x = val1
        start = end
        end += 16
        (_x.range, _x.bearing,) = _get_struct_2d().unpack(str[start:end])
        self.readings.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2d = None
def _get_struct_2d():
    global _struct_2d
    if _struct_2d is None:
        _struct_2d = struct.Struct("<2d")
    return _struct_2d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
