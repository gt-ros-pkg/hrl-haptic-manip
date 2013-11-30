"""autogenerated by genpy from hrl_haptic_manipulation_in_clutter_msgs/MPC_FormattedData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class MPC_FormattedData(genpy.Message):
  _md5sum = "35ec73a5af9869a7ed35470b92764e75"
  _type = "hrl_haptic_manipulation_in_clutter_msgs/MPC_FormattedData"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

float64 alpha
float64 beta
float64 gamma
float64[] delta_x_d
float64[] J
float64[] desired_dist_increase
float64[] x_0
float64[] KP_t_KP
float64[] q_min
float64[] q_max
float64[] dist_min
float64[] dist_max
float64[] n_J_ci
float64[] n_J_ci_max
float64[] u_min
float64[] u_max
float64[] Q





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
  __slots__ = ['header','alpha','beta','gamma','delta_x_d','J','desired_dist_increase','x_0','KP_t_KP','q_min','q_max','dist_min','dist_max','n_J_ci','n_J_ci_max','u_min','u_max','Q']
  _slot_types = ['std_msgs/Header','float64','float64','float64','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,alpha,beta,gamma,delta_x_d,J,desired_dist_increase,x_0,KP_t_KP,q_min,q_max,dist_min,dist_max,n_J_ci,n_J_ci_max,u_min,u_max,Q

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MPC_FormattedData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.alpha is None:
        self.alpha = 0.
      if self.beta is None:
        self.beta = 0.
      if self.gamma is None:
        self.gamma = 0.
      if self.delta_x_d is None:
        self.delta_x_d = []
      if self.J is None:
        self.J = []
      if self.desired_dist_increase is None:
        self.desired_dist_increase = []
      if self.x_0 is None:
        self.x_0 = []
      if self.KP_t_KP is None:
        self.KP_t_KP = []
      if self.q_min is None:
        self.q_min = []
      if self.q_max is None:
        self.q_max = []
      if self.dist_min is None:
        self.dist_min = []
      if self.dist_max is None:
        self.dist_max = []
      if self.n_J_ci is None:
        self.n_J_ci = []
      if self.n_J_ci_max is None:
        self.n_J_ci_max = []
      if self.u_min is None:
        self.u_min = []
      if self.u_max is None:
        self.u_max = []
      if self.Q is None:
        self.Q = []
    else:
      self.header = std_msgs.msg.Header()
      self.alpha = 0.
      self.beta = 0.
      self.gamma = 0.
      self.delta_x_d = []
      self.J = []
      self.desired_dist_increase = []
      self.x_0 = []
      self.KP_t_KP = []
      self.q_min = []
      self.q_max = []
      self.dist_min = []
      self.dist_max = []
      self.n_J_ci = []
      self.n_J_ci_max = []
      self.u_min = []
      self.u_max = []
      self.Q = []

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3d.pack(_x.alpha, _x.beta, _x.gamma))
      length = len(self.delta_x_d)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.delta_x_d))
      length = len(self.J)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.J))
      length = len(self.desired_dist_increase)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.desired_dist_increase))
      length = len(self.x_0)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.x_0))
      length = len(self.KP_t_KP)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.KP_t_KP))
      length = len(self.q_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.q_min))
      length = len(self.q_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.q_max))
      length = len(self.dist_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.dist_min))
      length = len(self.dist_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.dist_max))
      length = len(self.n_J_ci)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.n_J_ci))
      length = len(self.n_J_ci_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.n_J_ci_max))
      length = len(self.u_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.u_min))
      length = len(self.u_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.u_max))
      length = len(self.Q)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.Q))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
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
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.alpha, _x.beta, _x.gamma,) = _struct_3d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.delta_x_d = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.J = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.desired_dist_increase = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.x_0 = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.KP_t_KP = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.q_min = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.q_max = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.dist_min = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.dist_max = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.n_J_ci = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.n_J_ci_max = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.u_min = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.u_max = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.Q = struct.unpack(pattern, str[start:end])
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3d.pack(_x.alpha, _x.beta, _x.gamma))
      length = len(self.delta_x_d)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.delta_x_d.tostring())
      length = len(self.J)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.J.tostring())
      length = len(self.desired_dist_increase)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.desired_dist_increase.tostring())
      length = len(self.x_0)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.x_0.tostring())
      length = len(self.KP_t_KP)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.KP_t_KP.tostring())
      length = len(self.q_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.q_min.tostring())
      length = len(self.q_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.q_max.tostring())
      length = len(self.dist_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.dist_min.tostring())
      length = len(self.dist_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.dist_max.tostring())
      length = len(self.n_J_ci)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.n_J_ci.tostring())
      length = len(self.n_J_ci_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.n_J_ci_max.tostring())
      length = len(self.u_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.u_min.tostring())
      length = len(self.u_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.u_max.tostring())
      length = len(self.Q)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.Q.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
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
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.alpha, _x.beta, _x.gamma,) = _struct_3d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.delta_x_d = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.J = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.desired_dist_increase = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.x_0 = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.KP_t_KP = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.q_min = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.q_max = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.dist_min = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.dist_max = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.n_J_ci = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.n_J_ci_max = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.u_min = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.u_max = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.Q = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_3d = struct.Struct("<3d")
