"""autogenerated by genpy from sandbox_marc_darpa_m3/ServiceBasedMPCRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ServiceBasedMPCRequest(genpy.Message):
  _md5sum = "ff2d5410ce473a73156a5162cbdc78d0"
  _type = "sandbox_marc_darpa_m3/ServiceBasedMPCRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 alpha
float64 beta
float64 gamma
float64[] delta_x_d
float64[] J
float64[] desired_force_decrease
float64[] x_0
float64[] KP_t_KP
float64[] B
float64[] q_min
float64[] q_max
float64[] f_min
float64[] f_max
float64[] n_K_ci_J_ci
float64[] n_K_ci_J_ci_max
float64[] u_min
float64[] u_max
float64[] Q

"""
  __slots__ = ['alpha','beta','gamma','delta_x_d','J','desired_force_decrease','x_0','KP_t_KP','B','q_min','q_max','f_min','f_max','n_K_ci_J_ci','n_K_ci_J_ci_max','u_min','u_max','Q']
  _slot_types = ['float64','float64','float64','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       alpha,beta,gamma,delta_x_d,J,desired_force_decrease,x_0,KP_t_KP,B,q_min,q_max,f_min,f_max,n_K_ci_J_ci,n_K_ci_J_ci_max,u_min,u_max,Q

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ServiceBasedMPCRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
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
      if self.desired_force_decrease is None:
        self.desired_force_decrease = []
      if self.x_0 is None:
        self.x_0 = []
      if self.KP_t_KP is None:
        self.KP_t_KP = []
      if self.B is None:
        self.B = []
      if self.q_min is None:
        self.q_min = []
      if self.q_max is None:
        self.q_max = []
      if self.f_min is None:
        self.f_min = []
      if self.f_max is None:
        self.f_max = []
      if self.n_K_ci_J_ci is None:
        self.n_K_ci_J_ci = []
      if self.n_K_ci_J_ci_max is None:
        self.n_K_ci_J_ci_max = []
      if self.u_min is None:
        self.u_min = []
      if self.u_max is None:
        self.u_max = []
      if self.Q is None:
        self.Q = []
    else:
      self.alpha = 0.
      self.beta = 0.
      self.gamma = 0.
      self.delta_x_d = []
      self.J = []
      self.desired_force_decrease = []
      self.x_0 = []
      self.KP_t_KP = []
      self.B = []
      self.q_min = []
      self.q_max = []
      self.f_min = []
      self.f_max = []
      self.n_K_ci_J_ci = []
      self.n_K_ci_J_ci_max = []
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
      buff.write(_struct_3d.pack(_x.alpha, _x.beta, _x.gamma))
      length = len(self.delta_x_d)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.delta_x_d))
      length = len(self.J)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.J))
      length = len(self.desired_force_decrease)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.desired_force_decrease))
      length = len(self.x_0)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.x_0))
      length = len(self.KP_t_KP)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.KP_t_KP))
      length = len(self.B)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.B))
      length = len(self.q_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.q_min))
      length = len(self.q_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.q_max))
      length = len(self.f_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.f_min))
      length = len(self.f_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.f_max))
      length = len(self.n_K_ci_J_ci)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.n_K_ci_J_ci))
      length = len(self.n_K_ci_J_ci_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.n_K_ci_J_ci_max))
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
      end = 0
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
      self.desired_force_decrease = struct.unpack(pattern, str[start:end])
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
      self.B = struct.unpack(pattern, str[start:end])
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
      self.f_min = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.f_max = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.n_K_ci_J_ci = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.n_K_ci_J_ci_max = struct.unpack(pattern, str[start:end])
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
      buff.write(_struct_3d.pack(_x.alpha, _x.beta, _x.gamma))
      length = len(self.delta_x_d)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.delta_x_d.tostring())
      length = len(self.J)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.J.tostring())
      length = len(self.desired_force_decrease)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.desired_force_decrease.tostring())
      length = len(self.x_0)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.x_0.tostring())
      length = len(self.KP_t_KP)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.KP_t_KP.tostring())
      length = len(self.B)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.B.tostring())
      length = len(self.q_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.q_min.tostring())
      length = len(self.q_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.q_max.tostring())
      length = len(self.f_min)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.f_min.tostring())
      length = len(self.f_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.f_max.tostring())
      length = len(self.n_K_ci_J_ci)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.n_K_ci_J_ci.tostring())
      length = len(self.n_K_ci_J_ci_max)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.n_K_ci_J_ci_max.tostring())
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
      end = 0
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
      self.desired_force_decrease = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
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
      self.B = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
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
      self.f_min = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.f_max = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.n_K_ci_J_ci = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.n_K_ci_J_ci_max = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
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
_struct_3d = struct.Struct("<3d")
"""autogenerated by genpy from sandbox_marc_darpa_m3/ServiceBasedMPCResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ServiceBasedMPCResponse(genpy.Message):
  _md5sum = "11c19526c41aa75b8784d188a8c8af6b"
  _type = "sandbox_marc_darpa_m3/ServiceBasedMPCResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64[3] delta_phi_opt
float64[3] predicted_joint_angles






"""
  __slots__ = ['delta_phi_opt','predicted_joint_angles']
  _slot_types = ['float64[3]','float64[3]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       delta_phi_opt,predicted_joint_angles

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ServiceBasedMPCResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.delta_phi_opt is None:
        self.delta_phi_opt = [0.,0.,0.]
      if self.predicted_joint_angles is None:
        self.predicted_joint_angles = [0.,0.,0.]
    else:
      self.delta_phi_opt = [0.,0.,0.]
      self.predicted_joint_angles = [0.,0.,0.]

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
      buff.write(_struct_3d.pack(*self.delta_phi_opt))
      buff.write(_struct_3d.pack(*self.predicted_joint_angles))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 24
      self.delta_phi_opt = _struct_3d.unpack(str[start:end])
      start = end
      end += 24
      self.predicted_joint_angles = _struct_3d.unpack(str[start:end])
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
      buff.write(self.delta_phi_opt.tostring())
      buff.write(self.predicted_joint_angles.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 24
      self.delta_phi_opt = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 24
      self.predicted_joint_angles = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3d = struct.Struct("<3d")
class ServiceBasedMPC(object):
  _type          = 'sandbox_marc_darpa_m3/ServiceBasedMPC'
  _md5sum = '22c7980a02fcd5ae96fa9366f7c556ce'
  _request_class  = ServiceBasedMPCRequest
  _response_class = ServiceBasedMPCResponse
