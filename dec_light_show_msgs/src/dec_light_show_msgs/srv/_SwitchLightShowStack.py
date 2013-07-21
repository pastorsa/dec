"""autogenerated by genpy from dec_light_show_msgs/SwitchLightShowStackRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SwitchLightShowStackRequest(genpy.Message):
  _md5sum = "14f721af85f8ae76d79d8ffbcdb621c1"
  _type = "dec_light_show_msgs/SwitchLightShowStackRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string[] light_show_stacks

"""
  __slots__ = ['light_show_stacks']
  _slot_types = ['string[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       light_show_stacks

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SwitchLightShowStackRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.light_show_stacks is None:
        self.light_show_stacks = []
    else:
      self.light_show_stacks = []

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
      length = len(self.light_show_stacks)
      buff.write(_struct_I.pack(length))
      for val1 in self.light_show_stacks:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.light_show_stacks = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.light_show_stacks.append(val1)
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
      length = len(self.light_show_stacks)
      buff.write(_struct_I.pack(length))
      for val1 in self.light_show_stacks:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.light_show_stacks = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.light_show_stacks.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
"""autogenerated by genpy from dec_light_show_msgs/SwitchLightShowStackResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SwitchLightShowStackResponse(genpy.Message):
  _md5sum = "9089a629b78f6579ec632f786ee54188"
  _type = "dec_light_show_msgs/SwitchLightShowStackResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string[] light_show_stacks
int8 result
int8 SUCCEEDED = 1
int8 FAILED = 0

"""
  # Pseudo-constants
  SUCCEEDED = 1
  FAILED = 0

  __slots__ = ['light_show_stacks','result']
  _slot_types = ['string[]','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       light_show_stacks,result

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SwitchLightShowStackResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.light_show_stacks is None:
        self.light_show_stacks = []
      if self.result is None:
        self.result = 0
    else:
      self.light_show_stacks = []
      self.result = 0

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
      length = len(self.light_show_stacks)
      buff.write(_struct_I.pack(length))
      for val1 in self.light_show_stacks:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      buff.write(_struct_b.pack(self.result))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.light_show_stacks = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.light_show_stacks.append(val1)
      start = end
      end += 1
      (self.result,) = _struct_b.unpack(str[start:end])
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
      length = len(self.light_show_stacks)
      buff.write(_struct_I.pack(length))
      for val1 in self.light_show_stacks:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      buff.write(_struct_b.pack(self.result))
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.light_show_stacks = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.light_show_stacks.append(val1)
      start = end
      end += 1
      (self.result,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b = struct.Struct("<b")
class SwitchLightShowStack(object):
  _type          = 'dec_light_show_msgs/SwitchLightShowStack'
  _md5sum = 'c1fb1d6078c03de3fa5a1bd5337c007b'
  _request_class  = SwitchLightShowStackRequest
  _response_class = SwitchLightShowStackResponse