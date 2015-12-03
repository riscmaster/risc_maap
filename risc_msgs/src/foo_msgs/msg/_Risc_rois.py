"""autogenerated by genpy from foo_msgs/Risc_rois.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import foo_msgs.msg

class Risc_rois(genpy.Message):
  _md5sum = "f66579dedd062ccc57f1aced22cbbd3a"
  _type = "foo_msgs/Risc_rois"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string name
Risc_roi[] landmarks
Risc_roi[] quads

================================================================================
MSG: foo_msgs/Risc_roi
string name

bool visible

int32 x

int32 y

float32 width

float32 height

float64 angle

"""
  __slots__ = ['name','landmarks','quads']
  _slot_types = ['string','foo_msgs/Risc_roi[]','foo_msgs/Risc_roi[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       name,landmarks,quads

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Risc_rois, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.name is None:
        self.name = ''
      if self.landmarks is None:
        self.landmarks = []
      if self.quads is None:
        self.quads = []
    else:
      self.name = ''
      self.landmarks = []
      self.quads = []

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
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.landmarks)
      buff.write(_struct_I.pack(length))
      for val1 in self.landmarks:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_B2i2fd.pack(_x.visible, _x.x, _x.y, _x.width, _x.height, _x.angle))
      length = len(self.quads)
      buff.write(_struct_I.pack(length))
      for val1 in self.quads:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_B2i2fd.pack(_x.visible, _x.x, _x.y, _x.width, _x.height, _x.angle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.landmarks is None:
        self.landmarks = None
      if self.quads is None:
        self.quads = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.landmarks = []
      for i in range(0, length):
        val1 = foo_msgs.msg.Risc_roi()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 25
        (_x.visible, _x.x, _x.y, _x.width, _x.height, _x.angle,) = _struct_B2i2fd.unpack(str[start:end])
        val1.visible = bool(val1.visible)
        self.landmarks.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.quads = []
      for i in range(0, length):
        val1 = foo_msgs.msg.Risc_roi()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 25
        (_x.visible, _x.x, _x.y, _x.width, _x.height, _x.angle,) = _struct_B2i2fd.unpack(str[start:end])
        val1.visible = bool(val1.visible)
        self.quads.append(val1)
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
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      length = len(self.landmarks)
      buff.write(_struct_I.pack(length))
      for val1 in self.landmarks:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_B2i2fd.pack(_x.visible, _x.x, _x.y, _x.width, _x.height, _x.angle))
      length = len(self.quads)
      buff.write(_struct_I.pack(length))
      for val1 in self.quads:
        _x = val1.name
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_B2i2fd.pack(_x.visible, _x.x, _x.y, _x.width, _x.height, _x.angle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.landmarks is None:
        self.landmarks = None
      if self.quads is None:
        self.quads = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8')
      else:
        self.name = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.landmarks = []
      for i in range(0, length):
        val1 = foo_msgs.msg.Risc_roi()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 25
        (_x.visible, _x.x, _x.y, _x.width, _x.height, _x.angle,) = _struct_B2i2fd.unpack(str[start:end])
        val1.visible = bool(val1.visible)
        self.landmarks.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.quads = []
      for i in range(0, length):
        val1 = foo_msgs.msg.Risc_roi()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.name = str[start:end].decode('utf-8')
        else:
          val1.name = str[start:end]
        _x = val1
        start = end
        end += 25
        (_x.visible, _x.x, _x.y, _x.width, _x.height, _x.angle,) = _struct_B2i2fd.unpack(str[start:end])
        val1.visible = bool(val1.visible)
        self.quads.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B2i2fd = struct.Struct("<B2i2fd")
