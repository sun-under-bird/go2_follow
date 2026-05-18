# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'rssi'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LibAoaRobotMsg(type):
    """Metaclass of message 'LibAoaRobotMsg'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uwb_aoa_pkg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uwb_aoa_pkg.msg.LibAoaRobotMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__lib_aoa_robot_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__lib_aoa_robot_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__lib_aoa_robot_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__lib_aoa_robot_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__lib_aoa_robot_msg

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LibAoaRobotMsg(metaclass=Metaclass_LibAoaRobotMsg):
    """Message class 'LibAoaRobotMsg'."""

    __slots__ = [
        '_r',
        '_a',
        '_x',
        '_y',
        '_state',
        '_rssi',
        '_pos_confidence',
    ]

    _fields_and_field_types = {
        'r': 'double',
        'a': 'double',
        'x': 'double',
        'y': 'double',
        'state': 'int8',
        'rssi': 'int8[6]',
        'pos_confidence': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int8'), 6),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.r = kwargs.get('r', float())
        self.a = kwargs.get('a', float())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.state = kwargs.get('state', int())
        if 'rssi' not in kwargs:
            self.rssi = numpy.zeros(6, dtype=numpy.int8)
        else:
            self.rssi = kwargs.get('rssi')
        self.pos_confidence = kwargs.get('pos_confidence', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.r != other.r:
            return False
        if self.a != other.a:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.state != other.state:
            return False
        if any(self.rssi != other.rssi):
            return False
        if self.pos_confidence != other.pos_confidence:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def r(self):
        """Message field 'r'."""
        return self._r

    @r.setter
    def r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'r' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._r = value

    @builtins.property
    def a(self):
        """Message field 'a'."""
        return self._a

    @a.setter
    def a(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'a' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'a' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._a = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'state' field must be an integer in [-128, 127]"
        self._state = value

    @builtins.property
    def rssi(self):
        """Message field 'rssi'."""
        return self._rssi

    @rssi.setter
    def rssi(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int8, \
                "The 'rssi' numpy.ndarray() must have the dtype of 'numpy.int8'"
            assert value.size == 6, \
                "The 'rssi' numpy.ndarray() must have a size of 6"
            self._rssi = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 6 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -128 and val < 128 for val in value)), \
                "The 'rssi' field must be a set or sequence with length 6 and each value of type 'int' and each integer in [-128, 127]"
        self._rssi = numpy.array(value, dtype=numpy.int8)

    @builtins.property
    def pos_confidence(self):
        """Message field 'pos_confidence'."""
        return self._pos_confidence

    @pos_confidence.setter
    def pos_confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pos_confidence' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'pos_confidence' field must be an unsigned integer in [0, 255]"
        self._pos_confidence = value
