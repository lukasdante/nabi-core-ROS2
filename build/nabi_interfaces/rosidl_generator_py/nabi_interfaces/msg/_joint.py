# generated from rosidl_generator_py/resource/_idl.py.em
# with input from nabi_interfaces:msg/Joint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Joint(type):
    """Metaclass of message 'Joint'."""

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
            module = import_type_support('nabi_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'nabi_interfaces.msg.Joint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__joint
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__joint
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__joint
            cls._TYPE_SUPPORT = module.type_support_msg__msg__joint
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__joint

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Joint(metaclass=Metaclass_Joint):
    """Message class 'Joint'."""

    __slots__ = [
        '_can_id',
        '_joint_angle',
        '_velocity',
        '_acceleration',
    ]

    _fields_and_field_types = {
        'can_id': 'uint8',
        'joint_angle': 'int16',
        'velocity': 'uint16',
        'acceleration': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.can_id = kwargs.get('can_id', int())
        self.joint_angle = kwargs.get('joint_angle', int())
        self.velocity = kwargs.get('velocity', int())
        self.acceleration = kwargs.get('acceleration', int())

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
        if self.can_id != other.can_id:
            return False
        if self.joint_angle != other.joint_angle:
            return False
        if self.velocity != other.velocity:
            return False
        if self.acceleration != other.acceleration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def can_id(self):
        """Message field 'can_id'."""
        return self._can_id

    @can_id.setter
    def can_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'can_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'can_id' field must be an unsigned integer in [0, 255]"
        self._can_id = value

    @builtins.property
    def joint_angle(self):
        """Message field 'joint_angle'."""
        return self._joint_angle

    @joint_angle.setter
    def joint_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'joint_angle' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'joint_angle' field must be an integer in [-32768, 32767]"
        self._joint_angle = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'velocity' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'velocity' field must be an unsigned integer in [0, 65535]"
        self._velocity = value

    @builtins.property
    def acceleration(self):
        """Message field 'acceleration'."""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'acceleration' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'acceleration' field must be an unsigned integer in [0, 255]"
        self._acceleration = value
