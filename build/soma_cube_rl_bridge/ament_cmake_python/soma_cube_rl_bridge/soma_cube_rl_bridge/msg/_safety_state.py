# generated from rosidl_generator_py/resource/_idl.py.em
# with input from soma_cube_rl_bridge:msg/SafetyState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SafetyState(type):
    """Metaclass of message 'SafetyState'."""

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
            module = import_type_support('soma_cube_rl_bridge')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'soma_cube_rl_bridge.msg.SafetyState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__safety_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__safety_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__safety_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__safety_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__safety_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SafetyState(metaclass=Metaclass_SafetyState):
    """Message class 'SafetyState'."""

    __slots__ = [
        '_header',
        '_safe_to_move',
        '_reason',
        '_last_event_time',
        '_robot_mode',
        '_robot_state',
        '_emergency_stop',
        '_protective_stop',
        '_joint_limit_violations',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'safe_to_move': 'boolean',
        'reason': 'string',
        'last_event_time': 'builtin_interfaces/Time',
        'robot_mode': 'uint32',
        'robot_state': 'uint32',
        'emergency_stop': 'boolean',
        'protective_stop': 'boolean',
        'joint_limit_violations': 'sequence<boolean>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.safe_to_move = kwargs.get('safe_to_move', bool())
        self.reason = kwargs.get('reason', str())
        from builtin_interfaces.msg import Time
        self.last_event_time = kwargs.get('last_event_time', Time())
        self.robot_mode = kwargs.get('robot_mode', int())
        self.robot_state = kwargs.get('robot_state', int())
        self.emergency_stop = kwargs.get('emergency_stop', bool())
        self.protective_stop = kwargs.get('protective_stop', bool())
        self.joint_limit_violations = kwargs.get('joint_limit_violations', [])

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
        if self.header != other.header:
            return False
        if self.safe_to_move != other.safe_to_move:
            return False
        if self.reason != other.reason:
            return False
        if self.last_event_time != other.last_event_time:
            return False
        if self.robot_mode != other.robot_mode:
            return False
        if self.robot_state != other.robot_state:
            return False
        if self.emergency_stop != other.emergency_stop:
            return False
        if self.protective_stop != other.protective_stop:
            return False
        if self.joint_limit_violations != other.joint_limit_violations:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def safe_to_move(self):
        """Message field 'safe_to_move'."""
        return self._safe_to_move

    @safe_to_move.setter
    def safe_to_move(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'safe_to_move' field must be of type 'bool'"
        self._safe_to_move = value

    @builtins.property
    def reason(self):
        """Message field 'reason'."""
        return self._reason

    @reason.setter
    def reason(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'reason' field must be of type 'str'"
        self._reason = value

    @builtins.property
    def last_event_time(self):
        """Message field 'last_event_time'."""
        return self._last_event_time

    @last_event_time.setter
    def last_event_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'last_event_time' field must be a sub message of type 'Time'"
        self._last_event_time = value

    @builtins.property
    def robot_mode(self):
        """Message field 'robot_mode'."""
        return self._robot_mode

    @robot_mode.setter
    def robot_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_mode' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'robot_mode' field must be an unsigned integer in [0, 4294967295]"
        self._robot_mode = value

    @builtins.property
    def robot_state(self):
        """Message field 'robot_state'."""
        return self._robot_state

    @robot_state.setter
    def robot_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_state' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'robot_state' field must be an unsigned integer in [0, 4294967295]"
        self._robot_state = value

    @builtins.property
    def emergency_stop(self):
        """Message field 'emergency_stop'."""
        return self._emergency_stop

    @emergency_stop.setter
    def emergency_stop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'emergency_stop' field must be of type 'bool'"
        self._emergency_stop = value

    @builtins.property
    def protective_stop(self):
        """Message field 'protective_stop'."""
        return self._protective_stop

    @protective_stop.setter
    def protective_stop(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'protective_stop' field must be of type 'bool'"
        self._protective_stop = value

    @builtins.property
    def joint_limit_violations(self):
        """Message field 'joint_limit_violations'."""
        return self._joint_limit_violations

    @joint_limit_violations.setter
    def joint_limit_violations(self, value):
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'joint_limit_violations' field must be a set or sequence and each value of type 'bool'"
        self._joint_limit_violations = value
