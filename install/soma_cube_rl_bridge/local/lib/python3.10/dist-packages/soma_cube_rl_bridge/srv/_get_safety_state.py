# generated from rosidl_generator_py/resource/_idl.py.em
# with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetSafetyState_Request(type):
    """Metaclass of message 'GetSafetyState_Request'."""

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
                'soma_cube_rl_bridge.srv.GetSafetyState_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_safety_state__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_safety_state__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_safety_state__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_safety_state__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_safety_state__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetSafetyState_Request(metaclass=Metaclass_GetSafetyState_Request):
    """Message class 'GetSafetyState_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetSafetyState_Response(type):
    """Metaclass of message 'GetSafetyState_Response'."""

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
                'soma_cube_rl_bridge.srv.GetSafetyState_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_safety_state__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_safety_state__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_safety_state__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_safety_state__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_safety_state__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetSafetyState_Response(metaclass=Metaclass_GetSafetyState_Response):
    """Message class 'GetSafetyState_Response'."""

    __slots__ = [
        '_safe_to_move',
        '_reason',
        '_last_event_time',
    ]

    _fields_and_field_types = {
        'safe_to_move': 'boolean',
        'reason': 'string',
        'last_event_time': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.safe_to_move = kwargs.get('safe_to_move', bool())
        self.reason = kwargs.get('reason', str())
        from builtin_interfaces.msg import Time
        self.last_event_time = kwargs.get('last_event_time', Time())

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
        if self.safe_to_move != other.safe_to_move:
            return False
        if self.reason != other.reason:
            return False
        if self.last_event_time != other.last_event_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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


class Metaclass_GetSafetyState(type):
    """Metaclass of service 'GetSafetyState'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('soma_cube_rl_bridge')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'soma_cube_rl_bridge.srv.GetSafetyState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_safety_state

            from soma_cube_rl_bridge.srv import _get_safety_state
            if _get_safety_state.Metaclass_GetSafetyState_Request._TYPE_SUPPORT is None:
                _get_safety_state.Metaclass_GetSafetyState_Request.__import_type_support__()
            if _get_safety_state.Metaclass_GetSafetyState_Response._TYPE_SUPPORT is None:
                _get_safety_state.Metaclass_GetSafetyState_Response.__import_type_support__()


class GetSafetyState(metaclass=Metaclass_GetSafetyState):
    from soma_cube_rl_bridge.srv._get_safety_state import GetSafetyState_Request as Request
    from soma_cube_rl_bridge.srv._get_safety_state import GetSafetyState_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
