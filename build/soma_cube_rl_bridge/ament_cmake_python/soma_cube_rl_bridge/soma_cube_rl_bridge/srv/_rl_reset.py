# generated from rosidl_generator_py/resource/_idl.py.em
# with input from soma_cube_rl_bridge:srv/RLReset.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RLReset_Request(type):
    """Metaclass of message 'RLReset_Request'."""

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
                'soma_cube_rl_bridge.srv.RLReset_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__rl_reset__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__rl_reset__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__rl_reset__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__rl_reset__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__rl_reset__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RLReset_Request(metaclass=Metaclass_RLReset_Request):
    """Message class 'RLReset_Request'."""

    __slots__ = [
        '_seed',
    ]

    _fields_and_field_types = {
        'seed': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.seed = kwargs.get('seed', int())

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
        if self.seed != other.seed:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def seed(self):
        """Message field 'seed'."""
        return self._seed

    @seed.setter
    def seed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'seed' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'seed' field must be an integer in [-2147483648, 2147483647]"
        self._seed = value


# Import statements for member types

# Member 'initial_obs'
import array  # noqa: E402, I100

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_RLReset_Response(type):
    """Metaclass of message 'RLReset_Response'."""

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
                'soma_cube_rl_bridge.srv.RLReset_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__rl_reset__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__rl_reset__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__rl_reset__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__rl_reset__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__rl_reset__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RLReset_Response(metaclass=Metaclass_RLReset_Response):
    """Message class 'RLReset_Response'."""

    __slots__ = [
        '_initial_obs',
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'initial_obs': 'sequence<double>',
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.initial_obs = array.array('d', kwargs.get('initial_obs', []))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.initial_obs != other.initial_obs:
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def initial_obs(self):
        """Message field 'initial_obs'."""
        return self._initial_obs

    @initial_obs.setter
    def initial_obs(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'initial_obs' array.array() must have the type code of 'd'"
            self._initial_obs = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'initial_obs' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._initial_obs = array.array('d', value)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_RLReset(type):
    """Metaclass of service 'RLReset'."""

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
                'soma_cube_rl_bridge.srv.RLReset')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__rl_reset

            from soma_cube_rl_bridge.srv import _rl_reset
            if _rl_reset.Metaclass_RLReset_Request._TYPE_SUPPORT is None:
                _rl_reset.Metaclass_RLReset_Request.__import_type_support__()
            if _rl_reset.Metaclass_RLReset_Response._TYPE_SUPPORT is None:
                _rl_reset.Metaclass_RLReset_Response.__import_type_support__()


class RLReset(metaclass=Metaclass_RLReset):
    from soma_cube_rl_bridge.srv._rl_reset import RLReset_Request as Request
    from soma_cube_rl_bridge.srv._rl_reset import RLReset_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
