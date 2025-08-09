# generated from rosidl_generator_py/resource/_idl.py.em
# with input from soma_cube_rl_bridge:msg/RLObservation.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'joint_positions'
# Member 'joint_velocities'
# Member 'tool_force'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RLObservation(type):
    """Metaclass of message 'RLObservation'."""

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
                'soma_cube_rl_bridge.msg.RLObservation')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rl_observation
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rl_observation
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rl_observation
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rl_observation
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rl_observation

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

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


class RLObservation(metaclass=Metaclass_RLObservation):
    """Message class 'RLObservation'."""

    __slots__ = [
        '_header',
        '_joint_positions',
        '_joint_velocities',
        '_tcp_pose',
        '_tool_force',
        '_gripper_status',
        '_robot_state',
        '_task_success',
        '_contact_detected',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'joint_positions': 'sequence<double>',
        'joint_velocities': 'sequence<double>',
        'tcp_pose': 'geometry_msgs/Pose',
        'tool_force': 'sequence<double>',
        'gripper_status': 'uint32',
        'robot_state': 'uint32',
        'task_success': 'boolean',
        'contact_detected': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.joint_positions = array.array('d', kwargs.get('joint_positions', []))
        self.joint_velocities = array.array('d', kwargs.get('joint_velocities', []))
        from geometry_msgs.msg import Pose
        self.tcp_pose = kwargs.get('tcp_pose', Pose())
        self.tool_force = array.array('d', kwargs.get('tool_force', []))
        self.gripper_status = kwargs.get('gripper_status', int())
        self.robot_state = kwargs.get('robot_state', int())
        self.task_success = kwargs.get('task_success', bool())
        self.contact_detected = kwargs.get('contact_detected', bool())

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
        if self.joint_positions != other.joint_positions:
            return False
        if self.joint_velocities != other.joint_velocities:
            return False
        if self.tcp_pose != other.tcp_pose:
            return False
        if self.tool_force != other.tool_force:
            return False
        if self.gripper_status != other.gripper_status:
            return False
        if self.robot_state != other.robot_state:
            return False
        if self.task_success != other.task_success:
            return False
        if self.contact_detected != other.contact_detected:
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
    def joint_positions(self):
        """Message field 'joint_positions'."""
        return self._joint_positions

    @joint_positions.setter
    def joint_positions(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'joint_positions' array.array() must have the type code of 'd'"
            self._joint_positions = value
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
                "The 'joint_positions' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._joint_positions = array.array('d', value)

    @builtins.property
    def joint_velocities(self):
        """Message field 'joint_velocities'."""
        return self._joint_velocities

    @joint_velocities.setter
    def joint_velocities(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'joint_velocities' array.array() must have the type code of 'd'"
            self._joint_velocities = value
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
                "The 'joint_velocities' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._joint_velocities = array.array('d', value)

    @builtins.property
    def tcp_pose(self):
        """Message field 'tcp_pose'."""
        return self._tcp_pose

    @tcp_pose.setter
    def tcp_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'tcp_pose' field must be a sub message of type 'Pose'"
        self._tcp_pose = value

    @builtins.property
    def tool_force(self):
        """Message field 'tool_force'."""
        return self._tool_force

    @tool_force.setter
    def tool_force(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'tool_force' array.array() must have the type code of 'd'"
            self._tool_force = value
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
                "The 'tool_force' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._tool_force = array.array('d', value)

    @builtins.property
    def gripper_status(self):
        """Message field 'gripper_status'."""
        return self._gripper_status

    @gripper_status.setter
    def gripper_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gripper_status' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'gripper_status' field must be an unsigned integer in [0, 4294967295]"
        self._gripper_status = value

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
    def task_success(self):
        """Message field 'task_success'."""
        return self._task_success

    @task_success.setter
    def task_success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'task_success' field must be of type 'bool'"
        self._task_success = value

    @builtins.property
    def contact_detected(self):
        """Message field 'contact_detected'."""
        return self._contact_detected

    @contact_detected.setter
    def contact_detected(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'contact_detected' field must be of type 'bool'"
        self._contact_detected = value
