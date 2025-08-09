// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from soma_cube_rl_bridge:msg/SafetyState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "soma_cube_rl_bridge/msg/detail/safety_state__rosidl_typesupport_introspection_c.h"
#include "soma_cube_rl_bridge/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "soma_cube_rl_bridge/msg/detail/safety_state__functions.h"
#include "soma_cube_rl_bridge/msg/detail/safety_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `last_event_time`
#include "builtin_interfaces/msg/time.h"
// Member `last_event_time`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `joint_limit_violations`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  soma_cube_rl_bridge__msg__SafetyState__init(message_memory);
}

void soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_fini_function(void * message_memory)
{
  soma_cube_rl_bridge__msg__SafetyState__fini(message_memory);
}

size_t soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__size_function__SafetyState__joint_limit_violations(
  const void * untyped_member)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return member->size;
}

const void * soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__get_const_function__SafetyState__joint_limit_violations(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__boolean__Sequence * member =
    (const rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void * soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__get_function__SafetyState__joint_limit_violations(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  return &member->data[index];
}

void soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__fetch_function__SafetyState__joint_limit_violations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__get_const_function__SafetyState__joint_limit_violations(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__assign_function__SafetyState__joint_limit_violations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__get_function__SafetyState__joint_limit_violations(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

bool soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__resize_function__SafetyState__joint_limit_violations(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__boolean__Sequence * member =
    (rosidl_runtime_c__boolean__Sequence *)(untyped_member);
  rosidl_runtime_c__boolean__Sequence__fini(member);
  return rosidl_runtime_c__boolean__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "safe_to_move",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, safe_to_move),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "last_event_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, last_event_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, robot_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, robot_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "emergency_stop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, emergency_stop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "protective_stop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, protective_stop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_limit_violations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__SafetyState, joint_limit_violations),  // bytes offset in struct
    NULL,  // default value
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__size_function__SafetyState__joint_limit_violations,  // size() function pointer
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__get_const_function__SafetyState__joint_limit_violations,  // get_const(index) function pointer
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__get_function__SafetyState__joint_limit_violations,  // get(index) function pointer
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__fetch_function__SafetyState__joint_limit_violations,  // fetch(index, &value) function pointer
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__assign_function__SafetyState__joint_limit_violations,  // assign(index, value) function pointer
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__resize_function__SafetyState__joint_limit_violations  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_members = {
  "soma_cube_rl_bridge__msg",  // message namespace
  "SafetyState",  // message name
  9,  // number of fields
  sizeof(soma_cube_rl_bridge__msg__SafetyState),
  soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_member_array,  // message members
  soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_init_function,  // function to initialize message memory (memory has to be allocated)
  soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_type_support_handle = {
  0,
  &soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_soma_cube_rl_bridge
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, msg, SafetyState)() {
  soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_type_support_handle.typesupport_identifier) {
    soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &soma_cube_rl_bridge__msg__SafetyState__rosidl_typesupport_introspection_c__SafetyState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
