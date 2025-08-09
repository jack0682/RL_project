// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from soma_cube_rl_bridge:msg/RLObservation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "soma_cube_rl_bridge/msg/detail/rl_observation__rosidl_typesupport_introspection_c.h"
#include "soma_cube_rl_bridge/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "soma_cube_rl_bridge/msg/detail/rl_observation__functions.h"
#include "soma_cube_rl_bridge/msg/detail/rl_observation__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `joint_positions`
// Member `joint_velocities`
// Member `tool_force`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `tcp_pose`
#include "geometry_msgs/msg/pose.h"
// Member `tcp_pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  soma_cube_rl_bridge__msg__RLObservation__init(message_memory);
}

void soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_fini_function(void * message_memory)
{
  soma_cube_rl_bridge__msg__RLObservation__fini(message_memory);
}

size_t soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__size_function__RLObservation__joint_positions(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__joint_positions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__joint_positions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__fetch_function__RLObservation__joint_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__joint_positions(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__assign_function__RLObservation__joint_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__joint_positions(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__resize_function__RLObservation__joint_positions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__size_function__RLObservation__joint_velocities(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__joint_velocities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__joint_velocities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__fetch_function__RLObservation__joint_velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__joint_velocities(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__assign_function__RLObservation__joint_velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__joint_velocities(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__resize_function__RLObservation__joint_velocities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__size_function__RLObservation__tool_force(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__tool_force(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__tool_force(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__fetch_function__RLObservation__tool_force(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__tool_force(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__assign_function__RLObservation__tool_force(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__tool_force(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__resize_function__RLObservation__tool_force(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__RLObservation, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__RLObservation, joint_positions),  // bytes offset in struct
    NULL,  // default value
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__size_function__RLObservation__joint_positions,  // size() function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__joint_positions,  // get_const(index) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__joint_positions,  // get(index) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__fetch_function__RLObservation__joint_positions,  // fetch(index, &value) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__assign_function__RLObservation__joint_positions,  // assign(index, value) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__resize_function__RLObservation__joint_positions  // resize(index) function pointer
  },
  {
    "joint_velocities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__RLObservation, joint_velocities),  // bytes offset in struct
    NULL,  // default value
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__size_function__RLObservation__joint_velocities,  // size() function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__joint_velocities,  // get_const(index) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__joint_velocities,  // get(index) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__fetch_function__RLObservation__joint_velocities,  // fetch(index, &value) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__assign_function__RLObservation__joint_velocities,  // assign(index, value) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__resize_function__RLObservation__joint_velocities  // resize(index) function pointer
  },
  {
    "tcp_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__RLObservation, tcp_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tool_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__RLObservation, tool_force),  // bytes offset in struct
    NULL,  // default value
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__size_function__RLObservation__tool_force,  // size() function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_const_function__RLObservation__tool_force,  // get_const(index) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__get_function__RLObservation__tool_force,  // get(index) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__fetch_function__RLObservation__tool_force,  // fetch(index, &value) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__assign_function__RLObservation__tool_force,  // assign(index, value) function pointer
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__resize_function__RLObservation__tool_force  // resize(index) function pointer
  },
  {
    "gripper_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__RLObservation, gripper_status),  // bytes offset in struct
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
    offsetof(soma_cube_rl_bridge__msg__RLObservation, robot_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "task_success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__RLObservation, task_success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "contact_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__msg__RLObservation, contact_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_members = {
  "soma_cube_rl_bridge__msg",  // message namespace
  "RLObservation",  // message name
  9,  // number of fields
  sizeof(soma_cube_rl_bridge__msg__RLObservation),
  soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_member_array,  // message members
  soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_init_function,  // function to initialize message memory (memory has to be allocated)
  soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_type_support_handle = {
  0,
  &soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_soma_cube_rl_bridge
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, msg, RLObservation)() {
  soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_type_support_handle.typesupport_identifier) {
    soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &soma_cube_rl_bridge__msg__RLObservation__rosidl_typesupport_introspection_c__RLObservation_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
