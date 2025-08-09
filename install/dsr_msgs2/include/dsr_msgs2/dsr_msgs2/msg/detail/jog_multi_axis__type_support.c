// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dsr_msgs2:msg/JogMultiAxis.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dsr_msgs2/msg/detail/jog_multi_axis__rosidl_typesupport_introspection_c.h"
#include "dsr_msgs2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dsr_msgs2/msg/detail/jog_multi_axis__functions.h"
#include "dsr_msgs2/msg/detail/jog_multi_axis__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dsr_msgs2__msg__JogMultiAxis__init(message_memory);
}

void dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_fini_function(void * message_memory)
{
  dsr_msgs2__msg__JogMultiAxis__fini(message_memory);
}

size_t dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__size_function__JogMultiAxis__jog_axis(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__get_const_function__JogMultiAxis__jog_axis(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__get_function__JogMultiAxis__jog_axis(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__fetch_function__JogMultiAxis__jog_axis(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__get_const_function__JogMultiAxis__jog_axis(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__assign_function__JogMultiAxis__jog_axis(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__get_function__JogMultiAxis__jog_axis(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_message_member_array[3] = {
  {
    "jog_axis",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(dsr_msgs2__msg__JogMultiAxis, jog_axis),  // bytes offset in struct
    NULL,  // default value
    dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__size_function__JogMultiAxis__jog_axis,  // size() function pointer
    dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__get_const_function__JogMultiAxis__jog_axis,  // get_const(index) function pointer
    dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__get_function__JogMultiAxis__jog_axis,  // get(index) function pointer
    dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__fetch_function__JogMultiAxis__jog_axis,  // fetch(index, &value) function pointer
    dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__assign_function__JogMultiAxis__jog_axis,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "move_reference",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dsr_msgs2__msg__JogMultiAxis, move_reference),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dsr_msgs2__msg__JogMultiAxis, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_message_members = {
  "dsr_msgs2__msg",  // message namespace
  "JogMultiAxis",  // message name
  3,  // number of fields
  sizeof(dsr_msgs2__msg__JogMultiAxis),
  dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_message_member_array,  // message members
  dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_init_function,  // function to initialize message memory (memory has to be allocated)
  dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_message_type_support_handle = {
  0,
  &dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dsr_msgs2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dsr_msgs2, msg, JogMultiAxis)() {
  if (!dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_message_type_support_handle.typesupport_identifier) {
    dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dsr_msgs2__msg__JogMultiAxis__rosidl_typesupport_introspection_c__JogMultiAxis_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
