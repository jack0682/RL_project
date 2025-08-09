// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from soma_cube_rl_bridge:srv/RLStep.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "soma_cube_rl_bridge/srv/detail/rl_step__rosidl_typesupport_introspection_c.h"
#include "soma_cube_rl_bridge/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "soma_cube_rl_bridge/srv/detail/rl_step__functions.h"
#include "soma_cube_rl_bridge/srv/detail/rl_step__struct.h"


// Include directives for member types
// Member `action`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  soma_cube_rl_bridge__srv__RLStep_Request__init(message_memory);
}

void soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_fini_function(void * message_memory)
{
  soma_cube_rl_bridge__srv__RLStep_Request__fini(message_memory);
}

size_t soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__size_function__RLStep_Request__action(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__get_const_function__RLStep_Request__action(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__get_function__RLStep_Request__action(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__fetch_function__RLStep_Request__action(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__get_const_function__RLStep_Request__action(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__assign_function__RLStep_Request__action(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__get_function__RLStep_Request__action(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__resize_function__RLStep_Request__action(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_message_member_array[1] = {
  {
    "action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__srv__RLStep_Request, action),  // bytes offset in struct
    NULL,  // default value
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__size_function__RLStep_Request__action,  // size() function pointer
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__get_const_function__RLStep_Request__action,  // get_const(index) function pointer
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__get_function__RLStep_Request__action,  // get(index) function pointer
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__fetch_function__RLStep_Request__action,  // fetch(index, &value) function pointer
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__assign_function__RLStep_Request__action,  // assign(index, value) function pointer
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__resize_function__RLStep_Request__action  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_message_members = {
  "soma_cube_rl_bridge__srv",  // message namespace
  "RLStep_Request",  // message name
  1,  // number of fields
  sizeof(soma_cube_rl_bridge__srv__RLStep_Request),
  soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_message_member_array,  // message members
  soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_message_type_support_handle = {
  0,
  &soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_soma_cube_rl_bridge
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, RLStep_Request)() {
  if (!soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_message_type_support_handle.typesupport_identifier) {
    soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &soma_cube_rl_bridge__srv__RLStep_Request__rosidl_typesupport_introspection_c__RLStep_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "soma_cube_rl_bridge/srv/detail/rl_step__rosidl_typesupport_introspection_c.h"
// already included above
// #include "soma_cube_rl_bridge/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/rl_step__functions.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/rl_step__struct.h"


// Include directives for member types
// Member `obs`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `info`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  soma_cube_rl_bridge__srv__RLStep_Response__init(message_memory);
}

void soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_fini_function(void * message_memory)
{
  soma_cube_rl_bridge__srv__RLStep_Response__fini(message_memory);
}

size_t soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__size_function__RLStep_Response__obs(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__get_const_function__RLStep_Response__obs(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__get_function__RLStep_Response__obs(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__fetch_function__RLStep_Response__obs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__get_const_function__RLStep_Response__obs(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__assign_function__RLStep_Response__obs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__get_function__RLStep_Response__obs(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__resize_function__RLStep_Response__obs(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_message_member_array[5] = {
  {
    "obs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__srv__RLStep_Response, obs),  // bytes offset in struct
    NULL,  // default value
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__size_function__RLStep_Response__obs,  // size() function pointer
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__get_const_function__RLStep_Response__obs,  // get_const(index) function pointer
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__get_function__RLStep_Response__obs,  // get(index) function pointer
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__fetch_function__RLStep_Response__obs,  // fetch(index, &value) function pointer
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__assign_function__RLStep_Response__obs,  // assign(index, value) function pointer
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__resize_function__RLStep_Response__obs  // resize(index) function pointer
  },
  {
    "reward",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__srv__RLStep_Response, reward),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "done",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__srv__RLStep_Response, done),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__srv__RLStep_Response, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__srv__RLStep_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_message_members = {
  "soma_cube_rl_bridge__srv",  // message namespace
  "RLStep_Response",  // message name
  5,  // number of fields
  sizeof(soma_cube_rl_bridge__srv__RLStep_Response),
  soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_message_member_array,  // message members
  soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_message_type_support_handle = {
  0,
  &soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_soma_cube_rl_bridge
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, RLStep_Response)() {
  if (!soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_message_type_support_handle.typesupport_identifier) {
    soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &soma_cube_rl_bridge__srv__RLStep_Response__rosidl_typesupport_introspection_c__RLStep_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "soma_cube_rl_bridge/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/rl_step__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_service_members = {
  "soma_cube_rl_bridge__srv",  // service namespace
  "RLStep",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_Request_message_type_support_handle,
  NULL  // response message
  // soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_Response_message_type_support_handle
};

static rosidl_service_type_support_t soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_service_type_support_handle = {
  0,
  &soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, RLStep_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, RLStep_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_soma_cube_rl_bridge
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, RLStep)() {
  if (!soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_service_type_support_handle.typesupport_identifier) {
    soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, RLStep_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, RLStep_Response)()->data;
  }

  return &soma_cube_rl_bridge__srv__detail__rl_step__rosidl_typesupport_introspection_c__RLStep_service_type_support_handle;
}
