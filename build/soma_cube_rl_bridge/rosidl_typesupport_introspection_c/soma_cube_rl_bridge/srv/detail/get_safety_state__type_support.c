// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "soma_cube_rl_bridge/srv/detail/get_safety_state__rosidl_typesupport_introspection_c.h"
#include "soma_cube_rl_bridge/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "soma_cube_rl_bridge/srv/detail/get_safety_state__functions.h"
#include "soma_cube_rl_bridge/srv/detail/get_safety_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  soma_cube_rl_bridge__srv__GetSafetyState_Request__init(message_memory);
}

void soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_fini_function(void * message_memory)
{
  soma_cube_rl_bridge__srv__GetSafetyState_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__srv__GetSafetyState_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_members = {
  "soma_cube_rl_bridge__srv",  // message namespace
  "GetSafetyState_Request",  // message name
  1,  // number of fields
  sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Request),
  soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_member_array,  // message members
  soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_type_support_handle = {
  0,
  &soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_soma_cube_rl_bridge
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState_Request)() {
  if (!soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_type_support_handle.typesupport_identifier) {
    soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &soma_cube_rl_bridge__srv__GetSafetyState_Request__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "soma_cube_rl_bridge/srv/detail/get_safety_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "soma_cube_rl_bridge/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/get_safety_state__functions.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/get_safety_state__struct.h"


// Include directives for member types
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `last_event_time`
#include "builtin_interfaces/msg/time.h"
// Member `last_event_time`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  soma_cube_rl_bridge__srv__GetSafetyState_Response__init(message_memory);
}

void soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_fini_function(void * message_memory)
{
  soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_member_array[3] = {
  {
    "safe_to_move",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge__srv__GetSafetyState_Response, safe_to_move),  // bytes offset in struct
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
    offsetof(soma_cube_rl_bridge__srv__GetSafetyState_Response, reason),  // bytes offset in struct
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
    offsetof(soma_cube_rl_bridge__srv__GetSafetyState_Response, last_event_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_members = {
  "soma_cube_rl_bridge__srv",  // message namespace
  "GetSafetyState_Response",  // message name
  3,  // number of fields
  sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Response),
  soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_member_array,  // message members
  soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_type_support_handle = {
  0,
  &soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_soma_cube_rl_bridge
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState_Response)() {
  soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_type_support_handle.typesupport_identifier) {
    soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &soma_cube_rl_bridge__srv__GetSafetyState_Response__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "soma_cube_rl_bridge/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/get_safety_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_service_members = {
  "soma_cube_rl_bridge__srv",  // service namespace
  "GetSafetyState",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_Request_message_type_support_handle,
  NULL  // response message
  // soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_Response_message_type_support_handle
};

static rosidl_service_type_support_t soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_service_type_support_handle = {
  0,
  &soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_soma_cube_rl_bridge
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState)() {
  if (!soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_service_type_support_handle.typesupport_identifier) {
    soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState_Response)()->data;
  }

  return &soma_cube_rl_bridge__srv__detail__get_safety_state__rosidl_typesupport_introspection_c__GetSafetyState_service_type_support_handle;
}
