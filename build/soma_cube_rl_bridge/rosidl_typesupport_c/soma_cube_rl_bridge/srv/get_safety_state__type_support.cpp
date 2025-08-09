// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "soma_cube_rl_bridge/srv/detail/get_safety_state__struct.h"
#include "soma_cube_rl_bridge/srv/detail/get_safety_state__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace soma_cube_rl_bridge
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetSafetyState_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetSafetyState_Request_type_support_ids_t;

static const _GetSafetyState_Request_type_support_ids_t _GetSafetyState_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetSafetyState_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetSafetyState_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetSafetyState_Request_type_support_symbol_names_t _GetSafetyState_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, soma_cube_rl_bridge, srv, GetSafetyState_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState_Request)),
  }
};

typedef struct _GetSafetyState_Request_type_support_data_t
{
  void * data[2];
} _GetSafetyState_Request_type_support_data_t;

static _GetSafetyState_Request_type_support_data_t _GetSafetyState_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetSafetyState_Request_message_typesupport_map = {
  2,
  "soma_cube_rl_bridge",
  &_GetSafetyState_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetSafetyState_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetSafetyState_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetSafetyState_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetSafetyState_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace soma_cube_rl_bridge

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, soma_cube_rl_bridge, srv, GetSafetyState_Request)() {
  return &::soma_cube_rl_bridge::srv::rosidl_typesupport_c::GetSafetyState_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/get_safety_state__struct.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/get_safety_state__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace soma_cube_rl_bridge
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetSafetyState_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetSafetyState_Response_type_support_ids_t;

static const _GetSafetyState_Response_type_support_ids_t _GetSafetyState_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetSafetyState_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetSafetyState_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetSafetyState_Response_type_support_symbol_names_t _GetSafetyState_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, soma_cube_rl_bridge, srv, GetSafetyState_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState_Response)),
  }
};

typedef struct _GetSafetyState_Response_type_support_data_t
{
  void * data[2];
} _GetSafetyState_Response_type_support_data_t;

static _GetSafetyState_Response_type_support_data_t _GetSafetyState_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetSafetyState_Response_message_typesupport_map = {
  2,
  "soma_cube_rl_bridge",
  &_GetSafetyState_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetSafetyState_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetSafetyState_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetSafetyState_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetSafetyState_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace soma_cube_rl_bridge

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, soma_cube_rl_bridge, srv, GetSafetyState_Response)() {
  return &::soma_cube_rl_bridge::srv::rosidl_typesupport_c::GetSafetyState_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/get_safety_state__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace soma_cube_rl_bridge
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _GetSafetyState_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetSafetyState_type_support_ids_t;

static const _GetSafetyState_type_support_ids_t _GetSafetyState_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _GetSafetyState_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetSafetyState_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetSafetyState_type_support_symbol_names_t _GetSafetyState_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, soma_cube_rl_bridge, srv, GetSafetyState)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, soma_cube_rl_bridge, srv, GetSafetyState)),
  }
};

typedef struct _GetSafetyState_type_support_data_t
{
  void * data[2];
} _GetSafetyState_type_support_data_t;

static _GetSafetyState_type_support_data_t _GetSafetyState_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetSafetyState_service_typesupport_map = {
  2,
  "soma_cube_rl_bridge",
  &_GetSafetyState_service_typesupport_ids.typesupport_identifier[0],
  &_GetSafetyState_service_typesupport_symbol_names.symbol_name[0],
  &_GetSafetyState_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetSafetyState_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetSafetyState_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace soma_cube_rl_bridge

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, soma_cube_rl_bridge, srv, GetSafetyState)() {
  return &::soma_cube_rl_bridge::srv::rosidl_typesupport_c::GetSafetyState_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
