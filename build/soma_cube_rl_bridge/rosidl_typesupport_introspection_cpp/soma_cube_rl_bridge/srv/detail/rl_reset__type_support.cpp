// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from soma_cube_rl_bridge:srv/RLReset.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "soma_cube_rl_bridge/srv/detail/rl_reset__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace soma_cube_rl_bridge
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void RLReset_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) soma_cube_rl_bridge::srv::RLReset_Request(_init);
}

void RLReset_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<soma_cube_rl_bridge::srv::RLReset_Request *>(message_memory);
  typed_message->~RLReset_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RLReset_Request_message_member_array[1] = {
  {
    "seed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge::srv::RLReset_Request, seed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RLReset_Request_message_members = {
  "soma_cube_rl_bridge::srv",  // message namespace
  "RLReset_Request",  // message name
  1,  // number of fields
  sizeof(soma_cube_rl_bridge::srv::RLReset_Request),
  RLReset_Request_message_member_array,  // message members
  RLReset_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  RLReset_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RLReset_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RLReset_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace soma_cube_rl_bridge


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<soma_cube_rl_bridge::srv::RLReset_Request>()
{
  return &::soma_cube_rl_bridge::srv::rosidl_typesupport_introspection_cpp::RLReset_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, soma_cube_rl_bridge, srv, RLReset_Request)() {
  return &::soma_cube_rl_bridge::srv::rosidl_typesupport_introspection_cpp::RLReset_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/rl_reset__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace soma_cube_rl_bridge
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void RLReset_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) soma_cube_rl_bridge::srv::RLReset_Response(_init);
}

void RLReset_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<soma_cube_rl_bridge::srv::RLReset_Response *>(message_memory);
  typed_message->~RLReset_Response();
}

size_t size_function__RLReset_Response__initial_obs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__RLReset_Response__initial_obs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__RLReset_Response__initial_obs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__RLReset_Response__initial_obs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RLReset_Response__initial_obs(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RLReset_Response__initial_obs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RLReset_Response__initial_obs(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__RLReset_Response__initial_obs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RLReset_Response_message_member_array[3] = {
  {
    "initial_obs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge::srv::RLReset_Response, initial_obs),  // bytes offset in struct
    nullptr,  // default value
    size_function__RLReset_Response__initial_obs,  // size() function pointer
    get_const_function__RLReset_Response__initial_obs,  // get_const(index) function pointer
    get_function__RLReset_Response__initial_obs,  // get(index) function pointer
    fetch_function__RLReset_Response__initial_obs,  // fetch(index, &value) function pointer
    assign_function__RLReset_Response__initial_obs,  // assign(index, value) function pointer
    resize_function__RLReset_Response__initial_obs  // resize(index) function pointer
  },
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge::srv::RLReset_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(soma_cube_rl_bridge::srv::RLReset_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RLReset_Response_message_members = {
  "soma_cube_rl_bridge::srv",  // message namespace
  "RLReset_Response",  // message name
  3,  // number of fields
  sizeof(soma_cube_rl_bridge::srv::RLReset_Response),
  RLReset_Response_message_member_array,  // message members
  RLReset_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  RLReset_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RLReset_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RLReset_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace soma_cube_rl_bridge


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<soma_cube_rl_bridge::srv::RLReset_Response>()
{
  return &::soma_cube_rl_bridge::srv::rosidl_typesupport_introspection_cpp::RLReset_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, soma_cube_rl_bridge, srv, RLReset_Response)() {
  return &::soma_cube_rl_bridge::srv::rosidl_typesupport_introspection_cpp::RLReset_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "soma_cube_rl_bridge/srv/detail/rl_reset__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace soma_cube_rl_bridge
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers RLReset_service_members = {
  "soma_cube_rl_bridge::srv",  // service namespace
  "RLReset",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<soma_cube_rl_bridge::srv::RLReset>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t RLReset_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RLReset_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace soma_cube_rl_bridge


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<soma_cube_rl_bridge::srv::RLReset>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::soma_cube_rl_bridge::srv::rosidl_typesupport_introspection_cpp::RLReset_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::soma_cube_rl_bridge::srv::RLReset_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::soma_cube_rl_bridge::srv::RLReset_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, soma_cube_rl_bridge, srv, RLReset)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<soma_cube_rl_bridge::srv::RLReset>();
}

#ifdef __cplusplus
}
#endif
