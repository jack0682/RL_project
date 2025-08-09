// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__STRUCT_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__soma_cube_rl_bridge__srv__GetSafetyState_Request __attribute__((deprecated))
#else
# define DEPRECATED__soma_cube_rl_bridge__srv__GetSafetyState_Request __declspec(deprecated)
#endif

namespace soma_cube_rl_bridge
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSafetyState_Request_
{
  using Type = GetSafetyState_Request_<ContainerAllocator>;

  explicit GetSafetyState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetSafetyState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__soma_cube_rl_bridge__srv__GetSafetyState_Request
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__soma_cube_rl_bridge__srv__GetSafetyState_Request
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSafetyState_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSafetyState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSafetyState_Request_

// alias to use template instance with default allocator
using GetSafetyState_Request =
  soma_cube_rl_bridge::srv::GetSafetyState_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace soma_cube_rl_bridge


// Include directives for member types
// Member 'last_event_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__soma_cube_rl_bridge__srv__GetSafetyState_Response __attribute__((deprecated))
#else
# define DEPRECATED__soma_cube_rl_bridge__srv__GetSafetyState_Response __declspec(deprecated)
#endif

namespace soma_cube_rl_bridge
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetSafetyState_Response_
{
  using Type = GetSafetyState_Response_<ContainerAllocator>;

  explicit GetSafetyState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : last_event_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safe_to_move = false;
      this->reason = "";
    }
  }

  explicit GetSafetyState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reason(_alloc),
    last_event_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safe_to_move = false;
      this->reason = "";
    }
  }

  // field types and members
  using _safe_to_move_type =
    bool;
  _safe_to_move_type safe_to_move;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _last_event_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _last_event_time_type last_event_time;

  // setters for named parameter idiom
  Type & set__safe_to_move(
    const bool & _arg)
  {
    this->safe_to_move = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }
  Type & set__last_event_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->last_event_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__soma_cube_rl_bridge__srv__GetSafetyState_Response
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__soma_cube_rl_bridge__srv__GetSafetyState_Response
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetSafetyState_Response_ & other) const
  {
    if (this->safe_to_move != other.safe_to_move) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->last_event_time != other.last_event_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetSafetyState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetSafetyState_Response_

// alias to use template instance with default allocator
using GetSafetyState_Response =
  soma_cube_rl_bridge::srv::GetSafetyState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace soma_cube_rl_bridge

namespace soma_cube_rl_bridge
{

namespace srv
{

struct GetSafetyState
{
  using Request = soma_cube_rl_bridge::srv::GetSafetyState_Request;
  using Response = soma_cube_rl_bridge::srv::GetSafetyState_Response;
};

}  // namespace srv

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__STRUCT_HPP_
