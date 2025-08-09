// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from soma_cube_rl_bridge:msg/SafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__STRUCT_HPP_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'last_event_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__soma_cube_rl_bridge__msg__SafetyState __attribute__((deprecated))
#else
# define DEPRECATED__soma_cube_rl_bridge__msg__SafetyState __declspec(deprecated)
#endif

namespace soma_cube_rl_bridge
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SafetyState_
{
  using Type = SafetyState_<ContainerAllocator>;

  explicit SafetyState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    last_event_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safe_to_move = false;
      this->reason = "";
      this->robot_mode = 0ul;
      this->robot_state = 0ul;
      this->emergency_stop = false;
      this->protective_stop = false;
    }
  }

  explicit SafetyState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    reason(_alloc),
    last_event_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->safe_to_move = false;
      this->reason = "";
      this->robot_mode = 0ul;
      this->robot_state = 0ul;
      this->emergency_stop = false;
      this->protective_stop = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _safe_to_move_type =
    bool;
  _safe_to_move_type safe_to_move;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _last_event_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _last_event_time_type last_event_time;
  using _robot_mode_type =
    uint32_t;
  _robot_mode_type robot_mode;
  using _robot_state_type =
    uint32_t;
  _robot_state_type robot_state;
  using _emergency_stop_type =
    bool;
  _emergency_stop_type emergency_stop;
  using _protective_stop_type =
    bool;
  _protective_stop_type protective_stop;
  using _joint_limit_violations_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _joint_limit_violations_type joint_limit_violations;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
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
  Type & set__robot_mode(
    const uint32_t & _arg)
  {
    this->robot_mode = _arg;
    return *this;
  }
  Type & set__robot_state(
    const uint32_t & _arg)
  {
    this->robot_state = _arg;
    return *this;
  }
  Type & set__emergency_stop(
    const bool & _arg)
  {
    this->emergency_stop = _arg;
    return *this;
  }
  Type & set__protective_stop(
    const bool & _arg)
  {
    this->protective_stop = _arg;
    return *this;
  }
  Type & set__joint_limit_violations(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->joint_limit_violations = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator> *;
  using ConstRawPtr =
    const soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__soma_cube_rl_bridge__msg__SafetyState
    std::shared_ptr<soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__soma_cube_rl_bridge__msg__SafetyState
    std::shared_ptr<soma_cube_rl_bridge::msg::SafetyState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SafetyState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->safe_to_move != other.safe_to_move) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->last_event_time != other.last_event_time) {
      return false;
    }
    if (this->robot_mode != other.robot_mode) {
      return false;
    }
    if (this->robot_state != other.robot_state) {
      return false;
    }
    if (this->emergency_stop != other.emergency_stop) {
      return false;
    }
    if (this->protective_stop != other.protective_stop) {
      return false;
    }
    if (this->joint_limit_violations != other.joint_limit_violations) {
      return false;
    }
    return true;
  }
  bool operator!=(const SafetyState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SafetyState_

// alias to use template instance with default allocator
using SafetyState =
  soma_cube_rl_bridge::msg::SafetyState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__STRUCT_HPP_
