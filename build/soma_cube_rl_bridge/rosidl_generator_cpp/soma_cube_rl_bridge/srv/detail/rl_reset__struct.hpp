// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from soma_cube_rl_bridge:srv/RLReset.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__STRUCT_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__soma_cube_rl_bridge__srv__RLReset_Request __attribute__((deprecated))
#else
# define DEPRECATED__soma_cube_rl_bridge__srv__RLReset_Request __declspec(deprecated)
#endif

namespace soma_cube_rl_bridge
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RLReset_Request_
{
  using Type = RLReset_Request_<ContainerAllocator>;

  explicit RLReset_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seed = 0l;
    }
  }

  explicit RLReset_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->seed = 0l;
    }
  }

  // field types and members
  using _seed_type =
    int32_t;
  _seed_type seed;

  // setters for named parameter idiom
  Type & set__seed(
    const int32_t & _arg)
  {
    this->seed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__soma_cube_rl_bridge__srv__RLReset_Request
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__soma_cube_rl_bridge__srv__RLReset_Request
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RLReset_Request_ & other) const
  {
    if (this->seed != other.seed) {
      return false;
    }
    return true;
  }
  bool operator!=(const RLReset_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RLReset_Request_

// alias to use template instance with default allocator
using RLReset_Request =
  soma_cube_rl_bridge::srv::RLReset_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace soma_cube_rl_bridge


#ifndef _WIN32
# define DEPRECATED__soma_cube_rl_bridge__srv__RLReset_Response __attribute__((deprecated))
#else
# define DEPRECATED__soma_cube_rl_bridge__srv__RLReset_Response __declspec(deprecated)
#endif

namespace soma_cube_rl_bridge
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RLReset_Response_
{
  using Type = RLReset_Response_<ContainerAllocator>;

  explicit RLReset_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit RLReset_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _initial_obs_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _initial_obs_type initial_obs;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__initial_obs(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->initial_obs = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__soma_cube_rl_bridge__srv__RLReset_Response
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__soma_cube_rl_bridge__srv__RLReset_Response
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RLReset_Response_ & other) const
  {
    if (this->initial_obs != other.initial_obs) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const RLReset_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RLReset_Response_

// alias to use template instance with default allocator
using RLReset_Response =
  soma_cube_rl_bridge::srv::RLReset_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace soma_cube_rl_bridge

namespace soma_cube_rl_bridge
{

namespace srv
{

struct RLReset
{
  using Request = soma_cube_rl_bridge::srv::RLReset_Request;
  using Response = soma_cube_rl_bridge::srv::RLReset_Response;
};

}  // namespace srv

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__STRUCT_HPP_
