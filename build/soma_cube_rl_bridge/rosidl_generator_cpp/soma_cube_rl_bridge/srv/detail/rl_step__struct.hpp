// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from soma_cube_rl_bridge:srv/RLStep.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__STRUCT_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__soma_cube_rl_bridge__srv__RLStep_Request __attribute__((deprecated))
#else
# define DEPRECATED__soma_cube_rl_bridge__srv__RLStep_Request __declspec(deprecated)
#endif

namespace soma_cube_rl_bridge
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RLStep_Request_
{
  using Type = RLStep_Request_<ContainerAllocator>;

  explicit RLStep_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit RLStep_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _action_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _action_type action;

  // setters for named parameter idiom
  Type & set__action(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->action = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__soma_cube_rl_bridge__srv__RLStep_Request
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__soma_cube_rl_bridge__srv__RLStep_Request
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RLStep_Request_ & other) const
  {
    if (this->action != other.action) {
      return false;
    }
    return true;
  }
  bool operator!=(const RLStep_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RLStep_Request_

// alias to use template instance with default allocator
using RLStep_Request =
  soma_cube_rl_bridge::srv::RLStep_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace soma_cube_rl_bridge


#ifndef _WIN32
# define DEPRECATED__soma_cube_rl_bridge__srv__RLStep_Response __attribute__((deprecated))
#else
# define DEPRECATED__soma_cube_rl_bridge__srv__RLStep_Response __declspec(deprecated)
#endif

namespace soma_cube_rl_bridge
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RLStep_Response_
{
  using Type = RLStep_Response_<ContainerAllocator>;

  explicit RLStep_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reward = 0.0;
      this->done = false;
      this->info = "";
      this->success = false;
    }
  }

  explicit RLStep_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->reward = 0.0;
      this->done = false;
      this->info = "";
      this->success = false;
    }
  }

  // field types and members
  using _obs_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _obs_type obs;
  using _reward_type =
    double;
  _reward_type reward;
  using _done_type =
    bool;
  _done_type done;
  using _info_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _info_type info;
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__obs(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->obs = _arg;
    return *this;
  }
  Type & set__reward(
    const double & _arg)
  {
    this->reward = _arg;
    return *this;
  }
  Type & set__done(
    const bool & _arg)
  {
    this->done = _arg;
    return *this;
  }
  Type & set__info(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__soma_cube_rl_bridge__srv__RLStep_Response
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__soma_cube_rl_bridge__srv__RLStep_Response
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RLStep_Response_ & other) const
  {
    if (this->obs != other.obs) {
      return false;
    }
    if (this->reward != other.reward) {
      return false;
    }
    if (this->done != other.done) {
      return false;
    }
    if (this->info != other.info) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const RLStep_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RLStep_Response_

// alias to use template instance with default allocator
using RLStep_Response =
  soma_cube_rl_bridge::srv::RLStep_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace soma_cube_rl_bridge

namespace soma_cube_rl_bridge
{

namespace srv
{

struct RLStep
{
  using Request = soma_cube_rl_bridge::srv::RLStep_Request;
  using Response = soma_cube_rl_bridge::srv::RLStep_Response;
};

}  // namespace srv

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__STRUCT_HPP_
