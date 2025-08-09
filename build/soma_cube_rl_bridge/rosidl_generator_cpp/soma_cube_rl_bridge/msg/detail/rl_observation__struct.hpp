// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from soma_cube_rl_bridge:msg/RLObservation.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__STRUCT_HPP_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__STRUCT_HPP_

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
// Member 'tcp_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__soma_cube_rl_bridge__msg__RLObservation __attribute__((deprecated))
#else
# define DEPRECATED__soma_cube_rl_bridge__msg__RLObservation __declspec(deprecated)
#endif

namespace soma_cube_rl_bridge
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RLObservation_
{
  using Type = RLObservation_<ContainerAllocator>;

  explicit RLObservation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    tcp_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gripper_status = 0ul;
      this->robot_state = 0ul;
      this->task_success = false;
      this->contact_detected = false;
    }
  }

  explicit RLObservation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    tcp_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->gripper_status = 0ul;
      this->robot_state = 0ul;
      this->task_success = false;
      this->contact_detected = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_positions_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_positions_type joint_positions;
  using _joint_velocities_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_velocities_type joint_velocities;
  using _tcp_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _tcp_pose_type tcp_pose;
  using _tool_force_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _tool_force_type tool_force;
  using _gripper_status_type =
    uint32_t;
  _gripper_status_type gripper_status;
  using _robot_state_type =
    uint32_t;
  _robot_state_type robot_state;
  using _task_success_type =
    bool;
  _task_success_type task_success;
  using _contact_detected_type =
    bool;
  _contact_detected_type contact_detected;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_positions(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_positions = _arg;
    return *this;
  }
  Type & set__joint_velocities(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_velocities = _arg;
    return *this;
  }
  Type & set__tcp_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->tcp_pose = _arg;
    return *this;
  }
  Type & set__tool_force(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->tool_force = _arg;
    return *this;
  }
  Type & set__gripper_status(
    const uint32_t & _arg)
  {
    this->gripper_status = _arg;
    return *this;
  }
  Type & set__robot_state(
    const uint32_t & _arg)
  {
    this->robot_state = _arg;
    return *this;
  }
  Type & set__task_success(
    const bool & _arg)
  {
    this->task_success = _arg;
    return *this;
  }
  Type & set__contact_detected(
    const bool & _arg)
  {
    this->contact_detected = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator> *;
  using ConstRawPtr =
    const soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__soma_cube_rl_bridge__msg__RLObservation
    std::shared_ptr<soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__soma_cube_rl_bridge__msg__RLObservation
    std::shared_ptr<soma_cube_rl_bridge::msg::RLObservation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RLObservation_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_positions != other.joint_positions) {
      return false;
    }
    if (this->joint_velocities != other.joint_velocities) {
      return false;
    }
    if (this->tcp_pose != other.tcp_pose) {
      return false;
    }
    if (this->tool_force != other.tool_force) {
      return false;
    }
    if (this->gripper_status != other.gripper_status) {
      return false;
    }
    if (this->robot_state != other.robot_state) {
      return false;
    }
    if (this->task_success != other.task_success) {
      return false;
    }
    if (this->contact_detected != other.contact_detected) {
      return false;
    }
    return true;
  }
  bool operator!=(const RLObservation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RLObservation_

// alias to use template instance with default allocator
using RLObservation =
  soma_cube_rl_bridge::msg::RLObservation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__STRUCT_HPP_
