// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from soma_cube_rl_bridge:msg/RLObservation.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__STRUCT_H_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'joint_positions'
// Member 'joint_velocities'
// Member 'tool_force'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'tcp_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/RLObservation in the package soma_cube_rl_bridge.
typedef struct soma_cube_rl_bridge__msg__RLObservation
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__double__Sequence joint_positions;
  rosidl_runtime_c__double__Sequence joint_velocities;
  geometry_msgs__msg__Pose tcp_pose;
  rosidl_runtime_c__double__Sequence tool_force;
  uint32_t gripper_status;
  uint32_t robot_state;
  bool task_success;
  bool contact_detected;
} soma_cube_rl_bridge__msg__RLObservation;

// Struct for a sequence of soma_cube_rl_bridge__msg__RLObservation.
typedef struct soma_cube_rl_bridge__msg__RLObservation__Sequence
{
  soma_cube_rl_bridge__msg__RLObservation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} soma_cube_rl_bridge__msg__RLObservation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__STRUCT_H_
