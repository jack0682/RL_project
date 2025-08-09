// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from soma_cube_rl_bridge:msg/RLObservation.idl
// generated code does not contain a copyright notice
#include "soma_cube_rl_bridge/msg/detail/rl_observation__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `joint_positions`
// Member `joint_velocities`
// Member `tool_force`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `tcp_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
soma_cube_rl_bridge__msg__RLObservation__init(soma_cube_rl_bridge__msg__RLObservation * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    soma_cube_rl_bridge__msg__RLObservation__fini(msg);
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_positions, 0)) {
    soma_cube_rl_bridge__msg__RLObservation__fini(msg);
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_velocities, 0)) {
    soma_cube_rl_bridge__msg__RLObservation__fini(msg);
    return false;
  }
  // tcp_pose
  if (!geometry_msgs__msg__Pose__init(&msg->tcp_pose)) {
    soma_cube_rl_bridge__msg__RLObservation__fini(msg);
    return false;
  }
  // tool_force
  if (!rosidl_runtime_c__double__Sequence__init(&msg->tool_force, 0)) {
    soma_cube_rl_bridge__msg__RLObservation__fini(msg);
    return false;
  }
  // gripper_status
  // robot_state
  // task_success
  // contact_detected
  return true;
}

void
soma_cube_rl_bridge__msg__RLObservation__fini(soma_cube_rl_bridge__msg__RLObservation * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // joint_positions
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_positions);
  // joint_velocities
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_velocities);
  // tcp_pose
  geometry_msgs__msg__Pose__fini(&msg->tcp_pose);
  // tool_force
  rosidl_runtime_c__double__Sequence__fini(&msg->tool_force);
  // gripper_status
  // robot_state
  // task_success
  // contact_detected
}

bool
soma_cube_rl_bridge__msg__RLObservation__are_equal(const soma_cube_rl_bridge__msg__RLObservation * lhs, const soma_cube_rl_bridge__msg__RLObservation * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_positions), &(rhs->joint_positions)))
  {
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_velocities), &(rhs->joint_velocities)))
  {
    return false;
  }
  // tcp_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->tcp_pose), &(rhs->tcp_pose)))
  {
    return false;
  }
  // tool_force
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->tool_force), &(rhs->tool_force)))
  {
    return false;
  }
  // gripper_status
  if (lhs->gripper_status != rhs->gripper_status) {
    return false;
  }
  // robot_state
  if (lhs->robot_state != rhs->robot_state) {
    return false;
  }
  // task_success
  if (lhs->task_success != rhs->task_success) {
    return false;
  }
  // contact_detected
  if (lhs->contact_detected != rhs->contact_detected) {
    return false;
  }
  return true;
}

bool
soma_cube_rl_bridge__msg__RLObservation__copy(
  const soma_cube_rl_bridge__msg__RLObservation * input,
  soma_cube_rl_bridge__msg__RLObservation * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_positions), &(output->joint_positions)))
  {
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_velocities), &(output->joint_velocities)))
  {
    return false;
  }
  // tcp_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->tcp_pose), &(output->tcp_pose)))
  {
    return false;
  }
  // tool_force
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->tool_force), &(output->tool_force)))
  {
    return false;
  }
  // gripper_status
  output->gripper_status = input->gripper_status;
  // robot_state
  output->robot_state = input->robot_state;
  // task_success
  output->task_success = input->task_success;
  // contact_detected
  output->contact_detected = input->contact_detected;
  return true;
}

soma_cube_rl_bridge__msg__RLObservation *
soma_cube_rl_bridge__msg__RLObservation__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__msg__RLObservation * msg = (soma_cube_rl_bridge__msg__RLObservation *)allocator.allocate(sizeof(soma_cube_rl_bridge__msg__RLObservation), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(soma_cube_rl_bridge__msg__RLObservation));
  bool success = soma_cube_rl_bridge__msg__RLObservation__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
soma_cube_rl_bridge__msg__RLObservation__destroy(soma_cube_rl_bridge__msg__RLObservation * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    soma_cube_rl_bridge__msg__RLObservation__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
soma_cube_rl_bridge__msg__RLObservation__Sequence__init(soma_cube_rl_bridge__msg__RLObservation__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__msg__RLObservation * data = NULL;

  if (size) {
    data = (soma_cube_rl_bridge__msg__RLObservation *)allocator.zero_allocate(size, sizeof(soma_cube_rl_bridge__msg__RLObservation), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = soma_cube_rl_bridge__msg__RLObservation__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        soma_cube_rl_bridge__msg__RLObservation__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
soma_cube_rl_bridge__msg__RLObservation__Sequence__fini(soma_cube_rl_bridge__msg__RLObservation__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      soma_cube_rl_bridge__msg__RLObservation__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

soma_cube_rl_bridge__msg__RLObservation__Sequence *
soma_cube_rl_bridge__msg__RLObservation__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__msg__RLObservation__Sequence * array = (soma_cube_rl_bridge__msg__RLObservation__Sequence *)allocator.allocate(sizeof(soma_cube_rl_bridge__msg__RLObservation__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = soma_cube_rl_bridge__msg__RLObservation__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
soma_cube_rl_bridge__msg__RLObservation__Sequence__destroy(soma_cube_rl_bridge__msg__RLObservation__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    soma_cube_rl_bridge__msg__RLObservation__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
soma_cube_rl_bridge__msg__RLObservation__Sequence__are_equal(const soma_cube_rl_bridge__msg__RLObservation__Sequence * lhs, const soma_cube_rl_bridge__msg__RLObservation__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!soma_cube_rl_bridge__msg__RLObservation__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
soma_cube_rl_bridge__msg__RLObservation__Sequence__copy(
  const soma_cube_rl_bridge__msg__RLObservation__Sequence * input,
  soma_cube_rl_bridge__msg__RLObservation__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(soma_cube_rl_bridge__msg__RLObservation);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    soma_cube_rl_bridge__msg__RLObservation * data =
      (soma_cube_rl_bridge__msg__RLObservation *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!soma_cube_rl_bridge__msg__RLObservation__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          soma_cube_rl_bridge__msg__RLObservation__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!soma_cube_rl_bridge__msg__RLObservation__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
