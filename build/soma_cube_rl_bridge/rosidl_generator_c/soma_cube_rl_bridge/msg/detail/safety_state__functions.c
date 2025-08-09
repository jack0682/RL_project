// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from soma_cube_rl_bridge:msg/SafetyState.idl
// generated code does not contain a copyright notice
#include "soma_cube_rl_bridge/msg/detail/safety_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `last_event_time`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `joint_limit_violations`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
soma_cube_rl_bridge__msg__SafetyState__init(soma_cube_rl_bridge__msg__SafetyState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    soma_cube_rl_bridge__msg__SafetyState__fini(msg);
    return false;
  }
  // safe_to_move
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    soma_cube_rl_bridge__msg__SafetyState__fini(msg);
    return false;
  }
  // last_event_time
  if (!builtin_interfaces__msg__Time__init(&msg->last_event_time)) {
    soma_cube_rl_bridge__msg__SafetyState__fini(msg);
    return false;
  }
  // robot_mode
  // robot_state
  // emergency_stop
  // protective_stop
  // joint_limit_violations
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->joint_limit_violations, 0)) {
    soma_cube_rl_bridge__msg__SafetyState__fini(msg);
    return false;
  }
  return true;
}

void
soma_cube_rl_bridge__msg__SafetyState__fini(soma_cube_rl_bridge__msg__SafetyState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // safe_to_move
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // last_event_time
  builtin_interfaces__msg__Time__fini(&msg->last_event_time);
  // robot_mode
  // robot_state
  // emergency_stop
  // protective_stop
  // joint_limit_violations
  rosidl_runtime_c__boolean__Sequence__fini(&msg->joint_limit_violations);
}

bool
soma_cube_rl_bridge__msg__SafetyState__are_equal(const soma_cube_rl_bridge__msg__SafetyState * lhs, const soma_cube_rl_bridge__msg__SafetyState * rhs)
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
  // safe_to_move
  if (lhs->safe_to_move != rhs->safe_to_move) {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  // last_event_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->last_event_time), &(rhs->last_event_time)))
  {
    return false;
  }
  // robot_mode
  if (lhs->robot_mode != rhs->robot_mode) {
    return false;
  }
  // robot_state
  if (lhs->robot_state != rhs->robot_state) {
    return false;
  }
  // emergency_stop
  if (lhs->emergency_stop != rhs->emergency_stop) {
    return false;
  }
  // protective_stop
  if (lhs->protective_stop != rhs->protective_stop) {
    return false;
  }
  // joint_limit_violations
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->joint_limit_violations), &(rhs->joint_limit_violations)))
  {
    return false;
  }
  return true;
}

bool
soma_cube_rl_bridge__msg__SafetyState__copy(
  const soma_cube_rl_bridge__msg__SafetyState * input,
  soma_cube_rl_bridge__msg__SafetyState * output)
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
  // safe_to_move
  output->safe_to_move = input->safe_to_move;
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  // last_event_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->last_event_time), &(output->last_event_time)))
  {
    return false;
  }
  // robot_mode
  output->robot_mode = input->robot_mode;
  // robot_state
  output->robot_state = input->robot_state;
  // emergency_stop
  output->emergency_stop = input->emergency_stop;
  // protective_stop
  output->protective_stop = input->protective_stop;
  // joint_limit_violations
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->joint_limit_violations), &(output->joint_limit_violations)))
  {
    return false;
  }
  return true;
}

soma_cube_rl_bridge__msg__SafetyState *
soma_cube_rl_bridge__msg__SafetyState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__msg__SafetyState * msg = (soma_cube_rl_bridge__msg__SafetyState *)allocator.allocate(sizeof(soma_cube_rl_bridge__msg__SafetyState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(soma_cube_rl_bridge__msg__SafetyState));
  bool success = soma_cube_rl_bridge__msg__SafetyState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
soma_cube_rl_bridge__msg__SafetyState__destroy(soma_cube_rl_bridge__msg__SafetyState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    soma_cube_rl_bridge__msg__SafetyState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
soma_cube_rl_bridge__msg__SafetyState__Sequence__init(soma_cube_rl_bridge__msg__SafetyState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__msg__SafetyState * data = NULL;

  if (size) {
    data = (soma_cube_rl_bridge__msg__SafetyState *)allocator.zero_allocate(size, sizeof(soma_cube_rl_bridge__msg__SafetyState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = soma_cube_rl_bridge__msg__SafetyState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        soma_cube_rl_bridge__msg__SafetyState__fini(&data[i - 1]);
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
soma_cube_rl_bridge__msg__SafetyState__Sequence__fini(soma_cube_rl_bridge__msg__SafetyState__Sequence * array)
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
      soma_cube_rl_bridge__msg__SafetyState__fini(&array->data[i]);
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

soma_cube_rl_bridge__msg__SafetyState__Sequence *
soma_cube_rl_bridge__msg__SafetyState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__msg__SafetyState__Sequence * array = (soma_cube_rl_bridge__msg__SafetyState__Sequence *)allocator.allocate(sizeof(soma_cube_rl_bridge__msg__SafetyState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = soma_cube_rl_bridge__msg__SafetyState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
soma_cube_rl_bridge__msg__SafetyState__Sequence__destroy(soma_cube_rl_bridge__msg__SafetyState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    soma_cube_rl_bridge__msg__SafetyState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
soma_cube_rl_bridge__msg__SafetyState__Sequence__are_equal(const soma_cube_rl_bridge__msg__SafetyState__Sequence * lhs, const soma_cube_rl_bridge__msg__SafetyState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!soma_cube_rl_bridge__msg__SafetyState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
soma_cube_rl_bridge__msg__SafetyState__Sequence__copy(
  const soma_cube_rl_bridge__msg__SafetyState__Sequence * input,
  soma_cube_rl_bridge__msg__SafetyState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(soma_cube_rl_bridge__msg__SafetyState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    soma_cube_rl_bridge__msg__SafetyState * data =
      (soma_cube_rl_bridge__msg__SafetyState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!soma_cube_rl_bridge__msg__SafetyState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          soma_cube_rl_bridge__msg__SafetyState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!soma_cube_rl_bridge__msg__SafetyState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
