// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
// generated code does not contain a copyright notice
#include "soma_cube_rl_bridge/srv/detail/get_safety_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__init(soma_cube_rl_bridge__srv__GetSafetyState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
soma_cube_rl_bridge__srv__GetSafetyState_Request__fini(soma_cube_rl_bridge__srv__GetSafetyState_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__are_equal(const soma_cube_rl_bridge__srv__GetSafetyState_Request * lhs, const soma_cube_rl_bridge__srv__GetSafetyState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__copy(
  const soma_cube_rl_bridge__srv__GetSafetyState_Request * input,
  soma_cube_rl_bridge__srv__GetSafetyState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

soma_cube_rl_bridge__srv__GetSafetyState_Request *
soma_cube_rl_bridge__srv__GetSafetyState_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__GetSafetyState_Request * msg = (soma_cube_rl_bridge__srv__GetSafetyState_Request *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Request));
  bool success = soma_cube_rl_bridge__srv__GetSafetyState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
soma_cube_rl_bridge__srv__GetSafetyState_Request__destroy(soma_cube_rl_bridge__srv__GetSafetyState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    soma_cube_rl_bridge__srv__GetSafetyState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__init(soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__GetSafetyState_Request * data = NULL;

  if (size) {
    data = (soma_cube_rl_bridge__srv__GetSafetyState_Request *)allocator.zero_allocate(size, sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = soma_cube_rl_bridge__srv__GetSafetyState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        soma_cube_rl_bridge__srv__GetSafetyState_Request__fini(&data[i - 1]);
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
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__fini(soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * array)
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
      soma_cube_rl_bridge__srv__GetSafetyState_Request__fini(&array->data[i]);
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

soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence *
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * array = (soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__destroy(soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__are_equal(const soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * lhs, const soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!soma_cube_rl_bridge__srv__GetSafetyState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__copy(
  const soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * input,
  soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    soma_cube_rl_bridge__srv__GetSafetyState_Request * data =
      (soma_cube_rl_bridge__srv__GetSafetyState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!soma_cube_rl_bridge__srv__GetSafetyState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          soma_cube_rl_bridge__srv__GetSafetyState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!soma_cube_rl_bridge__srv__GetSafetyState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"
// Member `last_event_time`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__init(soma_cube_rl_bridge__srv__GetSafetyState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // safe_to_move
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(msg);
    return false;
  }
  // last_event_time
  if (!builtin_interfaces__msg__Time__init(&msg->last_event_time)) {
    soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(msg);
    return false;
  }
  return true;
}

void
soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(soma_cube_rl_bridge__srv__GetSafetyState_Response * msg)
{
  if (!msg) {
    return;
  }
  // safe_to_move
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // last_event_time
  builtin_interfaces__msg__Time__fini(&msg->last_event_time);
}

bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__are_equal(const soma_cube_rl_bridge__srv__GetSafetyState_Response * lhs, const soma_cube_rl_bridge__srv__GetSafetyState_Response * rhs)
{
  if (!lhs || !rhs) {
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
  return true;
}

bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__copy(
  const soma_cube_rl_bridge__srv__GetSafetyState_Response * input,
  soma_cube_rl_bridge__srv__GetSafetyState_Response * output)
{
  if (!input || !output) {
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
  return true;
}

soma_cube_rl_bridge__srv__GetSafetyState_Response *
soma_cube_rl_bridge__srv__GetSafetyState_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__GetSafetyState_Response * msg = (soma_cube_rl_bridge__srv__GetSafetyState_Response *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Response));
  bool success = soma_cube_rl_bridge__srv__GetSafetyState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
soma_cube_rl_bridge__srv__GetSafetyState_Response__destroy(soma_cube_rl_bridge__srv__GetSafetyState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__init(soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__GetSafetyState_Response * data = NULL;

  if (size) {
    data = (soma_cube_rl_bridge__srv__GetSafetyState_Response *)allocator.zero_allocate(size, sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = soma_cube_rl_bridge__srv__GetSafetyState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(&data[i - 1]);
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
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__fini(soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * array)
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
      soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(&array->data[i]);
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

soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence *
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * array = (soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__destroy(soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__are_equal(const soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * lhs, const soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!soma_cube_rl_bridge__srv__GetSafetyState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__copy(
  const soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * input,
  soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(soma_cube_rl_bridge__srv__GetSafetyState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    soma_cube_rl_bridge__srv__GetSafetyState_Response * data =
      (soma_cube_rl_bridge__srv__GetSafetyState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!soma_cube_rl_bridge__srv__GetSafetyState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!soma_cube_rl_bridge__srv__GetSafetyState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
