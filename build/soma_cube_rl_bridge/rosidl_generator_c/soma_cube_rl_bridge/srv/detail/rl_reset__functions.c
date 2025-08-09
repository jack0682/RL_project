// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from soma_cube_rl_bridge:srv/RLReset.idl
// generated code does not contain a copyright notice
#include "soma_cube_rl_bridge/srv/detail/rl_reset__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
soma_cube_rl_bridge__srv__RLReset_Request__init(soma_cube_rl_bridge__srv__RLReset_Request * msg)
{
  if (!msg) {
    return false;
  }
  // seed
  return true;
}

void
soma_cube_rl_bridge__srv__RLReset_Request__fini(soma_cube_rl_bridge__srv__RLReset_Request * msg)
{
  if (!msg) {
    return;
  }
  // seed
}

bool
soma_cube_rl_bridge__srv__RLReset_Request__are_equal(const soma_cube_rl_bridge__srv__RLReset_Request * lhs, const soma_cube_rl_bridge__srv__RLReset_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // seed
  if (lhs->seed != rhs->seed) {
    return false;
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__RLReset_Request__copy(
  const soma_cube_rl_bridge__srv__RLReset_Request * input,
  soma_cube_rl_bridge__srv__RLReset_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // seed
  output->seed = input->seed;
  return true;
}

soma_cube_rl_bridge__srv__RLReset_Request *
soma_cube_rl_bridge__srv__RLReset_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLReset_Request * msg = (soma_cube_rl_bridge__srv__RLReset_Request *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__RLReset_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(soma_cube_rl_bridge__srv__RLReset_Request));
  bool success = soma_cube_rl_bridge__srv__RLReset_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
soma_cube_rl_bridge__srv__RLReset_Request__destroy(soma_cube_rl_bridge__srv__RLReset_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    soma_cube_rl_bridge__srv__RLReset_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
soma_cube_rl_bridge__srv__RLReset_Request__Sequence__init(soma_cube_rl_bridge__srv__RLReset_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLReset_Request * data = NULL;

  if (size) {
    data = (soma_cube_rl_bridge__srv__RLReset_Request *)allocator.zero_allocate(size, sizeof(soma_cube_rl_bridge__srv__RLReset_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = soma_cube_rl_bridge__srv__RLReset_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        soma_cube_rl_bridge__srv__RLReset_Request__fini(&data[i - 1]);
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
soma_cube_rl_bridge__srv__RLReset_Request__Sequence__fini(soma_cube_rl_bridge__srv__RLReset_Request__Sequence * array)
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
      soma_cube_rl_bridge__srv__RLReset_Request__fini(&array->data[i]);
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

soma_cube_rl_bridge__srv__RLReset_Request__Sequence *
soma_cube_rl_bridge__srv__RLReset_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLReset_Request__Sequence * array = (soma_cube_rl_bridge__srv__RLReset_Request__Sequence *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__RLReset_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = soma_cube_rl_bridge__srv__RLReset_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
soma_cube_rl_bridge__srv__RLReset_Request__Sequence__destroy(soma_cube_rl_bridge__srv__RLReset_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    soma_cube_rl_bridge__srv__RLReset_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
soma_cube_rl_bridge__srv__RLReset_Request__Sequence__are_equal(const soma_cube_rl_bridge__srv__RLReset_Request__Sequence * lhs, const soma_cube_rl_bridge__srv__RLReset_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!soma_cube_rl_bridge__srv__RLReset_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__RLReset_Request__Sequence__copy(
  const soma_cube_rl_bridge__srv__RLReset_Request__Sequence * input,
  soma_cube_rl_bridge__srv__RLReset_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(soma_cube_rl_bridge__srv__RLReset_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    soma_cube_rl_bridge__srv__RLReset_Request * data =
      (soma_cube_rl_bridge__srv__RLReset_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!soma_cube_rl_bridge__srv__RLReset_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          soma_cube_rl_bridge__srv__RLReset_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!soma_cube_rl_bridge__srv__RLReset_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `initial_obs`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
soma_cube_rl_bridge__srv__RLReset_Response__init(soma_cube_rl_bridge__srv__RLReset_Response * msg)
{
  if (!msg) {
    return false;
  }
  // initial_obs
  if (!rosidl_runtime_c__double__Sequence__init(&msg->initial_obs, 0)) {
    soma_cube_rl_bridge__srv__RLReset_Response__fini(msg);
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    soma_cube_rl_bridge__srv__RLReset_Response__fini(msg);
    return false;
  }
  return true;
}

void
soma_cube_rl_bridge__srv__RLReset_Response__fini(soma_cube_rl_bridge__srv__RLReset_Response * msg)
{
  if (!msg) {
    return;
  }
  // initial_obs
  rosidl_runtime_c__double__Sequence__fini(&msg->initial_obs);
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
soma_cube_rl_bridge__srv__RLReset_Response__are_equal(const soma_cube_rl_bridge__srv__RLReset_Response * lhs, const soma_cube_rl_bridge__srv__RLReset_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // initial_obs
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->initial_obs), &(rhs->initial_obs)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__RLReset_Response__copy(
  const soma_cube_rl_bridge__srv__RLReset_Response * input,
  soma_cube_rl_bridge__srv__RLReset_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // initial_obs
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->initial_obs), &(output->initial_obs)))
  {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

soma_cube_rl_bridge__srv__RLReset_Response *
soma_cube_rl_bridge__srv__RLReset_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLReset_Response * msg = (soma_cube_rl_bridge__srv__RLReset_Response *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__RLReset_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(soma_cube_rl_bridge__srv__RLReset_Response));
  bool success = soma_cube_rl_bridge__srv__RLReset_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
soma_cube_rl_bridge__srv__RLReset_Response__destroy(soma_cube_rl_bridge__srv__RLReset_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    soma_cube_rl_bridge__srv__RLReset_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
soma_cube_rl_bridge__srv__RLReset_Response__Sequence__init(soma_cube_rl_bridge__srv__RLReset_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLReset_Response * data = NULL;

  if (size) {
    data = (soma_cube_rl_bridge__srv__RLReset_Response *)allocator.zero_allocate(size, sizeof(soma_cube_rl_bridge__srv__RLReset_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = soma_cube_rl_bridge__srv__RLReset_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        soma_cube_rl_bridge__srv__RLReset_Response__fini(&data[i - 1]);
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
soma_cube_rl_bridge__srv__RLReset_Response__Sequence__fini(soma_cube_rl_bridge__srv__RLReset_Response__Sequence * array)
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
      soma_cube_rl_bridge__srv__RLReset_Response__fini(&array->data[i]);
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

soma_cube_rl_bridge__srv__RLReset_Response__Sequence *
soma_cube_rl_bridge__srv__RLReset_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLReset_Response__Sequence * array = (soma_cube_rl_bridge__srv__RLReset_Response__Sequence *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__RLReset_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = soma_cube_rl_bridge__srv__RLReset_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
soma_cube_rl_bridge__srv__RLReset_Response__Sequence__destroy(soma_cube_rl_bridge__srv__RLReset_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    soma_cube_rl_bridge__srv__RLReset_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
soma_cube_rl_bridge__srv__RLReset_Response__Sequence__are_equal(const soma_cube_rl_bridge__srv__RLReset_Response__Sequence * lhs, const soma_cube_rl_bridge__srv__RLReset_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!soma_cube_rl_bridge__srv__RLReset_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__RLReset_Response__Sequence__copy(
  const soma_cube_rl_bridge__srv__RLReset_Response__Sequence * input,
  soma_cube_rl_bridge__srv__RLReset_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(soma_cube_rl_bridge__srv__RLReset_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    soma_cube_rl_bridge__srv__RLReset_Response * data =
      (soma_cube_rl_bridge__srv__RLReset_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!soma_cube_rl_bridge__srv__RLReset_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          soma_cube_rl_bridge__srv__RLReset_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!soma_cube_rl_bridge__srv__RLReset_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
