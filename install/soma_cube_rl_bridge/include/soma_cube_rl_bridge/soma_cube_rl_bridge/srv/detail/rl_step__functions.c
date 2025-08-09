// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from soma_cube_rl_bridge:srv/RLStep.idl
// generated code does not contain a copyright notice
#include "soma_cube_rl_bridge/srv/detail/rl_step__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `action`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
soma_cube_rl_bridge__srv__RLStep_Request__init(soma_cube_rl_bridge__srv__RLStep_Request * msg)
{
  if (!msg) {
    return false;
  }
  // action
  if (!rosidl_runtime_c__double__Sequence__init(&msg->action, 0)) {
    soma_cube_rl_bridge__srv__RLStep_Request__fini(msg);
    return false;
  }
  return true;
}

void
soma_cube_rl_bridge__srv__RLStep_Request__fini(soma_cube_rl_bridge__srv__RLStep_Request * msg)
{
  if (!msg) {
    return;
  }
  // action
  rosidl_runtime_c__double__Sequence__fini(&msg->action);
}

bool
soma_cube_rl_bridge__srv__RLStep_Request__are_equal(const soma_cube_rl_bridge__srv__RLStep_Request * lhs, const soma_cube_rl_bridge__srv__RLStep_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // action
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->action), &(rhs->action)))
  {
    return false;
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__RLStep_Request__copy(
  const soma_cube_rl_bridge__srv__RLStep_Request * input,
  soma_cube_rl_bridge__srv__RLStep_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // action
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->action), &(output->action)))
  {
    return false;
  }
  return true;
}

soma_cube_rl_bridge__srv__RLStep_Request *
soma_cube_rl_bridge__srv__RLStep_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLStep_Request * msg = (soma_cube_rl_bridge__srv__RLStep_Request *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__RLStep_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(soma_cube_rl_bridge__srv__RLStep_Request));
  bool success = soma_cube_rl_bridge__srv__RLStep_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
soma_cube_rl_bridge__srv__RLStep_Request__destroy(soma_cube_rl_bridge__srv__RLStep_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    soma_cube_rl_bridge__srv__RLStep_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
soma_cube_rl_bridge__srv__RLStep_Request__Sequence__init(soma_cube_rl_bridge__srv__RLStep_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLStep_Request * data = NULL;

  if (size) {
    data = (soma_cube_rl_bridge__srv__RLStep_Request *)allocator.zero_allocate(size, sizeof(soma_cube_rl_bridge__srv__RLStep_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = soma_cube_rl_bridge__srv__RLStep_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        soma_cube_rl_bridge__srv__RLStep_Request__fini(&data[i - 1]);
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
soma_cube_rl_bridge__srv__RLStep_Request__Sequence__fini(soma_cube_rl_bridge__srv__RLStep_Request__Sequence * array)
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
      soma_cube_rl_bridge__srv__RLStep_Request__fini(&array->data[i]);
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

soma_cube_rl_bridge__srv__RLStep_Request__Sequence *
soma_cube_rl_bridge__srv__RLStep_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLStep_Request__Sequence * array = (soma_cube_rl_bridge__srv__RLStep_Request__Sequence *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__RLStep_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = soma_cube_rl_bridge__srv__RLStep_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
soma_cube_rl_bridge__srv__RLStep_Request__Sequence__destroy(soma_cube_rl_bridge__srv__RLStep_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    soma_cube_rl_bridge__srv__RLStep_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
soma_cube_rl_bridge__srv__RLStep_Request__Sequence__are_equal(const soma_cube_rl_bridge__srv__RLStep_Request__Sequence * lhs, const soma_cube_rl_bridge__srv__RLStep_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!soma_cube_rl_bridge__srv__RLStep_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__RLStep_Request__Sequence__copy(
  const soma_cube_rl_bridge__srv__RLStep_Request__Sequence * input,
  soma_cube_rl_bridge__srv__RLStep_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(soma_cube_rl_bridge__srv__RLStep_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    soma_cube_rl_bridge__srv__RLStep_Request * data =
      (soma_cube_rl_bridge__srv__RLStep_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!soma_cube_rl_bridge__srv__RLStep_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          soma_cube_rl_bridge__srv__RLStep_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!soma_cube_rl_bridge__srv__RLStep_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `obs`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `info`
#include "rosidl_runtime_c/string_functions.h"

bool
soma_cube_rl_bridge__srv__RLStep_Response__init(soma_cube_rl_bridge__srv__RLStep_Response * msg)
{
  if (!msg) {
    return false;
  }
  // obs
  if (!rosidl_runtime_c__double__Sequence__init(&msg->obs, 0)) {
    soma_cube_rl_bridge__srv__RLStep_Response__fini(msg);
    return false;
  }
  // reward
  // done
  // info
  if (!rosidl_runtime_c__String__init(&msg->info)) {
    soma_cube_rl_bridge__srv__RLStep_Response__fini(msg);
    return false;
  }
  // success
  return true;
}

void
soma_cube_rl_bridge__srv__RLStep_Response__fini(soma_cube_rl_bridge__srv__RLStep_Response * msg)
{
  if (!msg) {
    return;
  }
  // obs
  rosidl_runtime_c__double__Sequence__fini(&msg->obs);
  // reward
  // done
  // info
  rosidl_runtime_c__String__fini(&msg->info);
  // success
}

bool
soma_cube_rl_bridge__srv__RLStep_Response__are_equal(const soma_cube_rl_bridge__srv__RLStep_Response * lhs, const soma_cube_rl_bridge__srv__RLStep_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // obs
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->obs), &(rhs->obs)))
  {
    return false;
  }
  // reward
  if (lhs->reward != rhs->reward) {
    return false;
  }
  // done
  if (lhs->done != rhs->done) {
    return false;
  }
  // info
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__RLStep_Response__copy(
  const soma_cube_rl_bridge__srv__RLStep_Response * input,
  soma_cube_rl_bridge__srv__RLStep_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // obs
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->obs), &(output->obs)))
  {
    return false;
  }
  // reward
  output->reward = input->reward;
  // done
  output->done = input->done;
  // info
  if (!rosidl_runtime_c__String__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

soma_cube_rl_bridge__srv__RLStep_Response *
soma_cube_rl_bridge__srv__RLStep_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLStep_Response * msg = (soma_cube_rl_bridge__srv__RLStep_Response *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__RLStep_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(soma_cube_rl_bridge__srv__RLStep_Response));
  bool success = soma_cube_rl_bridge__srv__RLStep_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
soma_cube_rl_bridge__srv__RLStep_Response__destroy(soma_cube_rl_bridge__srv__RLStep_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    soma_cube_rl_bridge__srv__RLStep_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
soma_cube_rl_bridge__srv__RLStep_Response__Sequence__init(soma_cube_rl_bridge__srv__RLStep_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLStep_Response * data = NULL;

  if (size) {
    data = (soma_cube_rl_bridge__srv__RLStep_Response *)allocator.zero_allocate(size, sizeof(soma_cube_rl_bridge__srv__RLStep_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = soma_cube_rl_bridge__srv__RLStep_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        soma_cube_rl_bridge__srv__RLStep_Response__fini(&data[i - 1]);
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
soma_cube_rl_bridge__srv__RLStep_Response__Sequence__fini(soma_cube_rl_bridge__srv__RLStep_Response__Sequence * array)
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
      soma_cube_rl_bridge__srv__RLStep_Response__fini(&array->data[i]);
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

soma_cube_rl_bridge__srv__RLStep_Response__Sequence *
soma_cube_rl_bridge__srv__RLStep_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  soma_cube_rl_bridge__srv__RLStep_Response__Sequence * array = (soma_cube_rl_bridge__srv__RLStep_Response__Sequence *)allocator.allocate(sizeof(soma_cube_rl_bridge__srv__RLStep_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = soma_cube_rl_bridge__srv__RLStep_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
soma_cube_rl_bridge__srv__RLStep_Response__Sequence__destroy(soma_cube_rl_bridge__srv__RLStep_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    soma_cube_rl_bridge__srv__RLStep_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
soma_cube_rl_bridge__srv__RLStep_Response__Sequence__are_equal(const soma_cube_rl_bridge__srv__RLStep_Response__Sequence * lhs, const soma_cube_rl_bridge__srv__RLStep_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!soma_cube_rl_bridge__srv__RLStep_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
soma_cube_rl_bridge__srv__RLStep_Response__Sequence__copy(
  const soma_cube_rl_bridge__srv__RLStep_Response__Sequence * input,
  soma_cube_rl_bridge__srv__RLStep_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(soma_cube_rl_bridge__srv__RLStep_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    soma_cube_rl_bridge__srv__RLStep_Response * data =
      (soma_cube_rl_bridge__srv__RLStep_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!soma_cube_rl_bridge__srv__RLStep_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          soma_cube_rl_bridge__srv__RLStep_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!soma_cube_rl_bridge__srv__RLStep_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
