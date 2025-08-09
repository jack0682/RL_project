// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from soma_cube_rl_bridge:msg/RLObservation.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__FUNCTIONS_H_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "soma_cube_rl_bridge/msg/rosidl_generator_c__visibility_control.h"

#include "soma_cube_rl_bridge/msg/detail/rl_observation__struct.h"

/// Initialize msg/RLObservation message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * soma_cube_rl_bridge__msg__RLObservation
 * )) before or use
 * soma_cube_rl_bridge__msg__RLObservation__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__msg__RLObservation__init(soma_cube_rl_bridge__msg__RLObservation * msg);

/// Finalize msg/RLObservation message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__msg__RLObservation__fini(soma_cube_rl_bridge__msg__RLObservation * msg);

/// Create msg/RLObservation message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * soma_cube_rl_bridge__msg__RLObservation__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
soma_cube_rl_bridge__msg__RLObservation *
soma_cube_rl_bridge__msg__RLObservation__create();

/// Destroy msg/RLObservation message.
/**
 * It calls
 * soma_cube_rl_bridge__msg__RLObservation__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__msg__RLObservation__destroy(soma_cube_rl_bridge__msg__RLObservation * msg);

/// Check for msg/RLObservation message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__msg__RLObservation__are_equal(const soma_cube_rl_bridge__msg__RLObservation * lhs, const soma_cube_rl_bridge__msg__RLObservation * rhs);

/// Copy a msg/RLObservation message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__msg__RLObservation__copy(
  const soma_cube_rl_bridge__msg__RLObservation * input,
  soma_cube_rl_bridge__msg__RLObservation * output);

/// Initialize array of msg/RLObservation messages.
/**
 * It allocates the memory for the number of elements and calls
 * soma_cube_rl_bridge__msg__RLObservation__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__msg__RLObservation__Sequence__init(soma_cube_rl_bridge__msg__RLObservation__Sequence * array, size_t size);

/// Finalize array of msg/RLObservation messages.
/**
 * It calls
 * soma_cube_rl_bridge__msg__RLObservation__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__msg__RLObservation__Sequence__fini(soma_cube_rl_bridge__msg__RLObservation__Sequence * array);

/// Create array of msg/RLObservation messages.
/**
 * It allocates the memory for the array and calls
 * soma_cube_rl_bridge__msg__RLObservation__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
soma_cube_rl_bridge__msg__RLObservation__Sequence *
soma_cube_rl_bridge__msg__RLObservation__Sequence__create(size_t size);

/// Destroy array of msg/RLObservation messages.
/**
 * It calls
 * soma_cube_rl_bridge__msg__RLObservation__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__msg__RLObservation__Sequence__destroy(soma_cube_rl_bridge__msg__RLObservation__Sequence * array);

/// Check for msg/RLObservation message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__msg__RLObservation__Sequence__are_equal(const soma_cube_rl_bridge__msg__RLObservation__Sequence * lhs, const soma_cube_rl_bridge__msg__RLObservation__Sequence * rhs);

/// Copy an array of msg/RLObservation messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__msg__RLObservation__Sequence__copy(
  const soma_cube_rl_bridge__msg__RLObservation__Sequence * input,
  soma_cube_rl_bridge__msg__RLObservation__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__FUNCTIONS_H_
