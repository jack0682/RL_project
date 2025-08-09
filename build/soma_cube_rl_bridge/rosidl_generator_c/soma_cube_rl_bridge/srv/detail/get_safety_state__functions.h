// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__FUNCTIONS_H_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "soma_cube_rl_bridge/msg/rosidl_generator_c__visibility_control.h"

#include "soma_cube_rl_bridge/srv/detail/get_safety_state__struct.h"

/// Initialize srv/GetSafetyState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * soma_cube_rl_bridge__srv__GetSafetyState_Request
 * )) before or use
 * soma_cube_rl_bridge__srv__GetSafetyState_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__init(soma_cube_rl_bridge__srv__GetSafetyState_Request * msg);

/// Finalize srv/GetSafetyState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__srv__GetSafetyState_Request__fini(soma_cube_rl_bridge__srv__GetSafetyState_Request * msg);

/// Create srv/GetSafetyState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
soma_cube_rl_bridge__srv__GetSafetyState_Request *
soma_cube_rl_bridge__srv__GetSafetyState_Request__create();

/// Destroy srv/GetSafetyState message.
/**
 * It calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__srv__GetSafetyState_Request__destroy(soma_cube_rl_bridge__srv__GetSafetyState_Request * msg);

/// Check for srv/GetSafetyState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__are_equal(const soma_cube_rl_bridge__srv__GetSafetyState_Request * lhs, const soma_cube_rl_bridge__srv__GetSafetyState_Request * rhs);

/// Copy a srv/GetSafetyState message.
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
soma_cube_rl_bridge__srv__GetSafetyState_Request__copy(
  const soma_cube_rl_bridge__srv__GetSafetyState_Request * input,
  soma_cube_rl_bridge__srv__GetSafetyState_Request * output);

/// Initialize array of srv/GetSafetyState messages.
/**
 * It allocates the memory for the number of elements and calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__init(soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetSafetyState messages.
/**
 * It calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__fini(soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * array);

/// Create array of srv/GetSafetyState messages.
/**
 * It allocates the memory for the array and calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence *
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetSafetyState messages.
/**
 * It calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__destroy(soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * array);

/// Check for srv/GetSafetyState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__are_equal(const soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * lhs, const soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * rhs);

/// Copy an array of srv/GetSafetyState messages.
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
soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence__copy(
  const soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * input,
  soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence * output);

/// Initialize srv/GetSafetyState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * soma_cube_rl_bridge__srv__GetSafetyState_Response
 * )) before or use
 * soma_cube_rl_bridge__srv__GetSafetyState_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__init(soma_cube_rl_bridge__srv__GetSafetyState_Response * msg);

/// Finalize srv/GetSafetyState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__srv__GetSafetyState_Response__fini(soma_cube_rl_bridge__srv__GetSafetyState_Response * msg);

/// Create srv/GetSafetyState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
soma_cube_rl_bridge__srv__GetSafetyState_Response *
soma_cube_rl_bridge__srv__GetSafetyState_Response__create();

/// Destroy srv/GetSafetyState message.
/**
 * It calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__srv__GetSafetyState_Response__destroy(soma_cube_rl_bridge__srv__GetSafetyState_Response * msg);

/// Check for srv/GetSafetyState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__are_equal(const soma_cube_rl_bridge__srv__GetSafetyState_Response * lhs, const soma_cube_rl_bridge__srv__GetSafetyState_Response * rhs);

/// Copy a srv/GetSafetyState message.
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
soma_cube_rl_bridge__srv__GetSafetyState_Response__copy(
  const soma_cube_rl_bridge__srv__GetSafetyState_Response * input,
  soma_cube_rl_bridge__srv__GetSafetyState_Response * output);

/// Initialize array of srv/GetSafetyState messages.
/**
 * It allocates the memory for the number of elements and calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__init(soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetSafetyState messages.
/**
 * It calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__fini(soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * array);

/// Create array of srv/GetSafetyState messages.
/**
 * It allocates the memory for the array and calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence *
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetSafetyState messages.
/**
 * It calls
 * soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
void
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__destroy(soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * array);

/// Check for srv/GetSafetyState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_soma_cube_rl_bridge
bool
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__are_equal(const soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * lhs, const soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * rhs);

/// Copy an array of srv/GetSafetyState messages.
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
soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence__copy(
  const soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * input,
  soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__FUNCTIONS_H_
