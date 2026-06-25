// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from self_state:msg/InsStatus.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__INS_STATUS__FUNCTIONS_H_
#define SELF_STATE__MSG__DETAIL__INS_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "self_state/msg/rosidl_generator_c__visibility_control.h"

#include "self_state/msg/detail/ins_status__struct.h"

/// Initialize msg/InsStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * self_state__msg__InsStatus
 * )) before or use
 * self_state__msg__InsStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__InsStatus__init(self_state__msg__InsStatus * msg);

/// Finalize msg/InsStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
void
self_state__msg__InsStatus__fini(self_state__msg__InsStatus * msg);

/// Create msg/InsStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * self_state__msg__InsStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
self_state__msg__InsStatus *
self_state__msg__InsStatus__create();

/// Destroy msg/InsStatus message.
/**
 * It calls
 * self_state__msg__InsStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
void
self_state__msg__InsStatus__destroy(self_state__msg__InsStatus * msg);

/// Check for msg/InsStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__InsStatus__are_equal(const self_state__msg__InsStatus * lhs, const self_state__msg__InsStatus * rhs);

/// Copy a msg/InsStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__InsStatus__copy(
  const self_state__msg__InsStatus * input,
  self_state__msg__InsStatus * output);

/// Initialize array of msg/InsStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * self_state__msg__InsStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__InsStatus__Sequence__init(self_state__msg__InsStatus__Sequence * array, size_t size);

/// Finalize array of msg/InsStatus messages.
/**
 * It calls
 * self_state__msg__InsStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
void
self_state__msg__InsStatus__Sequence__fini(self_state__msg__InsStatus__Sequence * array);

/// Create array of msg/InsStatus messages.
/**
 * It allocates the memory for the array and calls
 * self_state__msg__InsStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
self_state__msg__InsStatus__Sequence *
self_state__msg__InsStatus__Sequence__create(size_t size);

/// Destroy array of msg/InsStatus messages.
/**
 * It calls
 * self_state__msg__InsStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
void
self_state__msg__InsStatus__Sequence__destroy(self_state__msg__InsStatus__Sequence * array);

/// Check for msg/InsStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__InsStatus__Sequence__are_equal(const self_state__msg__InsStatus__Sequence * lhs, const self_state__msg__InsStatus__Sequence * rhs);

/// Copy an array of msg/InsStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__InsStatus__Sequence__copy(
  const self_state__msg__InsStatus__Sequence * input,
  self_state__msg__InsStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__INS_STATUS__FUNCTIONS_H_
