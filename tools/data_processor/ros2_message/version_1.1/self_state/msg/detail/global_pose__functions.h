// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from self_state:msg/GlobalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__GLOBAL_POSE__FUNCTIONS_H_
#define SELF_STATE__MSG__DETAIL__GLOBAL_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "self_state/msg/rosidl_generator_c__visibility_control.h"

#include "self_state/msg/detail/global_pose__struct.h"

/// Initialize msg/GlobalPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * self_state__msg__GlobalPose
 * )) before or use
 * self_state__msg__GlobalPose__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__GlobalPose__init(self_state__msg__GlobalPose * msg);

/// Finalize msg/GlobalPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
void
self_state__msg__GlobalPose__fini(self_state__msg__GlobalPose * msg);

/// Create msg/GlobalPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * self_state__msg__GlobalPose__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
self_state__msg__GlobalPose *
self_state__msg__GlobalPose__create();

/// Destroy msg/GlobalPose message.
/**
 * It calls
 * self_state__msg__GlobalPose__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
void
self_state__msg__GlobalPose__destroy(self_state__msg__GlobalPose * msg);

/// Check for msg/GlobalPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__GlobalPose__are_equal(const self_state__msg__GlobalPose * lhs, const self_state__msg__GlobalPose * rhs);

/// Copy a msg/GlobalPose message.
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
self_state__msg__GlobalPose__copy(
  const self_state__msg__GlobalPose * input,
  self_state__msg__GlobalPose * output);

/// Initialize array of msg/GlobalPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * self_state__msg__GlobalPose__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__GlobalPose__Sequence__init(self_state__msg__GlobalPose__Sequence * array, size_t size);

/// Finalize array of msg/GlobalPose messages.
/**
 * It calls
 * self_state__msg__GlobalPose__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
void
self_state__msg__GlobalPose__Sequence__fini(self_state__msg__GlobalPose__Sequence * array);

/// Create array of msg/GlobalPose messages.
/**
 * It allocates the memory for the array and calls
 * self_state__msg__GlobalPose__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
self_state__msg__GlobalPose__Sequence *
self_state__msg__GlobalPose__Sequence__create(size_t size);

/// Destroy array of msg/GlobalPose messages.
/**
 * It calls
 * self_state__msg__GlobalPose__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
void
self_state__msg__GlobalPose__Sequence__destroy(self_state__msg__GlobalPose__Sequence * array);

/// Check for msg/GlobalPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_self_state
bool
self_state__msg__GlobalPose__Sequence__are_equal(const self_state__msg__GlobalPose__Sequence * lhs, const self_state__msg__GlobalPose__Sequence * rhs);

/// Copy an array of msg/GlobalPose messages.
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
self_state__msg__GlobalPose__Sequence__copy(
  const self_state__msg__GlobalPose__Sequence * input,
  self_state__msg__GlobalPose__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__GLOBAL_POSE__FUNCTIONS_H_
