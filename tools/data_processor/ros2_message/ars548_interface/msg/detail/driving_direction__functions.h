// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ars548_interface:msg/DrivingDirection.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__FUNCTIONS_H_
#define ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ars548_interface/msg/rosidl_generator_c__visibility_control.h"

#include "ars548_interface/msg/detail/driving_direction__struct.h"

/// Initialize msg/DrivingDirection message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ars548_interface__msg__DrivingDirection
 * )) before or use
 * ars548_interface__msg__DrivingDirection__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__DrivingDirection__init(ars548_interface__msg__DrivingDirection * msg);

/// Finalize msg/DrivingDirection message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__DrivingDirection__fini(ars548_interface__msg__DrivingDirection * msg);

/// Create msg/DrivingDirection message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ars548_interface__msg__DrivingDirection__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
ars548_interface__msg__DrivingDirection *
ars548_interface__msg__DrivingDirection__create();

/// Destroy msg/DrivingDirection message.
/**
 * It calls
 * ars548_interface__msg__DrivingDirection__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__DrivingDirection__destroy(ars548_interface__msg__DrivingDirection * msg);

/// Check for msg/DrivingDirection message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__DrivingDirection__are_equal(const ars548_interface__msg__DrivingDirection * lhs, const ars548_interface__msg__DrivingDirection * rhs);

/// Copy a msg/DrivingDirection message.
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
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__DrivingDirection__copy(
  const ars548_interface__msg__DrivingDirection * input,
  ars548_interface__msg__DrivingDirection * output);

/// Initialize array of msg/DrivingDirection messages.
/**
 * It allocates the memory for the number of elements and calls
 * ars548_interface__msg__DrivingDirection__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__DrivingDirection__Sequence__init(ars548_interface__msg__DrivingDirection__Sequence * array, size_t size);

/// Finalize array of msg/DrivingDirection messages.
/**
 * It calls
 * ars548_interface__msg__DrivingDirection__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__DrivingDirection__Sequence__fini(ars548_interface__msg__DrivingDirection__Sequence * array);

/// Create array of msg/DrivingDirection messages.
/**
 * It allocates the memory for the array and calls
 * ars548_interface__msg__DrivingDirection__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
ars548_interface__msg__DrivingDirection__Sequence *
ars548_interface__msg__DrivingDirection__Sequence__create(size_t size);

/// Destroy array of msg/DrivingDirection messages.
/**
 * It calls
 * ars548_interface__msg__DrivingDirection__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__DrivingDirection__Sequence__destroy(ars548_interface__msg__DrivingDirection__Sequence * array);

/// Check for msg/DrivingDirection message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__DrivingDirection__Sequence__are_equal(const ars548_interface__msg__DrivingDirection__Sequence * lhs, const ars548_interface__msg__DrivingDirection__Sequence * rhs);

/// Copy an array of msg/DrivingDirection messages.
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
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__DrivingDirection__Sequence__copy(
  const ars548_interface__msg__DrivingDirection__Sequence * input,
  ars548_interface__msg__DrivingDirection__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__FUNCTIONS_H_
