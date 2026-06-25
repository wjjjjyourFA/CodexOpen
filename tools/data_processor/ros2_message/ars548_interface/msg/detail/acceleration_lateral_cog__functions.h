// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ars548_interface:msg/AccelerationLateralCog.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__FUNCTIONS_H_
#define ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ars548_interface/msg/rosidl_generator_c__visibility_control.h"

#include "ars548_interface/msg/detail/acceleration_lateral_cog__struct.h"

/// Initialize msg/AccelerationLateralCog message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ars548_interface__msg__AccelerationLateralCog
 * )) before or use
 * ars548_interface__msg__AccelerationLateralCog__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__AccelerationLateralCog__init(ars548_interface__msg__AccelerationLateralCog * msg);

/// Finalize msg/AccelerationLateralCog message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__AccelerationLateralCog__fini(ars548_interface__msg__AccelerationLateralCog * msg);

/// Create msg/AccelerationLateralCog message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ars548_interface__msg__AccelerationLateralCog__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
ars548_interface__msg__AccelerationLateralCog *
ars548_interface__msg__AccelerationLateralCog__create();

/// Destroy msg/AccelerationLateralCog message.
/**
 * It calls
 * ars548_interface__msg__AccelerationLateralCog__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__AccelerationLateralCog__destroy(ars548_interface__msg__AccelerationLateralCog * msg);

/// Check for msg/AccelerationLateralCog message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__AccelerationLateralCog__are_equal(const ars548_interface__msg__AccelerationLateralCog * lhs, const ars548_interface__msg__AccelerationLateralCog * rhs);

/// Copy a msg/AccelerationLateralCog message.
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
ars548_interface__msg__AccelerationLateralCog__copy(
  const ars548_interface__msg__AccelerationLateralCog * input,
  ars548_interface__msg__AccelerationLateralCog * output);

/// Initialize array of msg/AccelerationLateralCog messages.
/**
 * It allocates the memory for the number of elements and calls
 * ars548_interface__msg__AccelerationLateralCog__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__AccelerationLateralCog__Sequence__init(ars548_interface__msg__AccelerationLateralCog__Sequence * array, size_t size);

/// Finalize array of msg/AccelerationLateralCog messages.
/**
 * It calls
 * ars548_interface__msg__AccelerationLateralCog__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__AccelerationLateralCog__Sequence__fini(ars548_interface__msg__AccelerationLateralCog__Sequence * array);

/// Create array of msg/AccelerationLateralCog messages.
/**
 * It allocates the memory for the array and calls
 * ars548_interface__msg__AccelerationLateralCog__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
ars548_interface__msg__AccelerationLateralCog__Sequence *
ars548_interface__msg__AccelerationLateralCog__Sequence__create(size_t size);

/// Destroy array of msg/AccelerationLateralCog messages.
/**
 * It calls
 * ars548_interface__msg__AccelerationLateralCog__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__AccelerationLateralCog__Sequence__destroy(ars548_interface__msg__AccelerationLateralCog__Sequence * array);

/// Check for msg/AccelerationLateralCog message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__AccelerationLateralCog__Sequence__are_equal(const ars548_interface__msg__AccelerationLateralCog__Sequence * lhs, const ars548_interface__msg__AccelerationLateralCog__Sequence * rhs);

/// Copy an array of msg/AccelerationLateralCog messages.
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
ars548_interface__msg__AccelerationLateralCog__Sequence__copy(
  const ars548_interface__msg__AccelerationLateralCog__Sequence * input,
  ars548_interface__msg__AccelerationLateralCog__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__FUNCTIONS_H_
