// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ars548_interface:msg/SteeringAngleFrontAxle.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__FUNCTIONS_H_
#define ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ars548_interface/msg/rosidl_generator_c__visibility_control.h"

#include "ars548_interface/msg/detail/steering_angle_front_axle__struct.h"

/// Initialize msg/SteeringAngleFrontAxle message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ars548_interface__msg__SteeringAngleFrontAxle
 * )) before or use
 * ars548_interface__msg__SteeringAngleFrontAxle__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__SteeringAngleFrontAxle__init(ars548_interface__msg__SteeringAngleFrontAxle * msg);

/// Finalize msg/SteeringAngleFrontAxle message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__SteeringAngleFrontAxle__fini(ars548_interface__msg__SteeringAngleFrontAxle * msg);

/// Create msg/SteeringAngleFrontAxle message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ars548_interface__msg__SteeringAngleFrontAxle__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
ars548_interface__msg__SteeringAngleFrontAxle *
ars548_interface__msg__SteeringAngleFrontAxle__create();

/// Destroy msg/SteeringAngleFrontAxle message.
/**
 * It calls
 * ars548_interface__msg__SteeringAngleFrontAxle__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__SteeringAngleFrontAxle__destroy(ars548_interface__msg__SteeringAngleFrontAxle * msg);

/// Check for msg/SteeringAngleFrontAxle message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__SteeringAngleFrontAxle__are_equal(const ars548_interface__msg__SteeringAngleFrontAxle * lhs, const ars548_interface__msg__SteeringAngleFrontAxle * rhs);

/// Copy a msg/SteeringAngleFrontAxle message.
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
ars548_interface__msg__SteeringAngleFrontAxle__copy(
  const ars548_interface__msg__SteeringAngleFrontAxle * input,
  ars548_interface__msg__SteeringAngleFrontAxle * output);

/// Initialize array of msg/SteeringAngleFrontAxle messages.
/**
 * It allocates the memory for the number of elements and calls
 * ars548_interface__msg__SteeringAngleFrontAxle__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__SteeringAngleFrontAxle__Sequence__init(ars548_interface__msg__SteeringAngleFrontAxle__Sequence * array, size_t size);

/// Finalize array of msg/SteeringAngleFrontAxle messages.
/**
 * It calls
 * ars548_interface__msg__SteeringAngleFrontAxle__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__SteeringAngleFrontAxle__Sequence__fini(ars548_interface__msg__SteeringAngleFrontAxle__Sequence * array);

/// Create array of msg/SteeringAngleFrontAxle messages.
/**
 * It allocates the memory for the array and calls
 * ars548_interface__msg__SteeringAngleFrontAxle__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
ars548_interface__msg__SteeringAngleFrontAxle__Sequence *
ars548_interface__msg__SteeringAngleFrontAxle__Sequence__create(size_t size);

/// Destroy array of msg/SteeringAngleFrontAxle messages.
/**
 * It calls
 * ars548_interface__msg__SteeringAngleFrontAxle__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__SteeringAngleFrontAxle__Sequence__destroy(ars548_interface__msg__SteeringAngleFrontAxle__Sequence * array);

/// Check for msg/SteeringAngleFrontAxle message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__SteeringAngleFrontAxle__Sequence__are_equal(const ars548_interface__msg__SteeringAngleFrontAxle__Sequence * lhs, const ars548_interface__msg__SteeringAngleFrontAxle__Sequence * rhs);

/// Copy an array of msg/SteeringAngleFrontAxle messages.
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
ars548_interface__msg__SteeringAngleFrontAxle__Sequence__copy(
  const ars548_interface__msg__SteeringAngleFrontAxle__Sequence * input,
  ars548_interface__msg__SteeringAngleFrontAxle__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__FUNCTIONS_H_
