// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ars548_interface:msg/SensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__FUNCTIONS_H_
#define ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ars548_interface/msg/rosidl_generator_c__visibility_control.h"

#include "ars548_interface/msg/detail/sensor_configuration__struct.h"

/// Initialize msg/SensorConfiguration message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ars548_interface__msg__SensorConfiguration
 * )) before or use
 * ars548_interface__msg__SensorConfiguration__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__SensorConfiguration__init(ars548_interface__msg__SensorConfiguration * msg);

/// Finalize msg/SensorConfiguration message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__SensorConfiguration__fini(ars548_interface__msg__SensorConfiguration * msg);

/// Create msg/SensorConfiguration message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ars548_interface__msg__SensorConfiguration__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
ars548_interface__msg__SensorConfiguration *
ars548_interface__msg__SensorConfiguration__create();

/// Destroy msg/SensorConfiguration message.
/**
 * It calls
 * ars548_interface__msg__SensorConfiguration__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__SensorConfiguration__destroy(ars548_interface__msg__SensorConfiguration * msg);

/// Check for msg/SensorConfiguration message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__SensorConfiguration__are_equal(const ars548_interface__msg__SensorConfiguration * lhs, const ars548_interface__msg__SensorConfiguration * rhs);

/// Copy a msg/SensorConfiguration message.
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
ars548_interface__msg__SensorConfiguration__copy(
  const ars548_interface__msg__SensorConfiguration * input,
  ars548_interface__msg__SensorConfiguration * output);

/// Initialize array of msg/SensorConfiguration messages.
/**
 * It allocates the memory for the number of elements and calls
 * ars548_interface__msg__SensorConfiguration__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__SensorConfiguration__Sequence__init(ars548_interface__msg__SensorConfiguration__Sequence * array, size_t size);

/// Finalize array of msg/SensorConfiguration messages.
/**
 * It calls
 * ars548_interface__msg__SensorConfiguration__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__SensorConfiguration__Sequence__fini(ars548_interface__msg__SensorConfiguration__Sequence * array);

/// Create array of msg/SensorConfiguration messages.
/**
 * It allocates the memory for the array and calls
 * ars548_interface__msg__SensorConfiguration__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
ars548_interface__msg__SensorConfiguration__Sequence *
ars548_interface__msg__SensorConfiguration__Sequence__create(size_t size);

/// Destroy array of msg/SensorConfiguration messages.
/**
 * It calls
 * ars548_interface__msg__SensorConfiguration__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
void
ars548_interface__msg__SensorConfiguration__Sequence__destroy(ars548_interface__msg__SensorConfiguration__Sequence * array);

/// Check for msg/SensorConfiguration message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ars548_interface
bool
ars548_interface__msg__SensorConfiguration__Sequence__are_equal(const ars548_interface__msg__SensorConfiguration__Sequence * lhs, const ars548_interface__msg__SensorConfiguration__Sequence * rhs);

/// Copy an array of msg/SensorConfiguration messages.
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
ars548_interface__msg__SensorConfiguration__Sequence__copy(
  const ars548_interface__msg__SensorConfiguration__Sequence * input,
  ars548_interface__msg__SensorConfiguration__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__FUNCTIONS_H_
