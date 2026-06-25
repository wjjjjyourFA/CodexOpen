// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sensor:msg/GnssSolutionStatus.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__FUNCTIONS_H_
#define SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "sensor/msg/rosidl_generator_c__visibility_control.h"

#include "sensor/msg/detail/gnss_solution_status__struct.h"

/// Initialize msg/GnssSolutionStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sensor__msg__GnssSolutionStatus
 * )) before or use
 * sensor__msg__GnssSolutionStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
bool
sensor__msg__GnssSolutionStatus__init(sensor__msg__GnssSolutionStatus * msg);

/// Finalize msg/GnssSolutionStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
void
sensor__msg__GnssSolutionStatus__fini(sensor__msg__GnssSolutionStatus * msg);

/// Create msg/GnssSolutionStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sensor__msg__GnssSolutionStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
sensor__msg__GnssSolutionStatus *
sensor__msg__GnssSolutionStatus__create();

/// Destroy msg/GnssSolutionStatus message.
/**
 * It calls
 * sensor__msg__GnssSolutionStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
void
sensor__msg__GnssSolutionStatus__destroy(sensor__msg__GnssSolutionStatus * msg);

/// Check for msg/GnssSolutionStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
bool
sensor__msg__GnssSolutionStatus__are_equal(const sensor__msg__GnssSolutionStatus * lhs, const sensor__msg__GnssSolutionStatus * rhs);

/// Copy a msg/GnssSolutionStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_sensor
bool
sensor__msg__GnssSolutionStatus__copy(
  const sensor__msg__GnssSolutionStatus * input,
  sensor__msg__GnssSolutionStatus * output);

/// Initialize array of msg/GnssSolutionStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * sensor__msg__GnssSolutionStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
bool
sensor__msg__GnssSolutionStatus__Sequence__init(sensor__msg__GnssSolutionStatus__Sequence * array, size_t size);

/// Finalize array of msg/GnssSolutionStatus messages.
/**
 * It calls
 * sensor__msg__GnssSolutionStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
void
sensor__msg__GnssSolutionStatus__Sequence__fini(sensor__msg__GnssSolutionStatus__Sequence * array);

/// Create array of msg/GnssSolutionStatus messages.
/**
 * It allocates the memory for the array and calls
 * sensor__msg__GnssSolutionStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
sensor__msg__GnssSolutionStatus__Sequence *
sensor__msg__GnssSolutionStatus__Sequence__create(size_t size);

/// Destroy array of msg/GnssSolutionStatus messages.
/**
 * It calls
 * sensor__msg__GnssSolutionStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
void
sensor__msg__GnssSolutionStatus__Sequence__destroy(sensor__msg__GnssSolutionStatus__Sequence * array);

/// Check for msg/GnssSolutionStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sensor
bool
sensor__msg__GnssSolutionStatus__Sequence__are_equal(const sensor__msg__GnssSolutionStatus__Sequence * lhs, const sensor__msg__GnssSolutionStatus__Sequence * rhs);

/// Copy an array of msg/GnssSolutionStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_sensor
bool
sensor__msg__GnssSolutionStatus__Sequence__copy(
  const sensor__msg__GnssSolutionStatus__Sequence * input,
  sensor__msg__GnssSolutionStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__FUNCTIONS_H_
