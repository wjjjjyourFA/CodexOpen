// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sensor:msg/GnssSolutionStatus.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__STRUCT_H_
#define SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SOL_COMPUTED'.
enum
{
  sensor__msg__GnssSolutionStatus__SOL_COMPUTED = 0
};

/// Constant 'INSUFFICIENT_OBS'.
enum
{
  sensor__msg__GnssSolutionStatus__INSUFFICIENT_OBS = 1
};

/// Constant 'NO_CONVERGENCE'.
enum
{
  sensor__msg__GnssSolutionStatus__NO_CONVERGENCE = 2
};

/// Constant 'SINGULARITY'.
enum
{
  sensor__msg__GnssSolutionStatus__SINGULARITY = 3
};

/// Constant 'COV_TRACE'.
enum
{
  sensor__msg__GnssSolutionStatus__COV_TRACE = 4
};

/// Constant 'TEST_DIST'.
enum
{
  sensor__msg__GnssSolutionStatus__TEST_DIST = 5
};

/// Constant 'COLD_START'.
enum
{
  sensor__msg__GnssSolutionStatus__COLD_START = 6
};

/// Constant 'V_H_LIMIT'.
enum
{
  sensor__msg__GnssSolutionStatus__V_H_LIMIT = 7
};

/// Constant 'VARIANCE'.
enum
{
  sensor__msg__GnssSolutionStatus__VARIANCE = 8
};

/// Constant 'RESIDUALS'.
enum
{
  sensor__msg__GnssSolutionStatus__RESIDUALS = 9
};

/// Constant 'PENDING'.
enum
{
  sensor__msg__GnssSolutionStatus__PENDING = 18
};

/// Constant 'INVALID_FIX'.
enum
{
  sensor__msg__GnssSolutionStatus__INVALID_FIX = 19
};

// Struct defined in msg/GnssSolutionStatus in the package sensor.
typedef struct sensor__msg__GnssSolutionStatus
{
  int8_t sol_status;
} sensor__msg__GnssSolutionStatus;

// Struct for a sequence of sensor__msg__GnssSolutionStatus.
typedef struct sensor__msg__GnssSolutionStatus__Sequence
{
  sensor__msg__GnssSolutionStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sensor__msg__GnssSolutionStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__STRUCT_H_
