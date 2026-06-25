// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from self_state:msg/InsStatus.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__INS_STATUS__STRUCT_H_
#define SELF_STATE__MSG__DETAIL__INS_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'INS_INACTIVE'.
enum
{
  self_state__msg__InsStatus__INS_INACTIVE = 0
};

/// Constant 'INS_ALIGNING'.
enum
{
  self_state__msg__InsStatus__INS_ALIGNING = 1
};

/// Constant 'INS_HIGH_VARIANCE'.
enum
{
  self_state__msg__InsStatus__INS_HIGH_VARIANCE = 2
};

/// Constant 'INS_SOLUTION_GOOD'.
enum
{
  self_state__msg__InsStatus__INS_SOLUTION_GOOD = 3
};

/// Constant 'INS_SOLUTION_FREE'.
enum
{
  self_state__msg__InsStatus__INS_SOLUTION_FREE = 6
};

/// Constant 'INS_ALIGNMENT_COMPLETE'.
enum
{
  self_state__msg__InsStatus__INS_ALIGNMENT_COMPLETE = 7
};

/// Constant 'DETERMINING_ORIENTATION'.
enum
{
  self_state__msg__InsStatus__DETERMINING_ORIENTATION = 8
};

/// Constant 'WAITING_INITIALPOS'.
enum
{
  self_state__msg__InsStatus__WAITING_INITIALPOS = 9
};

// Struct defined in msg/InsStatus in the package self_state.
typedef struct self_state__msg__InsStatus
{
  int8_t ins_status;
} self_state__msg__InsStatus;

// Struct for a sequence of self_state__msg__InsStatus.
typedef struct self_state__msg__InsStatus__Sequence
{
  self_state__msg__InsStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} self_state__msg__InsStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__INS_STATUS__STRUCT_H_
