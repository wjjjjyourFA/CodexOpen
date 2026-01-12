// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/DrivingDirection.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/DrivingDirection in the package ars548_interface.
typedef struct ars548_interface__msg__DrivingDirection
{
  std_msgs__msg__Header header;
  uint8_t driving_direction_unconfirmed;
  uint8_t driving_direction_confirmed;
} ars548_interface__msg__DrivingDirection;

// Struct for a sequence of ars548_interface__msg__DrivingDirection.
typedef struct ars548_interface__msg__DrivingDirection__Sequence
{
  ars548_interface__msg__DrivingDirection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__DrivingDirection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__STRUCT_H_
