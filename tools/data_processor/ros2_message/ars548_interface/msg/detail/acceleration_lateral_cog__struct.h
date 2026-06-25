// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/AccelerationLateralCog.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__STRUCT_H_

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

// Struct defined in msg/AccelerationLateralCog in the package ars548_interface.
typedef struct ars548_interface__msg__AccelerationLateralCog
{
  std_msgs__msg__Header header;
  float acceleration_lateral_err_amp;
  uint8_t acceleration_lateral_err_amp_invalid_flag;
  uint8_t qualifier_acceleration_lateral;
  float acceleration_lateral;
  uint8_t acceleration_lateral_invalid_flag;
  uint8_t acceleration_lateral_event_data_qualifier;
} ars548_interface__msg__AccelerationLateralCog;

// Struct for a sequence of ars548_interface__msg__AccelerationLateralCog.
typedef struct ars548_interface__msg__AccelerationLateralCog__Sequence
{
  ars548_interface__msg__AccelerationLateralCog * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__AccelerationLateralCog__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__STRUCT_H_
