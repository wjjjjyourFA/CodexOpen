// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DETECTION__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__DETECTION__STRUCT_H_

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

// Struct defined in msg/Detection in the package ars548_interface.
typedef struct ars548_interface__msg__Detection
{
  std_msgs__msg__Header header;
  float f_x;
  float f_y;
  float f_z;
  float f_azimuth_angle;
  float f_azimuth_angle_std;
  uint8_t u_invalid_flags;
  float f_elevation_angle;
  float f_elevation_angle_std;
  float f_range;
  float f_range_std;
  float f_range_rate;
  float f_range_rate_std;
  int8_t s_rcs;
  uint16_t u_measurement_id;
  uint8_t u_positive_predictive_value;
  uint8_t u_classification;
  uint8_t u_multi_target_probability;
  uint16_t u_object_id;
  uint8_t u_ambiguity_flag;
  uint16_t u_sort_index;
} ars548_interface__msg__Detection;

// Struct for a sequence of ars548_interface__msg__Detection.
typedef struct ars548_interface__msg__Detection__Sequence
{
  ars548_interface__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__DETECTION__STRUCT_H_
