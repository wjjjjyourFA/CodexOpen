// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/SteeringAngleFrontAxle.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__STRUCT_H_

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

// Struct defined in msg/SteeringAngleFrontAxle in the package ars548_interface.
typedef struct ars548_interface__msg__SteeringAngleFrontAxle
{
  std_msgs__msg__Header header;
  uint8_t qualifier_steering_angle_front_axle;
  float steering_angle_front_axle_err_amp;
  uint8_t steering_angle_front_axle_err_amp_invalid_flag;
  float steering_angle_front_axle;
  uint8_t steering_angle_front_axle_invalid_flag;
  uint8_t steering_angle_front_axle_event_data_qualifier;
} ars548_interface__msg__SteeringAngleFrontAxle;

// Struct for a sequence of ars548_interface__msg__SteeringAngleFrontAxle.
typedef struct ars548_interface__msg__SteeringAngleFrontAxle__Sequence
{
  ars548_interface__msg__SteeringAngleFrontAxle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__SteeringAngleFrontAxle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__STRUCT_H_
