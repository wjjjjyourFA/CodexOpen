// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from self_state:msg/SteerAngle.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__STEER_ANGLE__STRUCT_H_
#define SELF_STATE__MSG__DETAIL__STEER_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SteerAngle in the package self_state.
typedef struct self_state__msg__SteerAngle
{
  double local_time;
  double utc_time;
  int32_t message_num;
  double actual_front_wheel_angle;
  double desired_front_wheel_angle;
  double actual_curvature;
  double desired_curvature;
  int32_t bcan_control_flag;
  int32_t left_light_flag;
  int32_t right_light_flag;
} self_state__msg__SteerAngle;

// Struct for a sequence of self_state__msg__SteerAngle.
typedef struct self_state__msg__SteerAngle__Sequence
{
  self_state__msg__SteerAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} self_state__msg__SteerAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__STEER_ANGLE__STRUCT_H_
