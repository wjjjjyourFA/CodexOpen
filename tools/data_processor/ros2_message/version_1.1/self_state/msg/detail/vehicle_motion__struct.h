// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from self_state:msg/VehicleMotion.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__STRUCT_H_
#define SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/VehicleMotion in the package self_state.
typedef struct self_state__msg__VehicleMotion
{
  double local_time;
  double utc_time;
  int32_t message_num;
  double longitudinal_speed;
  double lateral_speed;
  double vertical_speed;
  double angular_x;
  double angular_y;
  double angular_z;
  double acc_x;
  double acc_y;
  double acc_z;
} self_state__msg__VehicleMotion;

// Struct for a sequence of self_state__msg__VehicleMotion.
typedef struct self_state__msg__VehicleMotion__Sequence
{
  self_state__msg__VehicleMotion * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} self_state__msg__VehicleMotion__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__STRUCT_H_
