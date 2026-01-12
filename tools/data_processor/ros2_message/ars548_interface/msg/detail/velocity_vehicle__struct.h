// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/VelocityVehicle.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__STRUCT_H_

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

// Struct defined in msg/VelocityVehicle in the package ars548_interface.
typedef struct ars548_interface__msg__VelocityVehicle
{
  std_msgs__msg__Header header;
  uint8_t status_velocity_near_standstill;
  uint8_t qualifier_velocity_vehicle;
  uint8_t velocity_vehicle_event_data_qualifier;
  float velocity_vehicle;
  uint8_t velocity_vehicle_invalid_flag;
} ars548_interface__msg__VelocityVehicle;

// Struct for a sequence of ars548_interface__msg__VelocityVehicle.
typedef struct ars548_interface__msg__VelocityVehicle__Sequence
{
  ars548_interface__msg__VelocityVehicle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__VelocityVehicle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__STRUCT_H_
