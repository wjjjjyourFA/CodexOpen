// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/SensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__STRUCT_H_

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

// Struct defined in msg/SensorConfiguration in the package ars548_interface.
typedef struct ars548_interface__msg__SensorConfiguration
{
  std_msgs__msg__Header header;
  float longitudinal;
  float lateral;
  float vertical;
  float yaw;
  float pitch;
  uint8_t plug_orientation;
  float length;
  float width;
  float height;
  float wheelbase;
  uint16_t maximum_distance;
  uint8_t frequency_slot;
  uint8_t cycle_time;
  uint8_t time_slot;
  uint8_t hcc;
  uint8_t powersave_standstill;
  uint32_t sensor_ip_address_0;
  uint32_t sensor_ip_address_1;
  uint8_t new_sensor_mounting;
  uint8_t new_vehicle_parameters;
  uint8_t new_radar_parameters;
  uint8_t new_network_configuration;
} ars548_interface__msg__SensorConfiguration;

// Struct for a sequence of ars548_interface__msg__SensorConfiguration.
typedef struct ars548_interface__msg__SensorConfiguration__Sequence
{
  ars548_interface__msg__SensorConfiguration * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__SensorConfiguration__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__STRUCT_H_
