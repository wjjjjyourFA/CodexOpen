// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/SensorStatus.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__STRUCT_H_

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

// Struct defined in msg/SensorStatus in the package ars548_interface.
typedef struct ars548_interface__msg__SensorStatus
{
  std_msgs__msg__Header header;
  uint32_t timestamp_nanoseconds;
  uint32_t timestamp_seconds;
  uint8_t timestamp_sync_status;
  uint8_t sw_version_major;
  uint8_t sw_version_minor;
  uint8_t sw_version_patch;
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
  uint8_t configuration_counter;
  uint8_t status_longitudinal_velocity;
  uint8_t status_longitudinal_acceleration;
  uint8_t status_lateral_acceleration;
  uint8_t status_yaw_rate;
  uint8_t status_steering_angle;
  uint8_t status_driving_direction;
  uint8_t status_characteristic_speed;
  uint8_t status_radar_status;
  uint8_t status_voltage_status;
  uint8_t status_temperature_status;
  uint8_t status_blockage_status;
} ars548_interface__msg__SensorStatus;

// Struct for a sequence of ars548_interface__msg__SensorStatus.
typedef struct ars548_interface__msg__SensorStatus__Sequence
{
  ars548_interface__msg__SensorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__SensorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__STRUCT_H_
