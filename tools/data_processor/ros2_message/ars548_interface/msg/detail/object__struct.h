// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__OBJECT__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__OBJECT__STRUCT_H_

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

// Struct defined in msg/Object in the package ars548_interface.
typedef struct ars548_interface__msg__Object
{
  std_msgs__msg__Header header;
  uint16_t u_status_sensor;
  uint32_t u_id;
  uint16_t u_age;
  uint8_t u_status_measurement;
  uint8_t u_status_movement;
  uint16_t u_position_invalid_flags;
  uint8_t u_position_reference;
  float u_position_x;
  float u_position_y;
  float u_position_z;
  float u_position_x_std;
  float u_position_y_std;
  float u_position_z_std;
  float u_position_covariance_xy;
  float u_position_orientation;
  float u_position_orientation_std;
  uint8_t u_existence_invalid_flags;
  float u_existence_probability;
  float u_existence_ppv;
  uint8_t u_classification_car;
  uint8_t u_classification_truck;
  uint8_t u_classification_motorcycle;
  uint8_t u_classification_bicycle;
  uint8_t u_classification_pedestrian;
  uint8_t u_classification_animal;
  uint8_t u_classification_hazard;
  uint8_t u_classification_unknown;
  uint8_t u_classification_overdrivable;
  uint8_t u_classification_underdrivable;
  uint8_t u_dynamics_abs_vel_invalid_flags;
  float f_dynamics_abs_vel_x;
  float f_dynamics_abs_vel_y;
  float f_dynamics_abs_vel_x_std;
  float f_dynamics_abs_vel_y_std;
  float f_dynamics_abs_vel_covariance_xy;
  uint8_t u_dynamics_rel_vel_invalid_flags;
  float f_dynamics_rel_vel_x;
  float f_dynamics_rel_vel_y;
  float f_dynamics_rel_vel_x_std;
  float f_dynamics_rel_vel_y_std;
  float f_dynamics_rel_vel_covariance_xy;
  uint8_t u_dynamics_abs_accel_invalid_flags;
  float f_dynamics_abs_accel_x;
  float f_dynamics_abs_accel_y;
  float f_dynamics_abs_accel_x_std;
  float f_dynamics_abs_accel_y_std;
  float f_dynamics_abs_accel_covariance_xy;
  uint8_t u_dynamics_rel_accel_invalid_flags;
  float f_dynamics_rel_accel_x;
  float f_dynamics_rel_accel_y;
  float f_dynamics_rel_accel_x_std;
  float f_dynamics_rel_accel_y_std;
  float f_dynamics_rel_accel_covariance_xy;
  uint8_t u_dynamics_orientation_invalid_flags;
  float u_dynamics_orientation_rate_mean;
  float u_dynamics_orientation_rate_std;
  uint32_t u_shape_length_status;
  uint8_t u_shape_length_edge_invalid_flags;
  float u_shape_length_edge_mean;
  float u_shape_length_edge_std;
  uint32_t u_shape_width_status;
  uint8_t u_shape_width_edge_invalid_flags;
  float u_shape_width_edge_mean;
  float u_shape_width_edge_std;
} ars548_interface__msg__Object;

// Struct for a sequence of ars548_interface__msg__Object.
typedef struct ars548_interface__msg__Object__Sequence
{
  ars548_interface__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__OBJECT__STRUCT_H_
