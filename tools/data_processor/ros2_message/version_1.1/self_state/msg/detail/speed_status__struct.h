// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from self_state:msg/SpeedStatus.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__SPEED_STATUS__STRUCT_H_
#define SELF_STATE__MSG__DETAIL__SPEED_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SpeedStatus in the package self_state.
typedef struct self_state__msg__SpeedStatus
{
  double local_time;
  double utc_time;
  int32_t message_num;
  double desired_speed;
  double desired_acc;
  double current_speed;
  double current_acc;
  double desired_brake;
  double current_brake;
  double desired_fuel;
  double current_fuel;
  int32_t desired_trans_pos;
  int32_t current_trans_pos;
  int32_t hard_switch_on;
  int32_t emergence_flag;
  int32_t bcan_control_flag;
  int32_t horn_on_flag;
  int32_t emergency_lighton_flag;
} self_state__msg__SpeedStatus;

// Struct for a sequence of self_state__msg__SpeedStatus.
typedef struct self_state__msg__SpeedStatus__Sequence
{
  self_state__msg__SpeedStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} self_state__msg__SpeedStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__SPEED_STATUS__STRUCT_H_
