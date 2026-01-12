// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from self_state:msg/ChassisInfo.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__CHASSIS_INFO__STRUCT_H_
#define SELF_STATE__MSG__DETAIL__CHASSIS_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/ChassisInfo in the package self_state.
typedef struct self_state__msg__ChassisInfo
{
  double local_time;
  double utc_time;
  int32_t message_num;
  double lf_wheel_speed;
  double lr_wheel_speed;
  double rf_wheel_speed;
  double rr_wheel_speed;
  uint64_t lf_wheel_pulse_counter;
  uint64_t lr_wheel_pulse_counter;
  uint64_t rf_wheel_pulse_counter;
  uint64_t rr_wheel_pulse_counter;
  double lon_acc;
  double lat_acc;
  double yaw_rate;
  double cur_spd;
  double exp_speed;
  double exp_acceleration;
  double exp_yaw_rate;
  double exp_steer_wheel_angle;
  double exp_steer_wheel_speed;
  double exp_fuel;
  double exp_brake;
  int8_t exp_gear;
  int8_t exp_handbrake;
  double cur_steer_wheel_angle;
  double cur_steer_wheel_speed;
  double cur_fuel;
  double cur_brake;
  double cur_engine_speed;
  uint8_t cur_gear;
  uint8_t cur_handbrake;
  uint8_t actual_gear;
  int8_t exp_auto_enable;
  int8_t auto_enable;
  int8_t oil;
  int8_t battery_soc;
  uint32_t sta_code;
  uint32_t err_code;
  uint8_t horn;
  uint8_t left_signal_light;
  uint8_t right_signal_light;
  uint8_t headlight;
  uint8_t brake_light;
  uint8_t postion_light;
  uint8_t high_beam;
  uint8_t low_beam;
  uint8_t front_fog_light;
  uint8_t rear_fog_light;
  uint8_t state_act;
  uint8_t electrical_power_steering_availablity_status;
  uint8_t veh_spd_valid_flag;
  uint32_t reserved[8];
} self_state__msg__ChassisInfo;

// Struct for a sequence of self_state__msg__ChassisInfo.
typedef struct self_state__msg__ChassisInfo__Sequence
{
  self_state__msg__ChassisInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} self_state__msg__ChassisInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__CHASSIS_INFO__STRUCT_H_
