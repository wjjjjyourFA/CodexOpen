// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from self_state:msg/ChassisInfo.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__CHASSIS_INFO__BUILDER_HPP_
#define SELF_STATE__MSG__DETAIL__CHASSIS_INFO__BUILDER_HPP_

#include "self_state/msg/detail/chassis_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace self_state
{

namespace msg
{

namespace builder
{

class Init_ChassisInfo_reserved
{
public:
  explicit Init_ChassisInfo_reserved(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  ::self_state::msg::ChassisInfo reserved(::self_state::msg::ChassisInfo::_reserved_type arg)
  {
    msg_.reserved = std::move(arg);
    return std::move(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_veh_spd_valid_flag
{
public:
  explicit Init_ChassisInfo_veh_spd_valid_flag(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_reserved veh_spd_valid_flag(::self_state::msg::ChassisInfo::_veh_spd_valid_flag_type arg)
  {
    msg_.veh_spd_valid_flag = std::move(arg);
    return Init_ChassisInfo_reserved(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_electrical_power_steering_availablity_status
{
public:
  explicit Init_ChassisInfo_electrical_power_steering_availablity_status(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_veh_spd_valid_flag electrical_power_steering_availablity_status(::self_state::msg::ChassisInfo::_electrical_power_steering_availablity_status_type arg)
  {
    msg_.electrical_power_steering_availablity_status = std::move(arg);
    return Init_ChassisInfo_veh_spd_valid_flag(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_state_act
{
public:
  explicit Init_ChassisInfo_state_act(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_electrical_power_steering_availablity_status state_act(::self_state::msg::ChassisInfo::_state_act_type arg)
  {
    msg_.state_act = std::move(arg);
    return Init_ChassisInfo_electrical_power_steering_availablity_status(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_rear_fog_light
{
public:
  explicit Init_ChassisInfo_rear_fog_light(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_state_act rear_fog_light(::self_state::msg::ChassisInfo::_rear_fog_light_type arg)
  {
    msg_.rear_fog_light = std::move(arg);
    return Init_ChassisInfo_state_act(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_front_fog_light
{
public:
  explicit Init_ChassisInfo_front_fog_light(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_rear_fog_light front_fog_light(::self_state::msg::ChassisInfo::_front_fog_light_type arg)
  {
    msg_.front_fog_light = std::move(arg);
    return Init_ChassisInfo_rear_fog_light(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_low_beam
{
public:
  explicit Init_ChassisInfo_low_beam(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_front_fog_light low_beam(::self_state::msg::ChassisInfo::_low_beam_type arg)
  {
    msg_.low_beam = std::move(arg);
    return Init_ChassisInfo_front_fog_light(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_high_beam
{
public:
  explicit Init_ChassisInfo_high_beam(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_low_beam high_beam(::self_state::msg::ChassisInfo::_high_beam_type arg)
  {
    msg_.high_beam = std::move(arg);
    return Init_ChassisInfo_low_beam(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_postion_light
{
public:
  explicit Init_ChassisInfo_postion_light(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_high_beam postion_light(::self_state::msg::ChassisInfo::_postion_light_type arg)
  {
    msg_.postion_light = std::move(arg);
    return Init_ChassisInfo_high_beam(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_brake_light
{
public:
  explicit Init_ChassisInfo_brake_light(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_postion_light brake_light(::self_state::msg::ChassisInfo::_brake_light_type arg)
  {
    msg_.brake_light = std::move(arg);
    return Init_ChassisInfo_postion_light(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_headlight
{
public:
  explicit Init_ChassisInfo_headlight(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_brake_light headlight(::self_state::msg::ChassisInfo::_headlight_type arg)
  {
    msg_.headlight = std::move(arg);
    return Init_ChassisInfo_brake_light(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_right_signal_light
{
public:
  explicit Init_ChassisInfo_right_signal_light(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_headlight right_signal_light(::self_state::msg::ChassisInfo::_right_signal_light_type arg)
  {
    msg_.right_signal_light = std::move(arg);
    return Init_ChassisInfo_headlight(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_left_signal_light
{
public:
  explicit Init_ChassisInfo_left_signal_light(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_right_signal_light left_signal_light(::self_state::msg::ChassisInfo::_left_signal_light_type arg)
  {
    msg_.left_signal_light = std::move(arg);
    return Init_ChassisInfo_right_signal_light(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_horn
{
public:
  explicit Init_ChassisInfo_horn(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_left_signal_light horn(::self_state::msg::ChassisInfo::_horn_type arg)
  {
    msg_.horn = std::move(arg);
    return Init_ChassisInfo_left_signal_light(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_err_code
{
public:
  explicit Init_ChassisInfo_err_code(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_horn err_code(::self_state::msg::ChassisInfo::_err_code_type arg)
  {
    msg_.err_code = std::move(arg);
    return Init_ChassisInfo_horn(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_sta_code
{
public:
  explicit Init_ChassisInfo_sta_code(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_err_code sta_code(::self_state::msg::ChassisInfo::_sta_code_type arg)
  {
    msg_.sta_code = std::move(arg);
    return Init_ChassisInfo_err_code(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_battery_soc
{
public:
  explicit Init_ChassisInfo_battery_soc(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_sta_code battery_soc(::self_state::msg::ChassisInfo::_battery_soc_type arg)
  {
    msg_.battery_soc = std::move(arg);
    return Init_ChassisInfo_sta_code(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_oil
{
public:
  explicit Init_ChassisInfo_oil(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_battery_soc oil(::self_state::msg::ChassisInfo::_oil_type arg)
  {
    msg_.oil = std::move(arg);
    return Init_ChassisInfo_battery_soc(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_auto_enable
{
public:
  explicit Init_ChassisInfo_auto_enable(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_oil auto_enable(::self_state::msg::ChassisInfo::_auto_enable_type arg)
  {
    msg_.auto_enable = std::move(arg);
    return Init_ChassisInfo_oil(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_auto_enable
{
public:
  explicit Init_ChassisInfo_exp_auto_enable(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_auto_enable exp_auto_enable(::self_state::msg::ChassisInfo::_exp_auto_enable_type arg)
  {
    msg_.exp_auto_enable = std::move(arg);
    return Init_ChassisInfo_auto_enable(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_actual_gear
{
public:
  explicit Init_ChassisInfo_actual_gear(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_auto_enable actual_gear(::self_state::msg::ChassisInfo::_actual_gear_type arg)
  {
    msg_.actual_gear = std::move(arg);
    return Init_ChassisInfo_exp_auto_enable(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_cur_handbrake
{
public:
  explicit Init_ChassisInfo_cur_handbrake(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_actual_gear cur_handbrake(::self_state::msg::ChassisInfo::_cur_handbrake_type arg)
  {
    msg_.cur_handbrake = std::move(arg);
    return Init_ChassisInfo_actual_gear(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_cur_gear
{
public:
  explicit Init_ChassisInfo_cur_gear(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_cur_handbrake cur_gear(::self_state::msg::ChassisInfo::_cur_gear_type arg)
  {
    msg_.cur_gear = std::move(arg);
    return Init_ChassisInfo_cur_handbrake(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_cur_engine_speed
{
public:
  explicit Init_ChassisInfo_cur_engine_speed(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_cur_gear cur_engine_speed(::self_state::msg::ChassisInfo::_cur_engine_speed_type arg)
  {
    msg_.cur_engine_speed = std::move(arg);
    return Init_ChassisInfo_cur_gear(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_cur_brake
{
public:
  explicit Init_ChassisInfo_cur_brake(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_cur_engine_speed cur_brake(::self_state::msg::ChassisInfo::_cur_brake_type arg)
  {
    msg_.cur_brake = std::move(arg);
    return Init_ChassisInfo_cur_engine_speed(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_cur_fuel
{
public:
  explicit Init_ChassisInfo_cur_fuel(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_cur_brake cur_fuel(::self_state::msg::ChassisInfo::_cur_fuel_type arg)
  {
    msg_.cur_fuel = std::move(arg);
    return Init_ChassisInfo_cur_brake(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_cur_steer_wheel_speed
{
public:
  explicit Init_ChassisInfo_cur_steer_wheel_speed(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_cur_fuel cur_steer_wheel_speed(::self_state::msg::ChassisInfo::_cur_steer_wheel_speed_type arg)
  {
    msg_.cur_steer_wheel_speed = std::move(arg);
    return Init_ChassisInfo_cur_fuel(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_cur_steer_wheel_angle
{
public:
  explicit Init_ChassisInfo_cur_steer_wheel_angle(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_cur_steer_wheel_speed cur_steer_wheel_angle(::self_state::msg::ChassisInfo::_cur_steer_wheel_angle_type arg)
  {
    msg_.cur_steer_wheel_angle = std::move(arg);
    return Init_ChassisInfo_cur_steer_wheel_speed(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_handbrake
{
public:
  explicit Init_ChassisInfo_exp_handbrake(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_cur_steer_wheel_angle exp_handbrake(::self_state::msg::ChassisInfo::_exp_handbrake_type arg)
  {
    msg_.exp_handbrake = std::move(arg);
    return Init_ChassisInfo_cur_steer_wheel_angle(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_gear
{
public:
  explicit Init_ChassisInfo_exp_gear(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_handbrake exp_gear(::self_state::msg::ChassisInfo::_exp_gear_type arg)
  {
    msg_.exp_gear = std::move(arg);
    return Init_ChassisInfo_exp_handbrake(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_brake
{
public:
  explicit Init_ChassisInfo_exp_brake(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_gear exp_brake(::self_state::msg::ChassisInfo::_exp_brake_type arg)
  {
    msg_.exp_brake = std::move(arg);
    return Init_ChassisInfo_exp_gear(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_fuel
{
public:
  explicit Init_ChassisInfo_exp_fuel(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_brake exp_fuel(::self_state::msg::ChassisInfo::_exp_fuel_type arg)
  {
    msg_.exp_fuel = std::move(arg);
    return Init_ChassisInfo_exp_brake(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_steer_wheel_speed
{
public:
  explicit Init_ChassisInfo_exp_steer_wheel_speed(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_fuel exp_steer_wheel_speed(::self_state::msg::ChassisInfo::_exp_steer_wheel_speed_type arg)
  {
    msg_.exp_steer_wheel_speed = std::move(arg);
    return Init_ChassisInfo_exp_fuel(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_steer_wheel_angle
{
public:
  explicit Init_ChassisInfo_exp_steer_wheel_angle(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_steer_wheel_speed exp_steer_wheel_angle(::self_state::msg::ChassisInfo::_exp_steer_wheel_angle_type arg)
  {
    msg_.exp_steer_wheel_angle = std::move(arg);
    return Init_ChassisInfo_exp_steer_wheel_speed(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_yaw_rate
{
public:
  explicit Init_ChassisInfo_exp_yaw_rate(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_steer_wheel_angle exp_yaw_rate(::self_state::msg::ChassisInfo::_exp_yaw_rate_type arg)
  {
    msg_.exp_yaw_rate = std::move(arg);
    return Init_ChassisInfo_exp_steer_wheel_angle(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_acceleration
{
public:
  explicit Init_ChassisInfo_exp_acceleration(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_yaw_rate exp_acceleration(::self_state::msg::ChassisInfo::_exp_acceleration_type arg)
  {
    msg_.exp_acceleration = std::move(arg);
    return Init_ChassisInfo_exp_yaw_rate(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_exp_speed
{
public:
  explicit Init_ChassisInfo_exp_speed(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_acceleration exp_speed(::self_state::msg::ChassisInfo::_exp_speed_type arg)
  {
    msg_.exp_speed = std::move(arg);
    return Init_ChassisInfo_exp_acceleration(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_cur_spd
{
public:
  explicit Init_ChassisInfo_cur_spd(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_exp_speed cur_spd(::self_state::msg::ChassisInfo::_cur_spd_type arg)
  {
    msg_.cur_spd = std::move(arg);
    return Init_ChassisInfo_exp_speed(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_yaw_rate
{
public:
  explicit Init_ChassisInfo_yaw_rate(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_cur_spd yaw_rate(::self_state::msg::ChassisInfo::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return Init_ChassisInfo_cur_spd(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_lat_acc
{
public:
  explicit Init_ChassisInfo_lat_acc(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_yaw_rate lat_acc(::self_state::msg::ChassisInfo::_lat_acc_type arg)
  {
    msg_.lat_acc = std::move(arg);
    return Init_ChassisInfo_yaw_rate(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_lon_acc
{
public:
  explicit Init_ChassisInfo_lon_acc(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_lat_acc lon_acc(::self_state::msg::ChassisInfo::_lon_acc_type arg)
  {
    msg_.lon_acc = std::move(arg);
    return Init_ChassisInfo_lat_acc(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_rr_wheel_pulse_counter
{
public:
  explicit Init_ChassisInfo_rr_wheel_pulse_counter(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_lon_acc rr_wheel_pulse_counter(::self_state::msg::ChassisInfo::_rr_wheel_pulse_counter_type arg)
  {
    msg_.rr_wheel_pulse_counter = std::move(arg);
    return Init_ChassisInfo_lon_acc(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_rf_wheel_pulse_counter
{
public:
  explicit Init_ChassisInfo_rf_wheel_pulse_counter(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_rr_wheel_pulse_counter rf_wheel_pulse_counter(::self_state::msg::ChassisInfo::_rf_wheel_pulse_counter_type arg)
  {
    msg_.rf_wheel_pulse_counter = std::move(arg);
    return Init_ChassisInfo_rr_wheel_pulse_counter(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_lr_wheel_pulse_counter
{
public:
  explicit Init_ChassisInfo_lr_wheel_pulse_counter(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_rf_wheel_pulse_counter lr_wheel_pulse_counter(::self_state::msg::ChassisInfo::_lr_wheel_pulse_counter_type arg)
  {
    msg_.lr_wheel_pulse_counter = std::move(arg);
    return Init_ChassisInfo_rf_wheel_pulse_counter(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_lf_wheel_pulse_counter
{
public:
  explicit Init_ChassisInfo_lf_wheel_pulse_counter(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_lr_wheel_pulse_counter lf_wheel_pulse_counter(::self_state::msg::ChassisInfo::_lf_wheel_pulse_counter_type arg)
  {
    msg_.lf_wheel_pulse_counter = std::move(arg);
    return Init_ChassisInfo_lr_wheel_pulse_counter(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_rr_wheel_speed
{
public:
  explicit Init_ChassisInfo_rr_wheel_speed(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_lf_wheel_pulse_counter rr_wheel_speed(::self_state::msg::ChassisInfo::_rr_wheel_speed_type arg)
  {
    msg_.rr_wheel_speed = std::move(arg);
    return Init_ChassisInfo_lf_wheel_pulse_counter(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_rf_wheel_speed
{
public:
  explicit Init_ChassisInfo_rf_wheel_speed(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_rr_wheel_speed rf_wheel_speed(::self_state::msg::ChassisInfo::_rf_wheel_speed_type arg)
  {
    msg_.rf_wheel_speed = std::move(arg);
    return Init_ChassisInfo_rr_wheel_speed(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_lr_wheel_speed
{
public:
  explicit Init_ChassisInfo_lr_wheel_speed(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_rf_wheel_speed lr_wheel_speed(::self_state::msg::ChassisInfo::_lr_wheel_speed_type arg)
  {
    msg_.lr_wheel_speed = std::move(arg);
    return Init_ChassisInfo_rf_wheel_speed(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_lf_wheel_speed
{
public:
  explicit Init_ChassisInfo_lf_wheel_speed(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_lr_wheel_speed lf_wheel_speed(::self_state::msg::ChassisInfo::_lf_wheel_speed_type arg)
  {
    msg_.lf_wheel_speed = std::move(arg);
    return Init_ChassisInfo_lr_wheel_speed(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_message_num
{
public:
  explicit Init_ChassisInfo_message_num(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_lf_wheel_speed message_num(::self_state::msg::ChassisInfo::_message_num_type arg)
  {
    msg_.message_num = std::move(arg);
    return Init_ChassisInfo_lf_wheel_speed(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_utc_time
{
public:
  explicit Init_ChassisInfo_utc_time(::self_state::msg::ChassisInfo & msg)
  : msg_(msg)
  {}
  Init_ChassisInfo_message_num utc_time(::self_state::msg::ChassisInfo::_utc_time_type arg)
  {
    msg_.utc_time = std::move(arg);
    return Init_ChassisInfo_message_num(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

class Init_ChassisInfo_local_time
{
public:
  Init_ChassisInfo_local_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChassisInfo_utc_time local_time(::self_state::msg::ChassisInfo::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_ChassisInfo_utc_time(msg_);
  }

private:
  ::self_state::msg::ChassisInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::self_state::msg::ChassisInfo>()
{
  return self_state::msg::builder::Init_ChassisInfo_local_time();
}

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__CHASSIS_INFO__BUILDER_HPP_
