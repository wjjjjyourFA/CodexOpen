// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/SensorStatus.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__BUILDER_HPP_

#include "ars548_interface/msg/detail/sensor_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_SensorStatus_status_blockage_status
{
public:
  explicit Init_SensorStatus_status_blockage_status(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::SensorStatus status_blockage_status(::ars548_interface::msg::SensorStatus::_status_blockage_status_type arg)
  {
    msg_.status_blockage_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_temperature_status
{
public:
  explicit Init_SensorStatus_status_temperature_status(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_blockage_status status_temperature_status(::ars548_interface::msg::SensorStatus::_status_temperature_status_type arg)
  {
    msg_.status_temperature_status = std::move(arg);
    return Init_SensorStatus_status_blockage_status(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_voltage_status
{
public:
  explicit Init_SensorStatus_status_voltage_status(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_temperature_status status_voltage_status(::ars548_interface::msg::SensorStatus::_status_voltage_status_type arg)
  {
    msg_.status_voltage_status = std::move(arg);
    return Init_SensorStatus_status_temperature_status(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_radar_status
{
public:
  explicit Init_SensorStatus_status_radar_status(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_voltage_status status_radar_status(::ars548_interface::msg::SensorStatus::_status_radar_status_type arg)
  {
    msg_.status_radar_status = std::move(arg);
    return Init_SensorStatus_status_voltage_status(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_characteristic_speed
{
public:
  explicit Init_SensorStatus_status_characteristic_speed(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_radar_status status_characteristic_speed(::ars548_interface::msg::SensorStatus::_status_characteristic_speed_type arg)
  {
    msg_.status_characteristic_speed = std::move(arg);
    return Init_SensorStatus_status_radar_status(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_driving_direction
{
public:
  explicit Init_SensorStatus_status_driving_direction(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_characteristic_speed status_driving_direction(::ars548_interface::msg::SensorStatus::_status_driving_direction_type arg)
  {
    msg_.status_driving_direction = std::move(arg);
    return Init_SensorStatus_status_characteristic_speed(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_steering_angle
{
public:
  explicit Init_SensorStatus_status_steering_angle(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_driving_direction status_steering_angle(::ars548_interface::msg::SensorStatus::_status_steering_angle_type arg)
  {
    msg_.status_steering_angle = std::move(arg);
    return Init_SensorStatus_status_driving_direction(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_yaw_rate
{
public:
  explicit Init_SensorStatus_status_yaw_rate(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_steering_angle status_yaw_rate(::ars548_interface::msg::SensorStatus::_status_yaw_rate_type arg)
  {
    msg_.status_yaw_rate = std::move(arg);
    return Init_SensorStatus_status_steering_angle(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_lateral_acceleration
{
public:
  explicit Init_SensorStatus_status_lateral_acceleration(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_yaw_rate status_lateral_acceleration(::ars548_interface::msg::SensorStatus::_status_lateral_acceleration_type arg)
  {
    msg_.status_lateral_acceleration = std::move(arg);
    return Init_SensorStatus_status_yaw_rate(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_longitudinal_acceleration
{
public:
  explicit Init_SensorStatus_status_longitudinal_acceleration(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_lateral_acceleration status_longitudinal_acceleration(::ars548_interface::msg::SensorStatus::_status_longitudinal_acceleration_type arg)
  {
    msg_.status_longitudinal_acceleration = std::move(arg);
    return Init_SensorStatus_status_lateral_acceleration(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_status_longitudinal_velocity
{
public:
  explicit Init_SensorStatus_status_longitudinal_velocity(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_longitudinal_acceleration status_longitudinal_velocity(::ars548_interface::msg::SensorStatus::_status_longitudinal_velocity_type arg)
  {
    msg_.status_longitudinal_velocity = std::move(arg);
    return Init_SensorStatus_status_longitudinal_acceleration(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_configuration_counter
{
public:
  explicit Init_SensorStatus_configuration_counter(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_status_longitudinal_velocity configuration_counter(::ars548_interface::msg::SensorStatus::_configuration_counter_type arg)
  {
    msg_.configuration_counter = std::move(arg);
    return Init_SensorStatus_status_longitudinal_velocity(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_sensor_ip_address_1
{
public:
  explicit Init_SensorStatus_sensor_ip_address_1(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_configuration_counter sensor_ip_address_1(::ars548_interface::msg::SensorStatus::_sensor_ip_address_1_type arg)
  {
    msg_.sensor_ip_address_1 = std::move(arg);
    return Init_SensorStatus_configuration_counter(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_sensor_ip_address_0
{
public:
  explicit Init_SensorStatus_sensor_ip_address_0(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_sensor_ip_address_1 sensor_ip_address_0(::ars548_interface::msg::SensorStatus::_sensor_ip_address_0_type arg)
  {
    msg_.sensor_ip_address_0 = std::move(arg);
    return Init_SensorStatus_sensor_ip_address_1(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_powersave_standstill
{
public:
  explicit Init_SensorStatus_powersave_standstill(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_sensor_ip_address_0 powersave_standstill(::ars548_interface::msg::SensorStatus::_powersave_standstill_type arg)
  {
    msg_.powersave_standstill = std::move(arg);
    return Init_SensorStatus_sensor_ip_address_0(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_hcc
{
public:
  explicit Init_SensorStatus_hcc(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_powersave_standstill hcc(::ars548_interface::msg::SensorStatus::_hcc_type arg)
  {
    msg_.hcc = std::move(arg);
    return Init_SensorStatus_powersave_standstill(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_time_slot
{
public:
  explicit Init_SensorStatus_time_slot(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_hcc time_slot(::ars548_interface::msg::SensorStatus::_time_slot_type arg)
  {
    msg_.time_slot = std::move(arg);
    return Init_SensorStatus_hcc(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_cycle_time
{
public:
  explicit Init_SensorStatus_cycle_time(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_time_slot cycle_time(::ars548_interface::msg::SensorStatus::_cycle_time_type arg)
  {
    msg_.cycle_time = std::move(arg);
    return Init_SensorStatus_time_slot(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_frequency_slot
{
public:
  explicit Init_SensorStatus_frequency_slot(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_cycle_time frequency_slot(::ars548_interface::msg::SensorStatus::_frequency_slot_type arg)
  {
    msg_.frequency_slot = std::move(arg);
    return Init_SensorStatus_cycle_time(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_maximum_distance
{
public:
  explicit Init_SensorStatus_maximum_distance(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_frequency_slot maximum_distance(::ars548_interface::msg::SensorStatus::_maximum_distance_type arg)
  {
    msg_.maximum_distance = std::move(arg);
    return Init_SensorStatus_frequency_slot(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_wheelbase
{
public:
  explicit Init_SensorStatus_wheelbase(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_maximum_distance wheelbase(::ars548_interface::msg::SensorStatus::_wheelbase_type arg)
  {
    msg_.wheelbase = std::move(arg);
    return Init_SensorStatus_maximum_distance(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_height
{
public:
  explicit Init_SensorStatus_height(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_wheelbase height(::ars548_interface::msg::SensorStatus::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_SensorStatus_wheelbase(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_width
{
public:
  explicit Init_SensorStatus_width(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_height width(::ars548_interface::msg::SensorStatus::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_SensorStatus_height(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_length
{
public:
  explicit Init_SensorStatus_length(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_width length(::ars548_interface::msg::SensorStatus::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_SensorStatus_width(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_plug_orientation
{
public:
  explicit Init_SensorStatus_plug_orientation(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_length plug_orientation(::ars548_interface::msg::SensorStatus::_plug_orientation_type arg)
  {
    msg_.plug_orientation = std::move(arg);
    return Init_SensorStatus_length(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_pitch
{
public:
  explicit Init_SensorStatus_pitch(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_plug_orientation pitch(::ars548_interface::msg::SensorStatus::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_SensorStatus_plug_orientation(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_yaw
{
public:
  explicit Init_SensorStatus_yaw(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_pitch yaw(::ars548_interface::msg::SensorStatus::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_SensorStatus_pitch(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_vertical
{
public:
  explicit Init_SensorStatus_vertical(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_yaw vertical(::ars548_interface::msg::SensorStatus::_vertical_type arg)
  {
    msg_.vertical = std::move(arg);
    return Init_SensorStatus_yaw(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_lateral
{
public:
  explicit Init_SensorStatus_lateral(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_vertical lateral(::ars548_interface::msg::SensorStatus::_lateral_type arg)
  {
    msg_.lateral = std::move(arg);
    return Init_SensorStatus_vertical(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_longitudinal
{
public:
  explicit Init_SensorStatus_longitudinal(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_lateral longitudinal(::ars548_interface::msg::SensorStatus::_longitudinal_type arg)
  {
    msg_.longitudinal = std::move(arg);
    return Init_SensorStatus_lateral(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_sw_version_patch
{
public:
  explicit Init_SensorStatus_sw_version_patch(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_longitudinal sw_version_patch(::ars548_interface::msg::SensorStatus::_sw_version_patch_type arg)
  {
    msg_.sw_version_patch = std::move(arg);
    return Init_SensorStatus_longitudinal(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_sw_version_minor
{
public:
  explicit Init_SensorStatus_sw_version_minor(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_sw_version_patch sw_version_minor(::ars548_interface::msg::SensorStatus::_sw_version_minor_type arg)
  {
    msg_.sw_version_minor = std::move(arg);
    return Init_SensorStatus_sw_version_patch(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_sw_version_major
{
public:
  explicit Init_SensorStatus_sw_version_major(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_sw_version_minor sw_version_major(::ars548_interface::msg::SensorStatus::_sw_version_major_type arg)
  {
    msg_.sw_version_major = std::move(arg);
    return Init_SensorStatus_sw_version_minor(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_timestamp_sync_status
{
public:
  explicit Init_SensorStatus_timestamp_sync_status(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_sw_version_major timestamp_sync_status(::ars548_interface::msg::SensorStatus::_timestamp_sync_status_type arg)
  {
    msg_.timestamp_sync_status = std::move(arg);
    return Init_SensorStatus_sw_version_major(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_timestamp_seconds
{
public:
  explicit Init_SensorStatus_timestamp_seconds(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_timestamp_sync_status timestamp_seconds(::ars548_interface::msg::SensorStatus::_timestamp_seconds_type arg)
  {
    msg_.timestamp_seconds = std::move(arg);
    return Init_SensorStatus_timestamp_sync_status(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_timestamp_nanoseconds
{
public:
  explicit Init_SensorStatus_timestamp_nanoseconds(::ars548_interface::msg::SensorStatus & msg)
  : msg_(msg)
  {}
  Init_SensorStatus_timestamp_seconds timestamp_nanoseconds(::ars548_interface::msg::SensorStatus::_timestamp_nanoseconds_type arg)
  {
    msg_.timestamp_nanoseconds = std::move(arg);
    return Init_SensorStatus_timestamp_seconds(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

class Init_SensorStatus_header
{
public:
  Init_SensorStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorStatus_timestamp_nanoseconds header(::ars548_interface::msg::SensorStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SensorStatus_timestamp_nanoseconds(msg_);
  }

private:
  ::ars548_interface::msg::SensorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::SensorStatus>()
{
  return ars548_interface::msg::builder::Init_SensorStatus_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__BUILDER_HPP_
