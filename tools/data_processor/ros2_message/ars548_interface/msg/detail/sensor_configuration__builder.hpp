// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/SensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__BUILDER_HPP_

#include "ars548_interface/msg/detail/sensor_configuration__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_SensorConfiguration_new_network_configuration
{
public:
  explicit Init_SensorConfiguration_new_network_configuration(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::SensorConfiguration new_network_configuration(::ars548_interface::msg::SensorConfiguration::_new_network_configuration_type arg)
  {
    msg_.new_network_configuration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_new_radar_parameters
{
public:
  explicit Init_SensorConfiguration_new_radar_parameters(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_new_network_configuration new_radar_parameters(::ars548_interface::msg::SensorConfiguration::_new_radar_parameters_type arg)
  {
    msg_.new_radar_parameters = std::move(arg);
    return Init_SensorConfiguration_new_network_configuration(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_new_vehicle_parameters
{
public:
  explicit Init_SensorConfiguration_new_vehicle_parameters(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_new_radar_parameters new_vehicle_parameters(::ars548_interface::msg::SensorConfiguration::_new_vehicle_parameters_type arg)
  {
    msg_.new_vehicle_parameters = std::move(arg);
    return Init_SensorConfiguration_new_radar_parameters(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_new_sensor_mounting
{
public:
  explicit Init_SensorConfiguration_new_sensor_mounting(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_new_vehicle_parameters new_sensor_mounting(::ars548_interface::msg::SensorConfiguration::_new_sensor_mounting_type arg)
  {
    msg_.new_sensor_mounting = std::move(arg);
    return Init_SensorConfiguration_new_vehicle_parameters(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_sensor_ip_address_1
{
public:
  explicit Init_SensorConfiguration_sensor_ip_address_1(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_new_sensor_mounting sensor_ip_address_1(::ars548_interface::msg::SensorConfiguration::_sensor_ip_address_1_type arg)
  {
    msg_.sensor_ip_address_1 = std::move(arg);
    return Init_SensorConfiguration_new_sensor_mounting(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_sensor_ip_address_0
{
public:
  explicit Init_SensorConfiguration_sensor_ip_address_0(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_sensor_ip_address_1 sensor_ip_address_0(::ars548_interface::msg::SensorConfiguration::_sensor_ip_address_0_type arg)
  {
    msg_.sensor_ip_address_0 = std::move(arg);
    return Init_SensorConfiguration_sensor_ip_address_1(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_powersave_standstill
{
public:
  explicit Init_SensorConfiguration_powersave_standstill(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_sensor_ip_address_0 powersave_standstill(::ars548_interface::msg::SensorConfiguration::_powersave_standstill_type arg)
  {
    msg_.powersave_standstill = std::move(arg);
    return Init_SensorConfiguration_sensor_ip_address_0(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_hcc
{
public:
  explicit Init_SensorConfiguration_hcc(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_powersave_standstill hcc(::ars548_interface::msg::SensorConfiguration::_hcc_type arg)
  {
    msg_.hcc = std::move(arg);
    return Init_SensorConfiguration_powersave_standstill(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_time_slot
{
public:
  explicit Init_SensorConfiguration_time_slot(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_hcc time_slot(::ars548_interface::msg::SensorConfiguration::_time_slot_type arg)
  {
    msg_.time_slot = std::move(arg);
    return Init_SensorConfiguration_hcc(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_cycle_time
{
public:
  explicit Init_SensorConfiguration_cycle_time(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_time_slot cycle_time(::ars548_interface::msg::SensorConfiguration::_cycle_time_type arg)
  {
    msg_.cycle_time = std::move(arg);
    return Init_SensorConfiguration_time_slot(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_frequency_slot
{
public:
  explicit Init_SensorConfiguration_frequency_slot(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_cycle_time frequency_slot(::ars548_interface::msg::SensorConfiguration::_frequency_slot_type arg)
  {
    msg_.frequency_slot = std::move(arg);
    return Init_SensorConfiguration_cycle_time(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_maximum_distance
{
public:
  explicit Init_SensorConfiguration_maximum_distance(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_frequency_slot maximum_distance(::ars548_interface::msg::SensorConfiguration::_maximum_distance_type arg)
  {
    msg_.maximum_distance = std::move(arg);
    return Init_SensorConfiguration_frequency_slot(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_wheelbase
{
public:
  explicit Init_SensorConfiguration_wheelbase(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_maximum_distance wheelbase(::ars548_interface::msg::SensorConfiguration::_wheelbase_type arg)
  {
    msg_.wheelbase = std::move(arg);
    return Init_SensorConfiguration_maximum_distance(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_height
{
public:
  explicit Init_SensorConfiguration_height(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_wheelbase height(::ars548_interface::msg::SensorConfiguration::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_SensorConfiguration_wheelbase(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_width
{
public:
  explicit Init_SensorConfiguration_width(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_height width(::ars548_interface::msg::SensorConfiguration::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_SensorConfiguration_height(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_length
{
public:
  explicit Init_SensorConfiguration_length(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_width length(::ars548_interface::msg::SensorConfiguration::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_SensorConfiguration_width(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_plug_orientation
{
public:
  explicit Init_SensorConfiguration_plug_orientation(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_length plug_orientation(::ars548_interface::msg::SensorConfiguration::_plug_orientation_type arg)
  {
    msg_.plug_orientation = std::move(arg);
    return Init_SensorConfiguration_length(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_pitch
{
public:
  explicit Init_SensorConfiguration_pitch(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_plug_orientation pitch(::ars548_interface::msg::SensorConfiguration::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_SensorConfiguration_plug_orientation(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_yaw
{
public:
  explicit Init_SensorConfiguration_yaw(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_pitch yaw(::ars548_interface::msg::SensorConfiguration::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_SensorConfiguration_pitch(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_vertical
{
public:
  explicit Init_SensorConfiguration_vertical(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_yaw vertical(::ars548_interface::msg::SensorConfiguration::_vertical_type arg)
  {
    msg_.vertical = std::move(arg);
    return Init_SensorConfiguration_yaw(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_lateral
{
public:
  explicit Init_SensorConfiguration_lateral(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_vertical lateral(::ars548_interface::msg::SensorConfiguration::_lateral_type arg)
  {
    msg_.lateral = std::move(arg);
    return Init_SensorConfiguration_vertical(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_longitudinal
{
public:
  explicit Init_SensorConfiguration_longitudinal(::ars548_interface::msg::SensorConfiguration & msg)
  : msg_(msg)
  {}
  Init_SensorConfiguration_lateral longitudinal(::ars548_interface::msg::SensorConfiguration::_longitudinal_type arg)
  {
    msg_.longitudinal = std::move(arg);
    return Init_SensorConfiguration_lateral(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

class Init_SensorConfiguration_header
{
public:
  Init_SensorConfiguration_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorConfiguration_longitudinal header(::ars548_interface::msg::SensorConfiguration::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SensorConfiguration_longitudinal(msg_);
  }

private:
  ::ars548_interface::msg::SensorConfiguration msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::SensorConfiguration>()
{
  return ars548_interface::msg::builder::Init_SensorConfiguration_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__BUILDER_HPP_
