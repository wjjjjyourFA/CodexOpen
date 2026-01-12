// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/VelocityVehicle.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__BUILDER_HPP_

#include "ars548_interface/msg/detail/velocity_vehicle__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_VelocityVehicle_velocity_vehicle_invalid_flag
{
public:
  explicit Init_VelocityVehicle_velocity_vehicle_invalid_flag(::ars548_interface::msg::VelocityVehicle & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::VelocityVehicle velocity_vehicle_invalid_flag(::ars548_interface::msg::VelocityVehicle::_velocity_vehicle_invalid_flag_type arg)
  {
    msg_.velocity_vehicle_invalid_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::VelocityVehicle msg_;
};

class Init_VelocityVehicle_velocity_vehicle
{
public:
  explicit Init_VelocityVehicle_velocity_vehicle(::ars548_interface::msg::VelocityVehicle & msg)
  : msg_(msg)
  {}
  Init_VelocityVehicle_velocity_vehicle_invalid_flag velocity_vehicle(::ars548_interface::msg::VelocityVehicle::_velocity_vehicle_type arg)
  {
    msg_.velocity_vehicle = std::move(arg);
    return Init_VelocityVehicle_velocity_vehicle_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::VelocityVehicle msg_;
};

class Init_VelocityVehicle_velocity_vehicle_event_data_qualifier
{
public:
  explicit Init_VelocityVehicle_velocity_vehicle_event_data_qualifier(::ars548_interface::msg::VelocityVehicle & msg)
  : msg_(msg)
  {}
  Init_VelocityVehicle_velocity_vehicle velocity_vehicle_event_data_qualifier(::ars548_interface::msg::VelocityVehicle::_velocity_vehicle_event_data_qualifier_type arg)
  {
    msg_.velocity_vehicle_event_data_qualifier = std::move(arg);
    return Init_VelocityVehicle_velocity_vehicle(msg_);
  }

private:
  ::ars548_interface::msg::VelocityVehicle msg_;
};

class Init_VelocityVehicle_qualifier_velocity_vehicle
{
public:
  explicit Init_VelocityVehicle_qualifier_velocity_vehicle(::ars548_interface::msg::VelocityVehicle & msg)
  : msg_(msg)
  {}
  Init_VelocityVehicle_velocity_vehicle_event_data_qualifier qualifier_velocity_vehicle(::ars548_interface::msg::VelocityVehicle::_qualifier_velocity_vehicle_type arg)
  {
    msg_.qualifier_velocity_vehicle = std::move(arg);
    return Init_VelocityVehicle_velocity_vehicle_event_data_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::VelocityVehicle msg_;
};

class Init_VelocityVehicle_status_velocity_near_standstill
{
public:
  explicit Init_VelocityVehicle_status_velocity_near_standstill(::ars548_interface::msg::VelocityVehicle & msg)
  : msg_(msg)
  {}
  Init_VelocityVehicle_qualifier_velocity_vehicle status_velocity_near_standstill(::ars548_interface::msg::VelocityVehicle::_status_velocity_near_standstill_type arg)
  {
    msg_.status_velocity_near_standstill = std::move(arg);
    return Init_VelocityVehicle_qualifier_velocity_vehicle(msg_);
  }

private:
  ::ars548_interface::msg::VelocityVehicle msg_;
};

class Init_VelocityVehicle_header
{
public:
  Init_VelocityVehicle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelocityVehicle_status_velocity_near_standstill header(::ars548_interface::msg::VelocityVehicle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VelocityVehicle_status_velocity_near_standstill(msg_);
  }

private:
  ::ars548_interface::msg::VelocityVehicle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::VelocityVehicle>()
{
  return ars548_interface::msg::builder::Init_VelocityVehicle_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__BUILDER_HPP_
