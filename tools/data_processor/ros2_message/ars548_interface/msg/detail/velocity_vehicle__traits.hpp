// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ars548_interface:msg/VelocityVehicle.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__TRAITS_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__TRAITS_HPP_

#include "ars548_interface/msg/detail/velocity_vehicle__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ars548_interface::msg::VelocityVehicle>()
{
  return "ars548_interface::msg::VelocityVehicle";
}

template<>
inline const char * name<ars548_interface::msg::VelocityVehicle>()
{
  return "ars548_interface/msg/VelocityVehicle";
}

template<>
struct has_fixed_size<ars548_interface::msg::VelocityVehicle>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ars548_interface::msg::VelocityVehicle>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ars548_interface::msg::VelocityVehicle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__TRAITS_HPP_
