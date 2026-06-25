// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from self_state:msg/ChassisInfo.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__CHASSIS_INFO__TRAITS_HPP_
#define SELF_STATE__MSG__DETAIL__CHASSIS_INFO__TRAITS_HPP_

#include "self_state/msg/detail/chassis_info__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<self_state::msg::ChassisInfo>()
{
  return "self_state::msg::ChassisInfo";
}

template<>
inline const char * name<self_state::msg::ChassisInfo>()
{
  return "self_state/msg/ChassisInfo";
}

template<>
struct has_fixed_size<self_state::msg::ChassisInfo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<self_state::msg::ChassisInfo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<self_state::msg::ChassisInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SELF_STATE__MSG__DETAIL__CHASSIS_INFO__TRAITS_HPP_
