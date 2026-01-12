// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from self_state:msg/SteerAngle.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__STEER_ANGLE__TRAITS_HPP_
#define SELF_STATE__MSG__DETAIL__STEER_ANGLE__TRAITS_HPP_

#include "self_state/msg/detail/steer_angle__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<self_state::msg::SteerAngle>()
{
  return "self_state::msg::SteerAngle";
}

template<>
inline const char * name<self_state::msg::SteerAngle>()
{
  return "self_state/msg/SteerAngle";
}

template<>
struct has_fixed_size<self_state::msg::SteerAngle>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<self_state::msg::SteerAngle>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<self_state::msg::SteerAngle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SELF_STATE__MSG__DETAIL__STEER_ANGLE__TRAITS_HPP_
