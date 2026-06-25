// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sensor:msg/ContiRadarInfo.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__CONTI_RADAR_INFO__TRAITS_HPP_
#define SENSOR__MSG__DETAIL__CONTI_RADAR_INFO__TRAITS_HPP_

#include "sensor/msg/detail/conti_radar_info__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'localpose_stamped'
#include "geometry_msgs/msg/detail/pose2_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sensor::msg::ContiRadarInfo>()
{
  return "sensor::msg::ContiRadarInfo";
}

template<>
inline const char * name<sensor::msg::ContiRadarInfo>()
{
  return "sensor/msg/ContiRadarInfo";
}

template<>
struct has_fixed_size<sensor::msg::ContiRadarInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sensor::msg::ContiRadarInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sensor::msg::ContiRadarInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SENSOR__MSG__DETAIL__CONTI_RADAR_INFO__TRAITS_HPP_
