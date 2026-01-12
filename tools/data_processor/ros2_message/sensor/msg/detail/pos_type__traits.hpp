// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sensor:msg/PosType.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__POS_TYPE__TRAITS_HPP_
#define SENSOR__MSG__DETAIL__POS_TYPE__TRAITS_HPP_

#include "sensor/msg/detail/pos_type__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sensor::msg::PosType>()
{
  return "sensor::msg::PosType";
}

template<>
inline const char * name<sensor::msg::PosType>()
{
  return "sensor/msg/PosType";
}

template<>
struct has_fixed_size<sensor::msg::PosType>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sensor::msg::PosType>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sensor::msg::PosType>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SENSOR__MSG__DETAIL__POS_TYPE__TRAITS_HPP_
