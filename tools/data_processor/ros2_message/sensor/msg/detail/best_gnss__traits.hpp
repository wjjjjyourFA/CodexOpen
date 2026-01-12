// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sensor:msg/BestGnss.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__BEST_GNSS__TRAITS_HPP_
#define SENSOR__MSG__DETAIL__BEST_GNSS__TRAITS_HPP_

#include "sensor/msg/detail/best_gnss__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'sol_status'
#include "sensor/msg/detail/gnss_solution_status__traits.hpp"
// Member 'pos_type'
#include "sensor/msg/detail/pos_type__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sensor::msg::BestGnss>()
{
  return "sensor::msg::BestGnss";
}

template<>
inline const char * name<sensor::msg::BestGnss>()
{
  return "sensor/msg/BestGnss";
}

template<>
struct has_fixed_size<sensor::msg::BestGnss>
  : std::integral_constant<bool, has_fixed_size<sensor::msg::GnssSolutionStatus>::value && has_fixed_size<sensor::msg::PosType>::value> {};

template<>
struct has_bounded_size<sensor::msg::BestGnss>
  : std::integral_constant<bool, has_bounded_size<sensor::msg::GnssSolutionStatus>::value && has_bounded_size<sensor::msg::PosType>::value> {};

template<>
struct is_message<sensor::msg::BestGnss>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SENSOR__MSG__DETAIL__BEST_GNSS__TRAITS_HPP_
