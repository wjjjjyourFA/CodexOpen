// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from self_state:msg/GlobalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__GLOBAL_POSE__TRAITS_HPP_
#define SELF_STATE__MSG__DETAIL__GLOBAL_POSE__TRAITS_HPP_

#include "self_state/msg/detail/global_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'ins_status'
#include "self_state/msg/detail/ins_status__traits.hpp"
// Member 'pos_type'
#include "sensor/msg/detail/pos_type__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<self_state::msg::GlobalPose>()
{
  return "self_state::msg::GlobalPose";
}

template<>
inline const char * name<self_state::msg::GlobalPose>()
{
  return "self_state/msg/GlobalPose";
}

template<>
struct has_fixed_size<self_state::msg::GlobalPose>
  : std::integral_constant<bool, has_fixed_size<self_state::msg::InsStatus>::value && has_fixed_size<sensor::msg::PosType>::value> {};

template<>
struct has_bounded_size<self_state::msg::GlobalPose>
  : std::integral_constant<bool, has_bounded_size<self_state::msg::InsStatus>::value && has_bounded_size<sensor::msg::PosType>::value> {};

template<>
struct is_message<self_state::msg::GlobalPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SELF_STATE__MSG__DETAIL__GLOBAL_POSE__TRAITS_HPP_
