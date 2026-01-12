// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from self_state:msg/LidarLocalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__LIDAR_LOCAL_POSE__TRAITS_HPP_
#define SELF_STATE__MSG__DETAIL__LIDAR_LOCAL_POSE__TRAITS_HPP_

#include "self_state/msg/detail/lidar_local_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'local_pose'
#include "self_state/msg/detail/local_pose__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<self_state::msg::LidarLocalPose>()
{
  return "self_state::msg::LidarLocalPose";
}

template<>
inline const char * name<self_state::msg::LidarLocalPose>()
{
  return "self_state/msg/LidarLocalPose";
}

template<>
struct has_fixed_size<self_state::msg::LidarLocalPose>
  : std::integral_constant<bool, has_fixed_size<self_state::msg::LocalPose>::value> {};

template<>
struct has_bounded_size<self_state::msg::LidarLocalPose>
  : std::integral_constant<bool, has_bounded_size<self_state::msg::LocalPose>::value> {};

template<>
struct is_message<self_state::msg::LidarLocalPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SELF_STATE__MSG__DETAIL__LIDAR_LOCAL_POSE__TRAITS_HPP_
