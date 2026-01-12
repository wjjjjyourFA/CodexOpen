// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include "ars548_interface/msg/detail/detection__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_Detection_u_sort_index
{
public:
  explicit Init_Detection_u_sort_index(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::Detection u_sort_index(::ars548_interface::msg::Detection::_u_sort_index_type arg)
  {
    msg_.u_sort_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_u_ambiguity_flag
{
public:
  explicit Init_Detection_u_ambiguity_flag(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_u_sort_index u_ambiguity_flag(::ars548_interface::msg::Detection::_u_ambiguity_flag_type arg)
  {
    msg_.u_ambiguity_flag = std::move(arg);
    return Init_Detection_u_sort_index(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_u_object_id
{
public:
  explicit Init_Detection_u_object_id(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_u_ambiguity_flag u_object_id(::ars548_interface::msg::Detection::_u_object_id_type arg)
  {
    msg_.u_object_id = std::move(arg);
    return Init_Detection_u_ambiguity_flag(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_u_multi_target_probability
{
public:
  explicit Init_Detection_u_multi_target_probability(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_u_object_id u_multi_target_probability(::ars548_interface::msg::Detection::_u_multi_target_probability_type arg)
  {
    msg_.u_multi_target_probability = std::move(arg);
    return Init_Detection_u_object_id(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_u_classification
{
public:
  explicit Init_Detection_u_classification(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_u_multi_target_probability u_classification(::ars548_interface::msg::Detection::_u_classification_type arg)
  {
    msg_.u_classification = std::move(arg);
    return Init_Detection_u_multi_target_probability(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_u_positive_predictive_value
{
public:
  explicit Init_Detection_u_positive_predictive_value(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_u_classification u_positive_predictive_value(::ars548_interface::msg::Detection::_u_positive_predictive_value_type arg)
  {
    msg_.u_positive_predictive_value = std::move(arg);
    return Init_Detection_u_classification(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_u_measurement_id
{
public:
  explicit Init_Detection_u_measurement_id(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_u_positive_predictive_value u_measurement_id(::ars548_interface::msg::Detection::_u_measurement_id_type arg)
  {
    msg_.u_measurement_id = std::move(arg);
    return Init_Detection_u_positive_predictive_value(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_s_rcs
{
public:
  explicit Init_Detection_s_rcs(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_u_measurement_id s_rcs(::ars548_interface::msg::Detection::_s_rcs_type arg)
  {
    msg_.s_rcs = std::move(arg);
    return Init_Detection_u_measurement_id(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_range_rate_std
{
public:
  explicit Init_Detection_f_range_rate_std(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_s_rcs f_range_rate_std(::ars548_interface::msg::Detection::_f_range_rate_std_type arg)
  {
    msg_.f_range_rate_std = std::move(arg);
    return Init_Detection_s_rcs(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_range_rate
{
public:
  explicit Init_Detection_f_range_rate(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_range_rate_std f_range_rate(::ars548_interface::msg::Detection::_f_range_rate_type arg)
  {
    msg_.f_range_rate = std::move(arg);
    return Init_Detection_f_range_rate_std(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_range_std
{
public:
  explicit Init_Detection_f_range_std(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_range_rate f_range_std(::ars548_interface::msg::Detection::_f_range_std_type arg)
  {
    msg_.f_range_std = std::move(arg);
    return Init_Detection_f_range_rate(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_range
{
public:
  explicit Init_Detection_f_range(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_range_std f_range(::ars548_interface::msg::Detection::_f_range_type arg)
  {
    msg_.f_range = std::move(arg);
    return Init_Detection_f_range_std(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_elevation_angle_std
{
public:
  explicit Init_Detection_f_elevation_angle_std(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_range f_elevation_angle_std(::ars548_interface::msg::Detection::_f_elevation_angle_std_type arg)
  {
    msg_.f_elevation_angle_std = std::move(arg);
    return Init_Detection_f_range(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_elevation_angle
{
public:
  explicit Init_Detection_f_elevation_angle(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_elevation_angle_std f_elevation_angle(::ars548_interface::msg::Detection::_f_elevation_angle_type arg)
  {
    msg_.f_elevation_angle = std::move(arg);
    return Init_Detection_f_elevation_angle_std(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_u_invalid_flags
{
public:
  explicit Init_Detection_u_invalid_flags(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_elevation_angle u_invalid_flags(::ars548_interface::msg::Detection::_u_invalid_flags_type arg)
  {
    msg_.u_invalid_flags = std::move(arg);
    return Init_Detection_f_elevation_angle(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_azimuth_angle_std
{
public:
  explicit Init_Detection_f_azimuth_angle_std(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_u_invalid_flags f_azimuth_angle_std(::ars548_interface::msg::Detection::_f_azimuth_angle_std_type arg)
  {
    msg_.f_azimuth_angle_std = std::move(arg);
    return Init_Detection_u_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_azimuth_angle
{
public:
  explicit Init_Detection_f_azimuth_angle(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_azimuth_angle_std f_azimuth_angle(::ars548_interface::msg::Detection::_f_azimuth_angle_type arg)
  {
    msg_.f_azimuth_angle = std::move(arg);
    return Init_Detection_f_azimuth_angle_std(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_z
{
public:
  explicit Init_Detection_f_z(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_azimuth_angle f_z(::ars548_interface::msg::Detection::_f_z_type arg)
  {
    msg_.f_z = std::move(arg);
    return Init_Detection_f_azimuth_angle(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_y
{
public:
  explicit Init_Detection_f_y(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_z f_y(::ars548_interface::msg::Detection::_f_y_type arg)
  {
    msg_.f_y = std::move(arg);
    return Init_Detection_f_z(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_f_x
{
public:
  explicit Init_Detection_f_x(::ars548_interface::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_f_y f_x(::ars548_interface::msg::Detection::_f_x_type arg)
  {
    msg_.f_x = std::move(arg);
    return Init_Detection_f_y(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

class Init_Detection_header
{
public:
  Init_Detection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_f_x header(::ars548_interface::msg::Detection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Detection_f_x(msg_);
  }

private:
  ::ars548_interface::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::Detection>()
{
  return ars548_interface::msg::builder::Init_Detection_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__DETECTION__BUILDER_HPP_
