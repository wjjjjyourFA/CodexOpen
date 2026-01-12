// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/AccelerationLongitudinalCog.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LONGITUDINAL_COG__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LONGITUDINAL_COG__BUILDER_HPP_

#include "ars548_interface/msg/detail/acceleration_longitudinal_cog__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_AccelerationLongitudinalCog_acceleration_longitudinal_event_data_qualifier
{
public:
  explicit Init_AccelerationLongitudinalCog_acceleration_longitudinal_event_data_qualifier(::ars548_interface::msg::AccelerationLongitudinalCog & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::AccelerationLongitudinalCog acceleration_longitudinal_event_data_qualifier(::ars548_interface::msg::AccelerationLongitudinalCog::_acceleration_longitudinal_event_data_qualifier_type arg)
  {
    msg_.acceleration_longitudinal_event_data_qualifier = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLongitudinalCog msg_;
};

class Init_AccelerationLongitudinalCog_acceleration_longitudinal_invalid_flag
{
public:
  explicit Init_AccelerationLongitudinalCog_acceleration_longitudinal_invalid_flag(::ars548_interface::msg::AccelerationLongitudinalCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLongitudinalCog_acceleration_longitudinal_event_data_qualifier acceleration_longitudinal_invalid_flag(::ars548_interface::msg::AccelerationLongitudinalCog::_acceleration_longitudinal_invalid_flag_type arg)
  {
    msg_.acceleration_longitudinal_invalid_flag = std::move(arg);
    return Init_AccelerationLongitudinalCog_acceleration_longitudinal_event_data_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLongitudinalCog msg_;
};

class Init_AccelerationLongitudinalCog_acceleration_longitudinal
{
public:
  explicit Init_AccelerationLongitudinalCog_acceleration_longitudinal(::ars548_interface::msg::AccelerationLongitudinalCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLongitudinalCog_acceleration_longitudinal_invalid_flag acceleration_longitudinal(::ars548_interface::msg::AccelerationLongitudinalCog::_acceleration_longitudinal_type arg)
  {
    msg_.acceleration_longitudinal = std::move(arg);
    return Init_AccelerationLongitudinalCog_acceleration_longitudinal_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLongitudinalCog msg_;
};

class Init_AccelerationLongitudinalCog_qualifier_acceleration_longitudinal
{
public:
  explicit Init_AccelerationLongitudinalCog_qualifier_acceleration_longitudinal(::ars548_interface::msg::AccelerationLongitudinalCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLongitudinalCog_acceleration_longitudinal qualifier_acceleration_longitudinal(::ars548_interface::msg::AccelerationLongitudinalCog::_qualifier_acceleration_longitudinal_type arg)
  {
    msg_.qualifier_acceleration_longitudinal = std::move(arg);
    return Init_AccelerationLongitudinalCog_acceleration_longitudinal(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLongitudinalCog msg_;
};

class Init_AccelerationLongitudinalCog_acceleration_longitudinal_err_amp_invalid_flag
{
public:
  explicit Init_AccelerationLongitudinalCog_acceleration_longitudinal_err_amp_invalid_flag(::ars548_interface::msg::AccelerationLongitudinalCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLongitudinalCog_qualifier_acceleration_longitudinal acceleration_longitudinal_err_amp_invalid_flag(::ars548_interface::msg::AccelerationLongitudinalCog::_acceleration_longitudinal_err_amp_invalid_flag_type arg)
  {
    msg_.acceleration_longitudinal_err_amp_invalid_flag = std::move(arg);
    return Init_AccelerationLongitudinalCog_qualifier_acceleration_longitudinal(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLongitudinalCog msg_;
};

class Init_AccelerationLongitudinalCog_acceleration_longitudinal_err_amp
{
public:
  explicit Init_AccelerationLongitudinalCog_acceleration_longitudinal_err_amp(::ars548_interface::msg::AccelerationLongitudinalCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLongitudinalCog_acceleration_longitudinal_err_amp_invalid_flag acceleration_longitudinal_err_amp(::ars548_interface::msg::AccelerationLongitudinalCog::_acceleration_longitudinal_err_amp_type arg)
  {
    msg_.acceleration_longitudinal_err_amp = std::move(arg);
    return Init_AccelerationLongitudinalCog_acceleration_longitudinal_err_amp_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLongitudinalCog msg_;
};

class Init_AccelerationLongitudinalCog_header
{
public:
  Init_AccelerationLongitudinalCog_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AccelerationLongitudinalCog_acceleration_longitudinal_err_amp header(::ars548_interface::msg::AccelerationLongitudinalCog::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AccelerationLongitudinalCog_acceleration_longitudinal_err_amp(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLongitudinalCog msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::AccelerationLongitudinalCog>()
{
  return ars548_interface::msg::builder::Init_AccelerationLongitudinalCog_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LONGITUDINAL_COG__BUILDER_HPP_
