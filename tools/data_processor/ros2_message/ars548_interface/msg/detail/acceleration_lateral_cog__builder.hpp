// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/AccelerationLateralCog.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__BUILDER_HPP_

#include "ars548_interface/msg/detail/acceleration_lateral_cog__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_AccelerationLateralCog_acceleration_lateral_event_data_qualifier
{
public:
  explicit Init_AccelerationLateralCog_acceleration_lateral_event_data_qualifier(::ars548_interface::msg::AccelerationLateralCog & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::AccelerationLateralCog acceleration_lateral_event_data_qualifier(::ars548_interface::msg::AccelerationLateralCog::_acceleration_lateral_event_data_qualifier_type arg)
  {
    msg_.acceleration_lateral_event_data_qualifier = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLateralCog msg_;
};

class Init_AccelerationLateralCog_acceleration_lateral_invalid_flag
{
public:
  explicit Init_AccelerationLateralCog_acceleration_lateral_invalid_flag(::ars548_interface::msg::AccelerationLateralCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLateralCog_acceleration_lateral_event_data_qualifier acceleration_lateral_invalid_flag(::ars548_interface::msg::AccelerationLateralCog::_acceleration_lateral_invalid_flag_type arg)
  {
    msg_.acceleration_lateral_invalid_flag = std::move(arg);
    return Init_AccelerationLateralCog_acceleration_lateral_event_data_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLateralCog msg_;
};

class Init_AccelerationLateralCog_acceleration_lateral
{
public:
  explicit Init_AccelerationLateralCog_acceleration_lateral(::ars548_interface::msg::AccelerationLateralCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLateralCog_acceleration_lateral_invalid_flag acceleration_lateral(::ars548_interface::msg::AccelerationLateralCog::_acceleration_lateral_type arg)
  {
    msg_.acceleration_lateral = std::move(arg);
    return Init_AccelerationLateralCog_acceleration_lateral_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLateralCog msg_;
};

class Init_AccelerationLateralCog_qualifier_acceleration_lateral
{
public:
  explicit Init_AccelerationLateralCog_qualifier_acceleration_lateral(::ars548_interface::msg::AccelerationLateralCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLateralCog_acceleration_lateral qualifier_acceleration_lateral(::ars548_interface::msg::AccelerationLateralCog::_qualifier_acceleration_lateral_type arg)
  {
    msg_.qualifier_acceleration_lateral = std::move(arg);
    return Init_AccelerationLateralCog_acceleration_lateral(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLateralCog msg_;
};

class Init_AccelerationLateralCog_acceleration_lateral_err_amp_invalid_flag
{
public:
  explicit Init_AccelerationLateralCog_acceleration_lateral_err_amp_invalid_flag(::ars548_interface::msg::AccelerationLateralCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLateralCog_qualifier_acceleration_lateral acceleration_lateral_err_amp_invalid_flag(::ars548_interface::msg::AccelerationLateralCog::_acceleration_lateral_err_amp_invalid_flag_type arg)
  {
    msg_.acceleration_lateral_err_amp_invalid_flag = std::move(arg);
    return Init_AccelerationLateralCog_qualifier_acceleration_lateral(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLateralCog msg_;
};

class Init_AccelerationLateralCog_acceleration_lateral_err_amp
{
public:
  explicit Init_AccelerationLateralCog_acceleration_lateral_err_amp(::ars548_interface::msg::AccelerationLateralCog & msg)
  : msg_(msg)
  {}
  Init_AccelerationLateralCog_acceleration_lateral_err_amp_invalid_flag acceleration_lateral_err_amp(::ars548_interface::msg::AccelerationLateralCog::_acceleration_lateral_err_amp_type arg)
  {
    msg_.acceleration_lateral_err_amp = std::move(arg);
    return Init_AccelerationLateralCog_acceleration_lateral_err_amp_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLateralCog msg_;
};

class Init_AccelerationLateralCog_header
{
public:
  Init_AccelerationLateralCog_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AccelerationLateralCog_acceleration_lateral_err_amp header(::ars548_interface::msg::AccelerationLateralCog::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AccelerationLateralCog_acceleration_lateral_err_amp(msg_);
  }

private:
  ::ars548_interface::msg::AccelerationLateralCog msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::AccelerationLateralCog>()
{
  return ars548_interface::msg::builder::Init_AccelerationLateralCog_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LATERAL_COG__BUILDER_HPP_
