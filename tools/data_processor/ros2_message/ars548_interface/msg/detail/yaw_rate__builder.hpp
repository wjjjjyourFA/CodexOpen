// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/YawRate.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__YAW_RATE__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__YAW_RATE__BUILDER_HPP_

#include "ars548_interface/msg/detail/yaw_rate__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_YawRate_yaw_rate_event_data_qualifier
{
public:
  explicit Init_YawRate_yaw_rate_event_data_qualifier(::ars548_interface::msg::YawRate & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::YawRate yaw_rate_event_data_qualifier(::ars548_interface::msg::YawRate::_yaw_rate_event_data_qualifier_type arg)
  {
    msg_.yaw_rate_event_data_qualifier = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::YawRate msg_;
};

class Init_YawRate_yaw_rate_invalid_flag
{
public:
  explicit Init_YawRate_yaw_rate_invalid_flag(::ars548_interface::msg::YawRate & msg)
  : msg_(msg)
  {}
  Init_YawRate_yaw_rate_event_data_qualifier yaw_rate_invalid_flag(::ars548_interface::msg::YawRate::_yaw_rate_invalid_flag_type arg)
  {
    msg_.yaw_rate_invalid_flag = std::move(arg);
    return Init_YawRate_yaw_rate_event_data_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::YawRate msg_;
};

class Init_YawRate_yaw_rate
{
public:
  explicit Init_YawRate_yaw_rate(::ars548_interface::msg::YawRate & msg)
  : msg_(msg)
  {}
  Init_YawRate_yaw_rate_invalid_flag yaw_rate(::ars548_interface::msg::YawRate::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return Init_YawRate_yaw_rate_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::YawRate msg_;
};

class Init_YawRate_qualifier_yaw_rate
{
public:
  explicit Init_YawRate_qualifier_yaw_rate(::ars548_interface::msg::YawRate & msg)
  : msg_(msg)
  {}
  Init_YawRate_yaw_rate qualifier_yaw_rate(::ars548_interface::msg::YawRate::_qualifier_yaw_rate_type arg)
  {
    msg_.qualifier_yaw_rate = std::move(arg);
    return Init_YawRate_yaw_rate(msg_);
  }

private:
  ::ars548_interface::msg::YawRate msg_;
};

class Init_YawRate_yaw_rate_err_amp_invalid_flag
{
public:
  explicit Init_YawRate_yaw_rate_err_amp_invalid_flag(::ars548_interface::msg::YawRate & msg)
  : msg_(msg)
  {}
  Init_YawRate_qualifier_yaw_rate yaw_rate_err_amp_invalid_flag(::ars548_interface::msg::YawRate::_yaw_rate_err_amp_invalid_flag_type arg)
  {
    msg_.yaw_rate_err_amp_invalid_flag = std::move(arg);
    return Init_YawRate_qualifier_yaw_rate(msg_);
  }

private:
  ::ars548_interface::msg::YawRate msg_;
};

class Init_YawRate_yaw_rate_err_amp
{
public:
  explicit Init_YawRate_yaw_rate_err_amp(::ars548_interface::msg::YawRate & msg)
  : msg_(msg)
  {}
  Init_YawRate_yaw_rate_err_amp_invalid_flag yaw_rate_err_amp(::ars548_interface::msg::YawRate::_yaw_rate_err_amp_type arg)
  {
    msg_.yaw_rate_err_amp = std::move(arg);
    return Init_YawRate_yaw_rate_err_amp_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::YawRate msg_;
};

class Init_YawRate_header
{
public:
  Init_YawRate_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YawRate_yaw_rate_err_amp header(::ars548_interface::msg::YawRate::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_YawRate_yaw_rate_err_amp(msg_);
  }

private:
  ::ars548_interface::msg::YawRate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::YawRate>()
{
  return ars548_interface::msg::builder::Init_YawRate_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__YAW_RATE__BUILDER_HPP_
