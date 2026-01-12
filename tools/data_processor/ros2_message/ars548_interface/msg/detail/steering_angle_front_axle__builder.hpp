// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/SteeringAngleFrontAxle.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__BUILDER_HPP_

#include "ars548_interface/msg/detail/steering_angle_front_axle__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_SteeringAngleFrontAxle_steering_angle_front_axle_event_data_qualifier
{
public:
  explicit Init_SteeringAngleFrontAxle_steering_angle_front_axle_event_data_qualifier(::ars548_interface::msg::SteeringAngleFrontAxle & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::SteeringAngleFrontAxle steering_angle_front_axle_event_data_qualifier(::ars548_interface::msg::SteeringAngleFrontAxle::_steering_angle_front_axle_event_data_qualifier_type arg)
  {
    msg_.steering_angle_front_axle_event_data_qualifier = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::SteeringAngleFrontAxle msg_;
};

class Init_SteeringAngleFrontAxle_steering_angle_front_axle_invalid_flag
{
public:
  explicit Init_SteeringAngleFrontAxle_steering_angle_front_axle_invalid_flag(::ars548_interface::msg::SteeringAngleFrontAxle & msg)
  : msg_(msg)
  {}
  Init_SteeringAngleFrontAxle_steering_angle_front_axle_event_data_qualifier steering_angle_front_axle_invalid_flag(::ars548_interface::msg::SteeringAngleFrontAxle::_steering_angle_front_axle_invalid_flag_type arg)
  {
    msg_.steering_angle_front_axle_invalid_flag = std::move(arg);
    return Init_SteeringAngleFrontAxle_steering_angle_front_axle_event_data_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::SteeringAngleFrontAxle msg_;
};

class Init_SteeringAngleFrontAxle_steering_angle_front_axle
{
public:
  explicit Init_SteeringAngleFrontAxle_steering_angle_front_axle(::ars548_interface::msg::SteeringAngleFrontAxle & msg)
  : msg_(msg)
  {}
  Init_SteeringAngleFrontAxle_steering_angle_front_axle_invalid_flag steering_angle_front_axle(::ars548_interface::msg::SteeringAngleFrontAxle::_steering_angle_front_axle_type arg)
  {
    msg_.steering_angle_front_axle = std::move(arg);
    return Init_SteeringAngleFrontAxle_steering_angle_front_axle_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::SteeringAngleFrontAxle msg_;
};

class Init_SteeringAngleFrontAxle_steering_angle_front_axle_err_amp_invalid_flag
{
public:
  explicit Init_SteeringAngleFrontAxle_steering_angle_front_axle_err_amp_invalid_flag(::ars548_interface::msg::SteeringAngleFrontAxle & msg)
  : msg_(msg)
  {}
  Init_SteeringAngleFrontAxle_steering_angle_front_axle steering_angle_front_axle_err_amp_invalid_flag(::ars548_interface::msg::SteeringAngleFrontAxle::_steering_angle_front_axle_err_amp_invalid_flag_type arg)
  {
    msg_.steering_angle_front_axle_err_amp_invalid_flag = std::move(arg);
    return Init_SteeringAngleFrontAxle_steering_angle_front_axle(msg_);
  }

private:
  ::ars548_interface::msg::SteeringAngleFrontAxle msg_;
};

class Init_SteeringAngleFrontAxle_steering_angle_front_axle_err_amp
{
public:
  explicit Init_SteeringAngleFrontAxle_steering_angle_front_axle_err_amp(::ars548_interface::msg::SteeringAngleFrontAxle & msg)
  : msg_(msg)
  {}
  Init_SteeringAngleFrontAxle_steering_angle_front_axle_err_amp_invalid_flag steering_angle_front_axle_err_amp(::ars548_interface::msg::SteeringAngleFrontAxle::_steering_angle_front_axle_err_amp_type arg)
  {
    msg_.steering_angle_front_axle_err_amp = std::move(arg);
    return Init_SteeringAngleFrontAxle_steering_angle_front_axle_err_amp_invalid_flag(msg_);
  }

private:
  ::ars548_interface::msg::SteeringAngleFrontAxle msg_;
};

class Init_SteeringAngleFrontAxle_qualifier_steering_angle_front_axle
{
public:
  explicit Init_SteeringAngleFrontAxle_qualifier_steering_angle_front_axle(::ars548_interface::msg::SteeringAngleFrontAxle & msg)
  : msg_(msg)
  {}
  Init_SteeringAngleFrontAxle_steering_angle_front_axle_err_amp qualifier_steering_angle_front_axle(::ars548_interface::msg::SteeringAngleFrontAxle::_qualifier_steering_angle_front_axle_type arg)
  {
    msg_.qualifier_steering_angle_front_axle = std::move(arg);
    return Init_SteeringAngleFrontAxle_steering_angle_front_axle_err_amp(msg_);
  }

private:
  ::ars548_interface::msg::SteeringAngleFrontAxle msg_;
};

class Init_SteeringAngleFrontAxle_header
{
public:
  Init_SteeringAngleFrontAxle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SteeringAngleFrontAxle_qualifier_steering_angle_front_axle header(::ars548_interface::msg::SteeringAngleFrontAxle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SteeringAngleFrontAxle_qualifier_steering_angle_front_axle(msg_);
  }

private:
  ::ars548_interface::msg::SteeringAngleFrontAxle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::SteeringAngleFrontAxle>()
{
  return ars548_interface::msg::builder::Init_SteeringAngleFrontAxle_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__BUILDER_HPP_
