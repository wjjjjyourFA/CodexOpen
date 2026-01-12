// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/DrivingDirection.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__BUILDER_HPP_

#include "ars548_interface/msg/detail/driving_direction__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_DrivingDirection_driving_direction_confirmed
{
public:
  explicit Init_DrivingDirection_driving_direction_confirmed(::ars548_interface::msg::DrivingDirection & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::DrivingDirection driving_direction_confirmed(::ars548_interface::msg::DrivingDirection::_driving_direction_confirmed_type arg)
  {
    msg_.driving_direction_confirmed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::DrivingDirection msg_;
};

class Init_DrivingDirection_driving_direction_unconfirmed
{
public:
  explicit Init_DrivingDirection_driving_direction_unconfirmed(::ars548_interface::msg::DrivingDirection & msg)
  : msg_(msg)
  {}
  Init_DrivingDirection_driving_direction_confirmed driving_direction_unconfirmed(::ars548_interface::msg::DrivingDirection::_driving_direction_unconfirmed_type arg)
  {
    msg_.driving_direction_unconfirmed = std::move(arg);
    return Init_DrivingDirection_driving_direction_confirmed(msg_);
  }

private:
  ::ars548_interface::msg::DrivingDirection msg_;
};

class Init_DrivingDirection_header
{
public:
  Init_DrivingDirection_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DrivingDirection_driving_direction_unconfirmed header(::ars548_interface::msg::DrivingDirection::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DrivingDirection_driving_direction_unconfirmed(msg_);
  }

private:
  ::ars548_interface::msg::DrivingDirection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::DrivingDirection>()
{
  return ars548_interface::msg::builder::Init_DrivingDirection_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__BUILDER_HPP_
