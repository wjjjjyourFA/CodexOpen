// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/CharacteristicSpeed.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__BUILDER_HPP_

#include "ars548_interface/msg/detail/characteristic_speed__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_CharacteristicSpeed_characteristic_speed
{
public:
  explicit Init_CharacteristicSpeed_characteristic_speed(::ars548_interface::msg::CharacteristicSpeed & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::CharacteristicSpeed characteristic_speed(::ars548_interface::msg::CharacteristicSpeed::_characteristic_speed_type arg)
  {
    msg_.characteristic_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::CharacteristicSpeed msg_;
};

class Init_CharacteristicSpeed_qualifier_characteristic_speed
{
public:
  explicit Init_CharacteristicSpeed_qualifier_characteristic_speed(::ars548_interface::msg::CharacteristicSpeed & msg)
  : msg_(msg)
  {}
  Init_CharacteristicSpeed_characteristic_speed qualifier_characteristic_speed(::ars548_interface::msg::CharacteristicSpeed::_qualifier_characteristic_speed_type arg)
  {
    msg_.qualifier_characteristic_speed = std::move(arg);
    return Init_CharacteristicSpeed_characteristic_speed(msg_);
  }

private:
  ::ars548_interface::msg::CharacteristicSpeed msg_;
};

class Init_CharacteristicSpeed_characteristic_speed_err_amp
{
public:
  explicit Init_CharacteristicSpeed_characteristic_speed_err_amp(::ars548_interface::msg::CharacteristicSpeed & msg)
  : msg_(msg)
  {}
  Init_CharacteristicSpeed_qualifier_characteristic_speed characteristic_speed_err_amp(::ars548_interface::msg::CharacteristicSpeed::_characteristic_speed_err_amp_type arg)
  {
    msg_.characteristic_speed_err_amp = std::move(arg);
    return Init_CharacteristicSpeed_qualifier_characteristic_speed(msg_);
  }

private:
  ::ars548_interface::msg::CharacteristicSpeed msg_;
};

class Init_CharacteristicSpeed_header
{
public:
  Init_CharacteristicSpeed_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CharacteristicSpeed_characteristic_speed_err_amp header(::ars548_interface::msg::CharacteristicSpeed::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_CharacteristicSpeed_characteristic_speed_err_amp(msg_);
  }

private:
  ::ars548_interface::msg::CharacteristicSpeed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::CharacteristicSpeed>()
{
  return ars548_interface::msg::builder::Init_CharacteristicSpeed_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__BUILDER_HPP_
