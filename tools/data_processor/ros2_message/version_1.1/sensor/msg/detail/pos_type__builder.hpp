// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sensor:msg/PosType.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__POS_TYPE__BUILDER_HPP_
#define SENSOR__MSG__DETAIL__POS_TYPE__BUILDER_HPP_

#include "sensor/msg/detail/pos_type__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace sensor
{

namespace msg
{

namespace builder
{

class Init_PosType_pos_type
{
public:
  Init_PosType_pos_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sensor::msg::PosType pos_type(::sensor::msg::PosType::_pos_type_type arg)
  {
    msg_.pos_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sensor::msg::PosType msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sensor::msg::PosType>()
{
  return sensor::msg::builder::Init_PosType_pos_type();
}

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__POS_TYPE__BUILDER_HPP_
