// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from self_state:msg/InsStatus.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__INS_STATUS__BUILDER_HPP_
#define SELF_STATE__MSG__DETAIL__INS_STATUS__BUILDER_HPP_

#include "self_state/msg/detail/ins_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace self_state
{

namespace msg
{

namespace builder
{

class Init_InsStatus_ins_status
{
public:
  Init_InsStatus_ins_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::self_state::msg::InsStatus ins_status(::self_state::msg::InsStatus::_ins_status_type arg)
  {
    msg_.ins_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::self_state::msg::InsStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::self_state::msg::InsStatus>()
{
  return self_state::msg::builder::Init_InsStatus_ins_status();
}

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__INS_STATUS__BUILDER_HPP_
