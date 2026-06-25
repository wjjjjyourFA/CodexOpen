// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from self_state:msg/SpeedStatus.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__SPEED_STATUS__BUILDER_HPP_
#define SELF_STATE__MSG__DETAIL__SPEED_STATUS__BUILDER_HPP_

#include "self_state/msg/detail/speed_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace self_state
{

namespace msg
{

namespace builder
{

class Init_SpeedStatus_emergency_lighton_flag
{
public:
  explicit Init_SpeedStatus_emergency_lighton_flag(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  ::self_state::msg::SpeedStatus emergency_lighton_flag(::self_state::msg::SpeedStatus::_emergency_lighton_flag_type arg)
  {
    msg_.emergency_lighton_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_horn_on_flag
{
public:
  explicit Init_SpeedStatus_horn_on_flag(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_emergency_lighton_flag horn_on_flag(::self_state::msg::SpeedStatus::_horn_on_flag_type arg)
  {
    msg_.horn_on_flag = std::move(arg);
    return Init_SpeedStatus_emergency_lighton_flag(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_bcan_control_flag
{
public:
  explicit Init_SpeedStatus_bcan_control_flag(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_horn_on_flag bcan_control_flag(::self_state::msg::SpeedStatus::_bcan_control_flag_type arg)
  {
    msg_.bcan_control_flag = std::move(arg);
    return Init_SpeedStatus_horn_on_flag(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_emergence_flag
{
public:
  explicit Init_SpeedStatus_emergence_flag(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_bcan_control_flag emergence_flag(::self_state::msg::SpeedStatus::_emergence_flag_type arg)
  {
    msg_.emergence_flag = std::move(arg);
    return Init_SpeedStatus_bcan_control_flag(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_hard_switch_on
{
public:
  explicit Init_SpeedStatus_hard_switch_on(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_emergence_flag hard_switch_on(::self_state::msg::SpeedStatus::_hard_switch_on_type arg)
  {
    msg_.hard_switch_on = std::move(arg);
    return Init_SpeedStatus_emergence_flag(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_current_trans_pos
{
public:
  explicit Init_SpeedStatus_current_trans_pos(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_hard_switch_on current_trans_pos(::self_state::msg::SpeedStatus::_current_trans_pos_type arg)
  {
    msg_.current_trans_pos = std::move(arg);
    return Init_SpeedStatus_hard_switch_on(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_desired_trans_pos
{
public:
  explicit Init_SpeedStatus_desired_trans_pos(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_current_trans_pos desired_trans_pos(::self_state::msg::SpeedStatus::_desired_trans_pos_type arg)
  {
    msg_.desired_trans_pos = std::move(arg);
    return Init_SpeedStatus_current_trans_pos(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_current_fuel
{
public:
  explicit Init_SpeedStatus_current_fuel(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_desired_trans_pos current_fuel(::self_state::msg::SpeedStatus::_current_fuel_type arg)
  {
    msg_.current_fuel = std::move(arg);
    return Init_SpeedStatus_desired_trans_pos(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_desired_fuel
{
public:
  explicit Init_SpeedStatus_desired_fuel(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_current_fuel desired_fuel(::self_state::msg::SpeedStatus::_desired_fuel_type arg)
  {
    msg_.desired_fuel = std::move(arg);
    return Init_SpeedStatus_current_fuel(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_current_brake
{
public:
  explicit Init_SpeedStatus_current_brake(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_desired_fuel current_brake(::self_state::msg::SpeedStatus::_current_brake_type arg)
  {
    msg_.current_brake = std::move(arg);
    return Init_SpeedStatus_desired_fuel(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_desired_brake
{
public:
  explicit Init_SpeedStatus_desired_brake(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_current_brake desired_brake(::self_state::msg::SpeedStatus::_desired_brake_type arg)
  {
    msg_.desired_brake = std::move(arg);
    return Init_SpeedStatus_current_brake(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_current_acc
{
public:
  explicit Init_SpeedStatus_current_acc(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_desired_brake current_acc(::self_state::msg::SpeedStatus::_current_acc_type arg)
  {
    msg_.current_acc = std::move(arg);
    return Init_SpeedStatus_desired_brake(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_current_speed
{
public:
  explicit Init_SpeedStatus_current_speed(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_current_acc current_speed(::self_state::msg::SpeedStatus::_current_speed_type arg)
  {
    msg_.current_speed = std::move(arg);
    return Init_SpeedStatus_current_acc(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_desired_acc
{
public:
  explicit Init_SpeedStatus_desired_acc(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_current_speed desired_acc(::self_state::msg::SpeedStatus::_desired_acc_type arg)
  {
    msg_.desired_acc = std::move(arg);
    return Init_SpeedStatus_current_speed(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_desired_speed
{
public:
  explicit Init_SpeedStatus_desired_speed(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_desired_acc desired_speed(::self_state::msg::SpeedStatus::_desired_speed_type arg)
  {
    msg_.desired_speed = std::move(arg);
    return Init_SpeedStatus_desired_acc(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_message_num
{
public:
  explicit Init_SpeedStatus_message_num(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_desired_speed message_num(::self_state::msg::SpeedStatus::_message_num_type arg)
  {
    msg_.message_num = std::move(arg);
    return Init_SpeedStatus_desired_speed(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_utc_time
{
public:
  explicit Init_SpeedStatus_utc_time(::self_state::msg::SpeedStatus & msg)
  : msg_(msg)
  {}
  Init_SpeedStatus_message_num utc_time(::self_state::msg::SpeedStatus::_utc_time_type arg)
  {
    msg_.utc_time = std::move(arg);
    return Init_SpeedStatus_message_num(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

class Init_SpeedStatus_local_time
{
public:
  Init_SpeedStatus_local_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SpeedStatus_utc_time local_time(::self_state::msg::SpeedStatus::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_SpeedStatus_utc_time(msg_);
  }

private:
  ::self_state::msg::SpeedStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::self_state::msg::SpeedStatus>()
{
  return self_state::msg::builder::Init_SpeedStatus_local_time();
}

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__SPEED_STATUS__BUILDER_HPP_
