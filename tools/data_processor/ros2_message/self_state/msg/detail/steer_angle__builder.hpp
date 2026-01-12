// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from self_state:msg/SteerAngle.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__STEER_ANGLE__BUILDER_HPP_
#define SELF_STATE__MSG__DETAIL__STEER_ANGLE__BUILDER_HPP_

#include "self_state/msg/detail/steer_angle__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace self_state
{

namespace msg
{

namespace builder
{

class Init_SteerAngle_right_light_flag
{
public:
  explicit Init_SteerAngle_right_light_flag(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  ::self_state::msg::SteerAngle right_light_flag(::self_state::msg::SteerAngle::_right_light_flag_type arg)
  {
    msg_.right_light_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_left_light_flag
{
public:
  explicit Init_SteerAngle_left_light_flag(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  Init_SteerAngle_right_light_flag left_light_flag(::self_state::msg::SteerAngle::_left_light_flag_type arg)
  {
    msg_.left_light_flag = std::move(arg);
    return Init_SteerAngle_right_light_flag(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_bcan_control_flag
{
public:
  explicit Init_SteerAngle_bcan_control_flag(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  Init_SteerAngle_left_light_flag bcan_control_flag(::self_state::msg::SteerAngle::_bcan_control_flag_type arg)
  {
    msg_.bcan_control_flag = std::move(arg);
    return Init_SteerAngle_left_light_flag(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_desired_curvature
{
public:
  explicit Init_SteerAngle_desired_curvature(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  Init_SteerAngle_bcan_control_flag desired_curvature(::self_state::msg::SteerAngle::_desired_curvature_type arg)
  {
    msg_.desired_curvature = std::move(arg);
    return Init_SteerAngle_bcan_control_flag(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_actual_curvature
{
public:
  explicit Init_SteerAngle_actual_curvature(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  Init_SteerAngle_desired_curvature actual_curvature(::self_state::msg::SteerAngle::_actual_curvature_type arg)
  {
    msg_.actual_curvature = std::move(arg);
    return Init_SteerAngle_desired_curvature(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_desired_front_wheel_angle
{
public:
  explicit Init_SteerAngle_desired_front_wheel_angle(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  Init_SteerAngle_actual_curvature desired_front_wheel_angle(::self_state::msg::SteerAngle::_desired_front_wheel_angle_type arg)
  {
    msg_.desired_front_wheel_angle = std::move(arg);
    return Init_SteerAngle_actual_curvature(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_actual_front_wheel_angle
{
public:
  explicit Init_SteerAngle_actual_front_wheel_angle(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  Init_SteerAngle_desired_front_wheel_angle actual_front_wheel_angle(::self_state::msg::SteerAngle::_actual_front_wheel_angle_type arg)
  {
    msg_.actual_front_wheel_angle = std::move(arg);
    return Init_SteerAngle_desired_front_wheel_angle(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_message_num
{
public:
  explicit Init_SteerAngle_message_num(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  Init_SteerAngle_actual_front_wheel_angle message_num(::self_state::msg::SteerAngle::_message_num_type arg)
  {
    msg_.message_num = std::move(arg);
    return Init_SteerAngle_actual_front_wheel_angle(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_utc_time
{
public:
  explicit Init_SteerAngle_utc_time(::self_state::msg::SteerAngle & msg)
  : msg_(msg)
  {}
  Init_SteerAngle_message_num utc_time(::self_state::msg::SteerAngle::_utc_time_type arg)
  {
    msg_.utc_time = std::move(arg);
    return Init_SteerAngle_message_num(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

class Init_SteerAngle_local_time
{
public:
  Init_SteerAngle_local_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SteerAngle_utc_time local_time(::self_state::msg::SteerAngle::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_SteerAngle_utc_time(msg_);
  }

private:
  ::self_state::msg::SteerAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::self_state::msg::SteerAngle>()
{
  return self_state::msg::builder::Init_SteerAngle_local_time();
}

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__STEER_ANGLE__BUILDER_HPP_
