// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from self_state:msg/LocalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__LOCAL_POSE__BUILDER_HPP_
#define SELF_STATE__MSG__DETAIL__LOCAL_POSE__BUILDER_HPP_

#include "self_state/msg/detail/local_pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace self_state
{

namespace msg
{

namespace builder
{

class Init_LocalPose_reserved
{
public:
  explicit Init_LocalPose_reserved(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  ::self_state::msg::LocalPose reserved(::self_state::msg::LocalPose::_reserved_type arg)
  {
    msg_.reserved = std::move(arg);
    return std::move(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_driving_direction
{
public:
  explicit Init_LocalPose_driving_direction(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_reserved driving_direction(::self_state::msg::LocalPose::_driving_direction_type arg)
  {
    msg_.driving_direction = std::move(arg);
    return Init_LocalPose_reserved(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_speed_z
{
public:
  explicit Init_LocalPose_speed_z(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_driving_direction speed_z(::self_state::msg::LocalPose::_speed_z_type arg)
  {
    msg_.speed_z = std::move(arg);
    return Init_LocalPose_driving_direction(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_speed_y
{
public:
  explicit Init_LocalPose_speed_y(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_speed_z speed_y(::self_state::msg::LocalPose::_speed_y_type arg)
  {
    msg_.speed_y = std::move(arg);
    return Init_LocalPose_speed_z(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_speed_x
{
public:
  explicit Init_LocalPose_speed_x(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_speed_y speed_x(::self_state::msg::LocalPose::_speed_x_type arg)
  {
    msg_.speed_x = std::move(arg);
    return Init_LocalPose_speed_y(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_vehicle_speed
{
public:
  explicit Init_LocalPose_vehicle_speed(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_speed_x vehicle_speed(::self_state::msg::LocalPose::_vehicle_speed_type arg)
  {
    msg_.vehicle_speed = std::move(arg);
    return Init_LocalPose_speed_x(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_dr_heading
{
public:
  explicit Init_LocalPose_dr_heading(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_vehicle_speed dr_heading(::self_state::msg::LocalPose::_dr_heading_type arg)
  {
    msg_.dr_heading = std::move(arg);
    return Init_LocalPose_vehicle_speed(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_dr_pitch
{
public:
  explicit Init_LocalPose_dr_pitch(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_dr_heading dr_pitch(::self_state::msg::LocalPose::_dr_pitch_type arg)
  {
    msg_.dr_pitch = std::move(arg);
    return Init_LocalPose_dr_heading(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_dr_roll
{
public:
  explicit Init_LocalPose_dr_roll(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_dr_pitch dr_roll(::self_state::msg::LocalPose::_dr_roll_type arg)
  {
    msg_.dr_roll = std::move(arg);
    return Init_LocalPose_dr_pitch(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_dr_z
{
public:
  explicit Init_LocalPose_dr_z(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_dr_roll dr_z(::self_state::msg::LocalPose::_dr_z_type arg)
  {
    msg_.dr_z = std::move(arg);
    return Init_LocalPose_dr_roll(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_dr_y
{
public:
  explicit Init_LocalPose_dr_y(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_dr_z dr_y(::self_state::msg::LocalPose::_dr_y_type arg)
  {
    msg_.dr_y = std::move(arg);
    return Init_LocalPose_dr_z(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_dr_x
{
public:
  explicit Init_LocalPose_dr_x(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_dr_y dr_x(::self_state::msg::LocalPose::_dr_x_type arg)
  {
    msg_.dr_x = std::move(arg);
    return Init_LocalPose_dr_y(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_message_num
{
public:
  explicit Init_LocalPose_message_num(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_dr_x message_num(::self_state::msg::LocalPose::_message_num_type arg)
  {
    msg_.message_num = std::move(arg);
    return Init_LocalPose_dr_x(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_utc_time
{
public:
  explicit Init_LocalPose_utc_time(::self_state::msg::LocalPose & msg)
  : msg_(msg)
  {}
  Init_LocalPose_message_num utc_time(::self_state::msg::LocalPose::_utc_time_type arg)
  {
    msg_.utc_time = std::move(arg);
    return Init_LocalPose_message_num(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

class Init_LocalPose_local_time
{
public:
  Init_LocalPose_local_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalPose_utc_time local_time(::self_state::msg::LocalPose::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_LocalPose_utc_time(msg_);
  }

private:
  ::self_state::msg::LocalPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::self_state::msg::LocalPose>()
{
  return self_state::msg::builder::Init_LocalPose_local_time();
}

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__LOCAL_POSE__BUILDER_HPP_
