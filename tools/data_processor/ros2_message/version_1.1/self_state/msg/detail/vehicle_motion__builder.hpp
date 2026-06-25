// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from self_state:msg/VehicleMotion.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__BUILDER_HPP_
#define SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__BUILDER_HPP_

#include "self_state/msg/detail/vehicle_motion__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace self_state
{

namespace msg
{

namespace builder
{

class Init_VehicleMotion_acc_z
{
public:
  explicit Init_VehicleMotion_acc_z(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  ::self_state::msg::VehicleMotion acc_z(::self_state::msg::VehicleMotion::_acc_z_type arg)
  {
    msg_.acc_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_acc_y
{
public:
  explicit Init_VehicleMotion_acc_y(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_acc_z acc_y(::self_state::msg::VehicleMotion::_acc_y_type arg)
  {
    msg_.acc_y = std::move(arg);
    return Init_VehicleMotion_acc_z(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_acc_x
{
public:
  explicit Init_VehicleMotion_acc_x(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_acc_y acc_x(::self_state::msg::VehicleMotion::_acc_x_type arg)
  {
    msg_.acc_x = std::move(arg);
    return Init_VehicleMotion_acc_y(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_angular_z
{
public:
  explicit Init_VehicleMotion_angular_z(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_acc_x angular_z(::self_state::msg::VehicleMotion::_angular_z_type arg)
  {
    msg_.angular_z = std::move(arg);
    return Init_VehicleMotion_acc_x(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_angular_y
{
public:
  explicit Init_VehicleMotion_angular_y(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_angular_z angular_y(::self_state::msg::VehicleMotion::_angular_y_type arg)
  {
    msg_.angular_y = std::move(arg);
    return Init_VehicleMotion_angular_z(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_angular_x
{
public:
  explicit Init_VehicleMotion_angular_x(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_angular_y angular_x(::self_state::msg::VehicleMotion::_angular_x_type arg)
  {
    msg_.angular_x = std::move(arg);
    return Init_VehicleMotion_angular_y(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_vertical_speed
{
public:
  explicit Init_VehicleMotion_vertical_speed(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_angular_x vertical_speed(::self_state::msg::VehicleMotion::_vertical_speed_type arg)
  {
    msg_.vertical_speed = std::move(arg);
    return Init_VehicleMotion_angular_x(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_lateral_speed
{
public:
  explicit Init_VehicleMotion_lateral_speed(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_vertical_speed lateral_speed(::self_state::msg::VehicleMotion::_lateral_speed_type arg)
  {
    msg_.lateral_speed = std::move(arg);
    return Init_VehicleMotion_vertical_speed(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_longitudinal_speed
{
public:
  explicit Init_VehicleMotion_longitudinal_speed(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_lateral_speed longitudinal_speed(::self_state::msg::VehicleMotion::_longitudinal_speed_type arg)
  {
    msg_.longitudinal_speed = std::move(arg);
    return Init_VehicleMotion_lateral_speed(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_message_num
{
public:
  explicit Init_VehicleMotion_message_num(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_longitudinal_speed message_num(::self_state::msg::VehicleMotion::_message_num_type arg)
  {
    msg_.message_num = std::move(arg);
    return Init_VehicleMotion_longitudinal_speed(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_utc_time
{
public:
  explicit Init_VehicleMotion_utc_time(::self_state::msg::VehicleMotion & msg)
  : msg_(msg)
  {}
  Init_VehicleMotion_message_num utc_time(::self_state::msg::VehicleMotion::_utc_time_type arg)
  {
    msg_.utc_time = std::move(arg);
    return Init_VehicleMotion_message_num(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

class Init_VehicleMotion_local_time
{
public:
  Init_VehicleMotion_local_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleMotion_utc_time local_time(::self_state::msg::VehicleMotion::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_VehicleMotion_utc_time(msg_);
  }

private:
  ::self_state::msg::VehicleMotion msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::self_state::msg::VehicleMotion>()
{
  return self_state::msg::builder::Init_VehicleMotion_local_time();
}

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__BUILDER_HPP_
