// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from self_state:msg/LidarGlobalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__LIDAR_GLOBAL_POSE__BUILDER_HPP_
#define SELF_STATE__MSG__DETAIL__LIDAR_GLOBAL_POSE__BUILDER_HPP_

#include "self_state/msg/detail/lidar_global_pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace self_state
{

namespace msg
{

namespace builder
{

class Init_LidarGlobalPose_local_pose
{
public:
  explicit Init_LidarGlobalPose_local_pose(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  ::self_state::msg::LidarGlobalPose local_pose(::self_state::msg::LidarGlobalPose::_local_pose_type arg)
  {
    msg_.local_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_roll
{
public:
  explicit Init_LidarGlobalPose_roll(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_local_pose roll(::self_state::msg::LidarGlobalPose::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_LidarGlobalPose_local_pose(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_pitch
{
public:
  explicit Init_LidarGlobalPose_pitch(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_roll pitch(::self_state::msg::LidarGlobalPose::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_LidarGlobalPose_roll(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_azimuth
{
public:
  explicit Init_LidarGlobalPose_azimuth(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_pitch azimuth(::self_state::msg::LidarGlobalPose::_azimuth_type arg)
  {
    msg_.azimuth = std::move(arg);
    return Init_LidarGlobalPose_pitch(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_z_speed
{
public:
  explicit Init_LidarGlobalPose_z_speed(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_azimuth z_speed(::self_state::msg::LidarGlobalPose::_z_speed_type arg)
  {
    msg_.z_speed = std::move(arg);
    return Init_LidarGlobalPose_azimuth(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_y_speed
{
public:
  explicit Init_LidarGlobalPose_y_speed(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_z_speed y_speed(::self_state::msg::LidarGlobalPose::_y_speed_type arg)
  {
    msg_.y_speed = std::move(arg);
    return Init_LidarGlobalPose_z_speed(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_x_speed
{
public:
  explicit Init_LidarGlobalPose_x_speed(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_y_speed x_speed(::self_state::msg::LidarGlobalPose::_x_speed_type arg)
  {
    msg_.x_speed = std::move(arg);
    return Init_LidarGlobalPose_y_speed(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_z
{
public:
  explicit Init_LidarGlobalPose_z(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_x_speed z(::self_state::msg::LidarGlobalPose::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_LidarGlobalPose_x_speed(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_y
{
public:
  explicit Init_LidarGlobalPose_y(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_z y(::self_state::msg::LidarGlobalPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_LidarGlobalPose_z(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_x
{
public:
  explicit Init_LidarGlobalPose_x(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_y x(::self_state::msg::LidarGlobalPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_LidarGlobalPose_y(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_pos_type
{
public:
  explicit Init_LidarGlobalPose_pos_type(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_x pos_type(::self_state::msg::LidarGlobalPose::_pos_type_type arg)
  {
    msg_.pos_type = std::move(arg);
    return Init_LidarGlobalPose_x(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_message_num
{
public:
  explicit Init_LidarGlobalPose_message_num(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_pos_type message_num(::self_state::msg::LidarGlobalPose::_message_num_type arg)
  {
    msg_.message_num = std::move(arg);
    return Init_LidarGlobalPose_pos_type(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_utc_time
{
public:
  explicit Init_LidarGlobalPose_utc_time(::self_state::msg::LidarGlobalPose & msg)
  : msg_(msg)
  {}
  Init_LidarGlobalPose_message_num utc_time(::self_state::msg::LidarGlobalPose::_utc_time_type arg)
  {
    msg_.utc_time = std::move(arg);
    return Init_LidarGlobalPose_message_num(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

class Init_LidarGlobalPose_local_time
{
public:
  Init_LidarGlobalPose_local_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LidarGlobalPose_utc_time local_time(::self_state::msg::LidarGlobalPose::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_LidarGlobalPose_utc_time(msg_);
  }

private:
  ::self_state::msg::LidarGlobalPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::self_state::msg::LidarGlobalPose>()
{
  return self_state::msg::builder::Init_LidarGlobalPose_local_time();
}

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__LIDAR_GLOBAL_POSE__BUILDER_HPP_
