// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from self_state:msg/GlobalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__GLOBAL_POSE__BUILDER_HPP_
#define SELF_STATE__MSG__DETAIL__GLOBAL_POSE__BUILDER_HPP_

#include "self_state/msg/detail/global_pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace self_state
{

namespace msg
{

namespace builder
{

class Init_GlobalPose_reserved
{
public:
  explicit Init_GlobalPose_reserved(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  ::self_state::msg::GlobalPose reserved(::self_state::msg::GlobalPose::_reserved_type arg)
  {
    msg_.reserved = std::move(arg);
    return std::move(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_longitude
{
public:
  explicit Init_GlobalPose_longitude(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_reserved longitude(::self_state::msg::GlobalPose::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GlobalPose_reserved(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_latitude
{
public:
  explicit Init_GlobalPose_latitude(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_longitude latitude(::self_state::msg::GlobalPose::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GlobalPose_longitude(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_azimuth
{
public:
  explicit Init_GlobalPose_dev_azimuth(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_latitude dev_azimuth(::self_state::msg::GlobalPose::_dev_azimuth_type arg)
  {
    msg_.dev_azimuth = std::move(arg);
    return Init_GlobalPose_latitude(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_pitch
{
public:
  explicit Init_GlobalPose_dev_pitch(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_azimuth dev_pitch(::self_state::msg::GlobalPose::_dev_pitch_type arg)
  {
    msg_.dev_pitch = std::move(arg);
    return Init_GlobalPose_dev_azimuth(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_roll
{
public:
  explicit Init_GlobalPose_dev_roll(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_pitch dev_roll(::self_state::msg::GlobalPose::_dev_roll_type arg)
  {
    msg_.dev_roll = std::move(arg);
    return Init_GlobalPose_dev_pitch(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_v_up
{
public:
  explicit Init_GlobalPose_dev_v_up(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_roll dev_v_up(::self_state::msg::GlobalPose::_dev_v_up_type arg)
  {
    msg_.dev_v_up = std::move(arg);
    return Init_GlobalPose_dev_roll(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_v_east
{
public:
  explicit Init_GlobalPose_dev_v_east(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_v_up dev_v_east(::self_state::msg::GlobalPose::_dev_v_east_type arg)
  {
    msg_.dev_v_east = std::move(arg);
    return Init_GlobalPose_dev_v_up(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_v_north
{
public:
  explicit Init_GlobalPose_dev_v_north(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_v_east dev_v_north(::self_state::msg::GlobalPose::_dev_v_north_type arg)
  {
    msg_.dev_v_north = std::move(arg);
    return Init_GlobalPose_dev_v_east(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_height
{
public:
  explicit Init_GlobalPose_dev_height(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_v_north dev_height(::self_state::msg::GlobalPose::_dev_height_type arg)
  {
    msg_.dev_height = std::move(arg);
    return Init_GlobalPose_dev_v_north(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_gauss_y
{
public:
  explicit Init_GlobalPose_dev_gauss_y(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_height dev_gauss_y(::self_state::msg::GlobalPose::_dev_gauss_y_type arg)
  {
    msg_.dev_gauss_y = std::move(arg);
    return Init_GlobalPose_dev_height(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_dev_gauss_x
{
public:
  explicit Init_GlobalPose_dev_gauss_x(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_gauss_y dev_gauss_x(::self_state::msg::GlobalPose::_dev_gauss_x_type arg)
  {
    msg_.dev_gauss_x = std::move(arg);
    return Init_GlobalPose_dev_gauss_y(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_azimuth
{
public:
  explicit Init_GlobalPose_azimuth(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_dev_gauss_x azimuth(::self_state::msg::GlobalPose::_azimuth_type arg)
  {
    msg_.azimuth = std::move(arg);
    return Init_GlobalPose_dev_gauss_x(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_pitch
{
public:
  explicit Init_GlobalPose_pitch(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_azimuth pitch(::self_state::msg::GlobalPose::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_GlobalPose_azimuth(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_roll
{
public:
  explicit Init_GlobalPose_roll(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_pitch roll(::self_state::msg::GlobalPose::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_GlobalPose_pitch(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_v_up
{
public:
  explicit Init_GlobalPose_v_up(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_roll v_up(::self_state::msg::GlobalPose::_v_up_type arg)
  {
    msg_.v_up = std::move(arg);
    return Init_GlobalPose_roll(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_v_east
{
public:
  explicit Init_GlobalPose_v_east(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_v_up v_east(::self_state::msg::GlobalPose::_v_east_type arg)
  {
    msg_.v_east = std::move(arg);
    return Init_GlobalPose_v_up(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_v_north
{
public:
  explicit Init_GlobalPose_v_north(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_v_east v_north(::self_state::msg::GlobalPose::_v_north_type arg)
  {
    msg_.v_north = std::move(arg);
    return Init_GlobalPose_v_east(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_height
{
public:
  explicit Init_GlobalPose_height(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_v_north height(::self_state::msg::GlobalPose::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_GlobalPose_v_north(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_gauss_y
{
public:
  explicit Init_GlobalPose_gauss_y(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_height gauss_y(::self_state::msg::GlobalPose::_gauss_y_type arg)
  {
    msg_.gauss_y = std::move(arg);
    return Init_GlobalPose_height(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_gauss_x
{
public:
  explicit Init_GlobalPose_gauss_x(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_gauss_y gauss_x(::self_state::msg::GlobalPose::_gauss_x_type arg)
  {
    msg_.gauss_x = std::move(arg);
    return Init_GlobalPose_gauss_y(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_pos_type
{
public:
  explicit Init_GlobalPose_pos_type(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_gauss_x pos_type(::self_state::msg::GlobalPose::_pos_type_type arg)
  {
    msg_.pos_type = std::move(arg);
    return Init_GlobalPose_gauss_x(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_ins_status
{
public:
  explicit Init_GlobalPose_ins_status(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_pos_type ins_status(::self_state::msg::GlobalPose::_ins_status_type arg)
  {
    msg_.ins_status = std::move(arg);
    return Init_GlobalPose_pos_type(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_message_num
{
public:
  explicit Init_GlobalPose_message_num(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_ins_status message_num(::self_state::msg::GlobalPose::_message_num_type arg)
  {
    msg_.message_num = std::move(arg);
    return Init_GlobalPose_ins_status(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_utc_time
{
public:
  explicit Init_GlobalPose_utc_time(::self_state::msg::GlobalPose & msg)
  : msg_(msg)
  {}
  Init_GlobalPose_message_num utc_time(::self_state::msg::GlobalPose::_utc_time_type arg)
  {
    msg_.utc_time = std::move(arg);
    return Init_GlobalPose_message_num(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

class Init_GlobalPose_local_time
{
public:
  Init_GlobalPose_local_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GlobalPose_utc_time local_time(::self_state::msg::GlobalPose::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_GlobalPose_utc_time(msg_);
  }

private:
  ::self_state::msg::GlobalPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::self_state::msg::GlobalPose>()
{
  return self_state::msg::builder::Init_GlobalPose_local_time();
}

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__GLOBAL_POSE__BUILDER_HPP_
