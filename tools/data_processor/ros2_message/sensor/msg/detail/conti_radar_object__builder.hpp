// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sensor:msg/ContiRadarObject.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__CONTI_RADAR_OBJECT__BUILDER_HPP_
#define SENSOR__MSG__DETAIL__CONTI_RADAR_OBJECT__BUILDER_HPP_

#include "sensor/msg/detail/conti_radar_object__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace sensor
{

namespace msg
{

namespace builder
{

class Init_ContiRadarObject_rcsvalue
{
public:
  explicit Init_ContiRadarObject_rcsvalue(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  ::sensor::msg::ContiRadarObject rcsvalue(::sensor::msg::ContiRadarObject::_rcsvalue_type arg)
  {
    msg_.rcsvalue = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_confidence
{
public:
  explicit Init_ContiRadarObject_confidence(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_rcsvalue confidence(::sensor::msg::ContiRadarObject::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_ContiRadarObject_rcsvalue(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_type
{
public:
  explicit Init_ContiRadarObject_type(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_confidence type(::sensor::msg::ContiRadarObject::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_ContiRadarObject_confidence(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_is_fcw_target
{
public:
  explicit Init_ContiRadarObject_is_fcw_target(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_type is_fcw_target(::sensor::msg::ContiRadarObject::_is_fcw_target_type arg)
  {
    msg_.is_fcw_target = std::move(arg);
    return Init_ContiRadarObject_type(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_is_cmbb_target
{
public:
  explicit Init_ContiRadarObject_is_cmbb_target(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_is_fcw_target is_cmbb_target(::sensor::msg::ContiRadarObject::_is_cmbb_target_type arg)
  {
    msg_.is_cmbb_target = std::move(arg);
    return Init_ContiRadarObject_is_fcw_target(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_is_acc_target
{
public:
  explicit Init_ContiRadarObject_is_acc_target(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_is_cmbb_target is_acc_target(::sensor::msg::ContiRadarObject::_is_acc_target_type arg)
  {
    msg_.is_acc_target = std::move(arg);
    return Init_ContiRadarObject_is_cmbb_target(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_track_status
{
public:
  explicit Init_ContiRadarObject_track_status(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_is_acc_target track_status(::sensor::msg::ContiRadarObject::_track_status_type arg)
  {
    msg_.track_status = std::move(arg);
    return Init_ContiRadarObject_is_acc_target(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_lat_rate
{
public:
  explicit Init_ContiRadarObject_lat_rate(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_track_status lat_rate(::sensor::msg::ContiRadarObject::_lat_rate_type arg)
  {
    msg_.lat_rate = std::move(arg);
    return Init_ContiRadarObject_track_status(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_range_rate
{
public:
  explicit Init_ContiRadarObject_range_rate(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_lat_rate range_rate(::sensor::msg::ContiRadarObject::_range_rate_type arg)
  {
    msg_.range_rate = std::move(arg);
    return Init_ContiRadarObject_lat_rate(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_width
{
public:
  explicit Init_ContiRadarObject_width(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_range_rate width(::sensor::msg::ContiRadarObject::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_ContiRadarObject_range_rate(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_height
{
public:
  explicit Init_ContiRadarObject_height(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_width height(::sensor::msg::ContiRadarObject::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_ContiRadarObject_width(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_speed_y
{
public:
  explicit Init_ContiRadarObject_speed_y(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_height speed_y(::sensor::msg::ContiRadarObject::_speed_y_type arg)
  {
    msg_.speed_y = std::move(arg);
    return Init_ContiRadarObject_height(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_speed_x
{
public:
  explicit Init_ContiRadarObject_speed_x(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_speed_y speed_x(::sensor::msg::ContiRadarObject::_speed_x_type arg)
  {
    msg_.speed_x = std::move(arg);
    return Init_ContiRadarObject_speed_y(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_speed
{
public:
  explicit Init_ContiRadarObject_speed(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_speed_x speed(::sensor::msg::ContiRadarObject::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_ContiRadarObject_speed_x(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_z
{
public:
  explicit Init_ContiRadarObject_z(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_speed z(::sensor::msg::ContiRadarObject::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ContiRadarObject_speed(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_y
{
public:
  explicit Init_ContiRadarObject_y(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_z y(::sensor::msg::ContiRadarObject::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ContiRadarObject_z(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_x
{
public:
  explicit Init_ContiRadarObject_x(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_y x(::sensor::msg::ContiRadarObject::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ContiRadarObject_y(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_angle
{
public:
  explicit Init_ContiRadarObject_angle(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_x angle(::sensor::msg::ContiRadarObject::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_ContiRadarObject_x(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_range
{
public:
  explicit Init_ContiRadarObject_range(::sensor::msg::ContiRadarObject & msg)
  : msg_(msg)
  {}
  Init_ContiRadarObject_angle range(::sensor::msg::ContiRadarObject::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_ContiRadarObject_angle(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

class Init_ContiRadarObject_target_id
{
public:
  Init_ContiRadarObject_target_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContiRadarObject_range target_id(::sensor::msg::ContiRadarObject::_target_id_type arg)
  {
    msg_.target_id = std::move(arg);
    return Init_ContiRadarObject_range(msg_);
  }

private:
  ::sensor::msg::ContiRadarObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sensor::msg::ContiRadarObject>()
{
  return sensor::msg::builder::Init_ContiRadarObject_target_id();
}

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__CONTI_RADAR_OBJECT__BUILDER_HPP_
