// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sensor:msg/EsrRadarObject.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__ESR_RADAR_OBJECT__BUILDER_HPP_
#define SENSOR__MSG__DETAIL__ESR_RADAR_OBJECT__BUILDER_HPP_

#include "sensor/msg/detail/esr_radar_object__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace sensor
{

namespace msg
{

namespace builder
{

class Init_EsrRadarObject_rcsvalue
{
public:
  explicit Init_EsrRadarObject_rcsvalue(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  ::sensor::msg::EsrRadarObject rcsvalue(::sensor::msg::EsrRadarObject::_rcsvalue_type arg)
  {
    msg_.rcsvalue = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_confidence
{
public:
  explicit Init_EsrRadarObject_confidence(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_rcsvalue confidence(::sensor::msg::EsrRadarObject::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_EsrRadarObject_rcsvalue(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_type
{
public:
  explicit Init_EsrRadarObject_type(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_confidence type(::sensor::msg::EsrRadarObject::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_EsrRadarObject_confidence(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_is_fcw_target
{
public:
  explicit Init_EsrRadarObject_is_fcw_target(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_type is_fcw_target(::sensor::msg::EsrRadarObject::_is_fcw_target_type arg)
  {
    msg_.is_fcw_target = std::move(arg);
    return Init_EsrRadarObject_type(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_is_cmbb_target
{
public:
  explicit Init_EsrRadarObject_is_cmbb_target(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_is_fcw_target is_cmbb_target(::sensor::msg::EsrRadarObject::_is_cmbb_target_type arg)
  {
    msg_.is_cmbb_target = std::move(arg);
    return Init_EsrRadarObject_is_fcw_target(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_is_acc_target
{
public:
  explicit Init_EsrRadarObject_is_acc_target(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_is_cmbb_target is_acc_target(::sensor::msg::EsrRadarObject::_is_acc_target_type arg)
  {
    msg_.is_acc_target = std::move(arg);
    return Init_EsrRadarObject_is_cmbb_target(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_track_status
{
public:
  explicit Init_EsrRadarObject_track_status(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_is_acc_target track_status(::sensor::msg::EsrRadarObject::_track_status_type arg)
  {
    msg_.track_status = std::move(arg);
    return Init_EsrRadarObject_is_acc_target(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_lat_rate
{
public:
  explicit Init_EsrRadarObject_lat_rate(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_track_status lat_rate(::sensor::msg::EsrRadarObject::_lat_rate_type arg)
  {
    msg_.lat_rate = std::move(arg);
    return Init_EsrRadarObject_track_status(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_range_rate
{
public:
  explicit Init_EsrRadarObject_range_rate(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_lat_rate range_rate(::sensor::msg::EsrRadarObject::_range_rate_type arg)
  {
    msg_.range_rate = std::move(arg);
    return Init_EsrRadarObject_lat_rate(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_width
{
public:
  explicit Init_EsrRadarObject_width(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_range_rate width(::sensor::msg::EsrRadarObject::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_EsrRadarObject_range_rate(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_height
{
public:
  explicit Init_EsrRadarObject_height(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_width height(::sensor::msg::EsrRadarObject::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_EsrRadarObject_width(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_speed
{
public:
  explicit Init_EsrRadarObject_speed(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_height speed(::sensor::msg::EsrRadarObject::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_EsrRadarObject_height(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_left_distance
{
public:
  explicit Init_EsrRadarObject_left_distance(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_speed left_distance(::sensor::msg::EsrRadarObject::_left_distance_type arg)
  {
    msg_.left_distance = std::move(arg);
    return Init_EsrRadarObject_speed(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_front_distance
{
public:
  explicit Init_EsrRadarObject_front_distance(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_left_distance front_distance(::sensor::msg::EsrRadarObject::_front_distance_type arg)
  {
    msg_.front_distance = std::move(arg);
    return Init_EsrRadarObject_left_distance(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_angle
{
public:
  explicit Init_EsrRadarObject_angle(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_front_distance angle(::sensor::msg::EsrRadarObject::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_EsrRadarObject_front_distance(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_range
{
public:
  explicit Init_EsrRadarObject_range(::sensor::msg::EsrRadarObject & msg)
  : msg_(msg)
  {}
  Init_EsrRadarObject_angle range(::sensor::msg::EsrRadarObject::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_EsrRadarObject_angle(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

class Init_EsrRadarObject_target_i_d
{
public:
  Init_EsrRadarObject_target_i_d()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EsrRadarObject_range target_i_d(::sensor::msg::EsrRadarObject::_target_i_d_type arg)
  {
    msg_.target_i_d = std::move(arg);
    return Init_EsrRadarObject_range(msg_);
  }

private:
  ::sensor::msg::EsrRadarObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sensor::msg::EsrRadarObject>()
{
  return sensor::msg::builder::Init_EsrRadarObject_target_i_d();
}

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__ESR_RADAR_OBJECT__BUILDER_HPP_
