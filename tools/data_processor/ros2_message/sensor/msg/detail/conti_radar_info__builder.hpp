// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sensor:msg/ContiRadarInfo.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__CONTI_RADAR_INFO__BUILDER_HPP_
#define SENSOR__MSG__DETAIL__CONTI_RADAR_INFO__BUILDER_HPP_

#include "sensor/msg/detail/conti_radar_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace sensor
{

namespace msg
{

namespace builder
{

class Init_ContiRadarInfo_object_data
{
public:
  explicit Init_ContiRadarInfo_object_data(::sensor::msg::ContiRadarInfo & msg)
  : msg_(msg)
  {}
  ::sensor::msg::ContiRadarInfo object_data(::sensor::msg::ContiRadarInfo::_object_data_type arg)
  {
    msg_.object_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sensor::msg::ContiRadarInfo msg_;
};

class Init_ContiRadarInfo_object_num
{
public:
  explicit Init_ContiRadarInfo_object_num(::sensor::msg::ContiRadarInfo & msg)
  : msg_(msg)
  {}
  Init_ContiRadarInfo_object_data object_num(::sensor::msg::ContiRadarInfo::_object_num_type arg)
  {
    msg_.object_num = std::move(arg);
    return Init_ContiRadarInfo_object_data(msg_);
  }

private:
  ::sensor::msg::ContiRadarInfo msg_;
};

class Init_ContiRadarInfo_radar_i_d
{
public:
  explicit Init_ContiRadarInfo_radar_i_d(::sensor::msg::ContiRadarInfo & msg)
  : msg_(msg)
  {}
  Init_ContiRadarInfo_object_num radar_i_d(::sensor::msg::ContiRadarInfo::_radar_i_d_type arg)
  {
    msg_.radar_i_d = std::move(arg);
    return Init_ContiRadarInfo_object_num(msg_);
  }

private:
  ::sensor::msg::ContiRadarInfo msg_;
};

class Init_ContiRadarInfo_localpose_stamped
{
public:
  explicit Init_ContiRadarInfo_localpose_stamped(::sensor::msg::ContiRadarInfo & msg)
  : msg_(msg)
  {}
  Init_ContiRadarInfo_radar_i_d localpose_stamped(::sensor::msg::ContiRadarInfo::_localpose_stamped_type arg)
  {
    msg_.localpose_stamped = std::move(arg);
    return Init_ContiRadarInfo_radar_i_d(msg_);
  }

private:
  ::sensor::msg::ContiRadarInfo msg_;
};

class Init_ContiRadarInfo_gps_time
{
public:
  explicit Init_ContiRadarInfo_gps_time(::sensor::msg::ContiRadarInfo & msg)
  : msg_(msg)
  {}
  Init_ContiRadarInfo_localpose_stamped gps_time(::sensor::msg::ContiRadarInfo::_gps_time_type arg)
  {
    msg_.gps_time = std::move(arg);
    return Init_ContiRadarInfo_localpose_stamped(msg_);
  }

private:
  ::sensor::msg::ContiRadarInfo msg_;
};

class Init_ContiRadarInfo_local_time
{
public:
  explicit Init_ContiRadarInfo_local_time(::sensor::msg::ContiRadarInfo & msg)
  : msg_(msg)
  {}
  Init_ContiRadarInfo_gps_time local_time(::sensor::msg::ContiRadarInfo::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_ContiRadarInfo_gps_time(msg_);
  }

private:
  ::sensor::msg::ContiRadarInfo msg_;
};

class Init_ContiRadarInfo_header
{
public:
  Init_ContiRadarInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContiRadarInfo_local_time header(::sensor::msg::ContiRadarInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ContiRadarInfo_local_time(msg_);
  }

private:
  ::sensor::msg::ContiRadarInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sensor::msg::ContiRadarInfo>()
{
  return sensor::msg::builder::Init_ContiRadarInfo_header();
}

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__CONTI_RADAR_INFO__BUILDER_HPP_
