// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sensor:msg/EsrRadarInfo.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__ESR_RADAR_INFO__BUILDER_HPP_
#define SENSOR__MSG__DETAIL__ESR_RADAR_INFO__BUILDER_HPP_

#include "sensor/msg/detail/esr_radar_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace sensor
{

namespace msg
{

namespace builder
{

class Init_EsrRadarInfo_object_data
{
public:
  explicit Init_EsrRadarInfo_object_data(::sensor::msg::EsrRadarInfo & msg)
  : msg_(msg)
  {}
  ::sensor::msg::EsrRadarInfo object_data(::sensor::msg::EsrRadarInfo::_object_data_type arg)
  {
    msg_.object_data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sensor::msg::EsrRadarInfo msg_;
};

class Init_EsrRadarInfo_object_num
{
public:
  explicit Init_EsrRadarInfo_object_num(::sensor::msg::EsrRadarInfo & msg)
  : msg_(msg)
  {}
  Init_EsrRadarInfo_object_data object_num(::sensor::msg::EsrRadarInfo::_object_num_type arg)
  {
    msg_.object_num = std::move(arg);
    return Init_EsrRadarInfo_object_data(msg_);
  }

private:
  ::sensor::msg::EsrRadarInfo msg_;
};

class Init_EsrRadarInfo_radar_id
{
public:
  explicit Init_EsrRadarInfo_radar_id(::sensor::msg::EsrRadarInfo & msg)
  : msg_(msg)
  {}
  Init_EsrRadarInfo_object_num radar_id(::sensor::msg::EsrRadarInfo::_radar_id_type arg)
  {
    msg_.radar_id = std::move(arg);
    return Init_EsrRadarInfo_object_num(msg_);
  }

private:
  ::sensor::msg::EsrRadarInfo msg_;
};

class Init_EsrRadarInfo_localpose_stamped
{
public:
  explicit Init_EsrRadarInfo_localpose_stamped(::sensor::msg::EsrRadarInfo & msg)
  : msg_(msg)
  {}
  Init_EsrRadarInfo_radar_id localpose_stamped(::sensor::msg::EsrRadarInfo::_localpose_stamped_type arg)
  {
    msg_.localpose_stamped = std::move(arg);
    return Init_EsrRadarInfo_radar_id(msg_);
  }

private:
  ::sensor::msg::EsrRadarInfo msg_;
};

class Init_EsrRadarInfo_gps_time
{
public:
  explicit Init_EsrRadarInfo_gps_time(::sensor::msg::EsrRadarInfo & msg)
  : msg_(msg)
  {}
  Init_EsrRadarInfo_localpose_stamped gps_time(::sensor::msg::EsrRadarInfo::_gps_time_type arg)
  {
    msg_.gps_time = std::move(arg);
    return Init_EsrRadarInfo_localpose_stamped(msg_);
  }

private:
  ::sensor::msg::EsrRadarInfo msg_;
};

class Init_EsrRadarInfo_local_time
{
public:
  explicit Init_EsrRadarInfo_local_time(::sensor::msg::EsrRadarInfo & msg)
  : msg_(msg)
  {}
  Init_EsrRadarInfo_gps_time local_time(::sensor::msg::EsrRadarInfo::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_EsrRadarInfo_gps_time(msg_);
  }

private:
  ::sensor::msg::EsrRadarInfo msg_;
};

class Init_EsrRadarInfo_header
{
public:
  Init_EsrRadarInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EsrRadarInfo_local_time header(::sensor::msg::EsrRadarInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EsrRadarInfo_local_time(msg_);
  }

private:
  ::sensor::msg::EsrRadarInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sensor::msg::EsrRadarInfo>()
{
  return sensor::msg::builder::Init_EsrRadarInfo_header();
}

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__ESR_RADAR_INFO__BUILDER_HPP_
