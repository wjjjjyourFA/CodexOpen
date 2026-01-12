// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sensor:msg/BestGnss.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__BEST_GNSS__BUILDER_HPP_
#define SENSOR__MSG__DETAIL__BEST_GNSS__BUILDER_HPP_

#include "sensor/msg/detail/best_gnss__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace sensor
{

namespace msg
{

namespace builder
{

class Init_BestGnss_height_gnss
{
public:
  explicit Init_BestGnss_height_gnss(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  ::sensor::msg::BestGnss height_gnss(::sensor::msg::BestGnss::_height_gnss_type arg)
  {
    msg_.height_gnss = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_longitude_gnss
{
public:
  explicit Init_BestGnss_longitude_gnss(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  Init_BestGnss_height_gnss longitude_gnss(::sensor::msg::BestGnss::_longitude_gnss_type arg)
  {
    msg_.longitude_gnss = std::move(arg);
    return Init_BestGnss_height_gnss(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_latitude_gnss
{
public:
  explicit Init_BestGnss_latitude_gnss(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  Init_BestGnss_longitude_gnss latitude_gnss(::sensor::msg::BestGnss::_latitude_gnss_type arg)
  {
    msg_.latitude_gnss = std::move(arg);
    return Init_BestGnss_longitude_gnss(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_num_satellite_used
{
public:
  explicit Init_BestGnss_num_satellite_used(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  Init_BestGnss_latitude_gnss num_satellite_used(::sensor::msg::BestGnss::_num_satellite_used_type arg)
  {
    msg_.num_satellite_used = std::move(arg);
    return Init_BestGnss_latitude_gnss(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_num_satellite_tracked
{
public:
  explicit Init_BestGnss_num_satellite_tracked(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  Init_BestGnss_num_satellite_used num_satellite_tracked(::sensor::msg::BestGnss::_num_satellite_tracked_type arg)
  {
    msg_.num_satellite_tracked = std::move(arg);
    return Init_BestGnss_num_satellite_used(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_pos_type
{
public:
  explicit Init_BestGnss_pos_type(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  Init_BestGnss_num_satellite_tracked pos_type(::sensor::msg::BestGnss::_pos_type_type arg)
  {
    msg_.pos_type = std::move(arg);
    return Init_BestGnss_num_satellite_tracked(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_sol_status
{
public:
  explicit Init_BestGnss_sol_status(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  Init_BestGnss_pos_type sol_status(::sensor::msg::BestGnss::_sol_status_type arg)
  {
    msg_.sol_status = std::move(arg);
    return Init_BestGnss_pos_type(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_message_num
{
public:
  explicit Init_BestGnss_message_num(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  Init_BestGnss_sol_status message_num(::sensor::msg::BestGnss::_message_num_type arg)
  {
    msg_.message_num = std::move(arg);
    return Init_BestGnss_sol_status(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_u_t_c_time
{
public:
  explicit Init_BestGnss_u_t_c_time(::sensor::msg::BestGnss & msg)
  : msg_(msg)
  {}
  Init_BestGnss_message_num u_t_c_time(::sensor::msg::BestGnss::_u_t_c_time_type arg)
  {
    msg_.u_t_c_time = std::move(arg);
    return Init_BestGnss_message_num(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

class Init_BestGnss_local_time
{
public:
  Init_BestGnss_local_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BestGnss_u_t_c_time local_time(::sensor::msg::BestGnss::_local_time_type arg)
  {
    msg_.local_time = std::move(arg);
    return Init_BestGnss_u_t_c_time(msg_);
  }

private:
  ::sensor::msg::BestGnss msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sensor::msg::BestGnss>()
{
  return sensor::msg::builder::Init_BestGnss_local_time();
}

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__BEST_GNSS__BUILDER_HPP_
