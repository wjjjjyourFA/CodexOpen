// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sensor:msg/GnssSolutionStatus.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__BUILDER_HPP_
#define SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__BUILDER_HPP_

#include "sensor/msg/detail/gnss_solution_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace sensor
{

namespace msg
{

namespace builder
{

class Init_GnssSolutionStatus_sol_status
{
public:
  Init_GnssSolutionStatus_sol_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::sensor::msg::GnssSolutionStatus sol_status(::sensor::msg::GnssSolutionStatus::_sol_status_type arg)
  {
    msg_.sol_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sensor::msg::GnssSolutionStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sensor::msg::GnssSolutionStatus>()
{
  return sensor::msg::builder::Init_GnssSolutionStatus_sol_status();
}

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__BUILDER_HPP_
