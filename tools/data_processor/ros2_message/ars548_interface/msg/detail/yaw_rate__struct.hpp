// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/YawRate.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__YAW_RATE__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__YAW_RATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ars548_interface__msg__YawRate __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__YawRate __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YawRate_
{
  using Type = YawRate_<ContainerAllocator>;

  explicit YawRate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw_rate_err_amp = 0.0f;
      this->yaw_rate_err_amp_invalid_flag = 0;
      this->qualifier_yaw_rate = 0;
      this->yaw_rate = 0.0f;
      this->yaw_rate_invalid_flag = 0;
      this->yaw_rate_event_data_qualifier = 0;
    }
  }

  explicit YawRate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw_rate_err_amp = 0.0f;
      this->yaw_rate_err_amp_invalid_flag = 0;
      this->qualifier_yaw_rate = 0;
      this->yaw_rate = 0.0f;
      this->yaw_rate_invalid_flag = 0;
      this->yaw_rate_event_data_qualifier = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _yaw_rate_err_amp_type =
    float;
  _yaw_rate_err_amp_type yaw_rate_err_amp;
  using _yaw_rate_err_amp_invalid_flag_type =
    uint8_t;
  _yaw_rate_err_amp_invalid_flag_type yaw_rate_err_amp_invalid_flag;
  using _qualifier_yaw_rate_type =
    uint8_t;
  _qualifier_yaw_rate_type qualifier_yaw_rate;
  using _yaw_rate_type =
    float;
  _yaw_rate_type yaw_rate;
  using _yaw_rate_invalid_flag_type =
    uint8_t;
  _yaw_rate_invalid_flag_type yaw_rate_invalid_flag;
  using _yaw_rate_event_data_qualifier_type =
    uint8_t;
  _yaw_rate_event_data_qualifier_type yaw_rate_event_data_qualifier;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__yaw_rate_err_amp(
    const float & _arg)
  {
    this->yaw_rate_err_amp = _arg;
    return *this;
  }
  Type & set__yaw_rate_err_amp_invalid_flag(
    const uint8_t & _arg)
  {
    this->yaw_rate_err_amp_invalid_flag = _arg;
    return *this;
  }
  Type & set__qualifier_yaw_rate(
    const uint8_t & _arg)
  {
    this->qualifier_yaw_rate = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const float & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }
  Type & set__yaw_rate_invalid_flag(
    const uint8_t & _arg)
  {
    this->yaw_rate_invalid_flag = _arg;
    return *this;
  }
  Type & set__yaw_rate_event_data_qualifier(
    const uint8_t & _arg)
  {
    this->yaw_rate_event_data_qualifier = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::YawRate_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::YawRate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::YawRate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::YawRate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::YawRate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::YawRate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::YawRate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::YawRate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::YawRate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::YawRate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__YawRate
    std::shared_ptr<ars548_interface::msg::YawRate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__YawRate
    std::shared_ptr<ars548_interface::msg::YawRate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YawRate_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->yaw_rate_err_amp != other.yaw_rate_err_amp) {
      return false;
    }
    if (this->yaw_rate_err_amp_invalid_flag != other.yaw_rate_err_amp_invalid_flag) {
      return false;
    }
    if (this->qualifier_yaw_rate != other.qualifier_yaw_rate) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    if (this->yaw_rate_invalid_flag != other.yaw_rate_invalid_flag) {
      return false;
    }
    if (this->yaw_rate_event_data_qualifier != other.yaw_rate_event_data_qualifier) {
      return false;
    }
    return true;
  }
  bool operator!=(const YawRate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YawRate_

// alias to use template instance with default allocator
using YawRate =
  ars548_interface::msg::YawRate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__YAW_RATE__STRUCT_HPP_
