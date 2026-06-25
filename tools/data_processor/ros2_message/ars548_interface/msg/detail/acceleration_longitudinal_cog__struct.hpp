// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/AccelerationLongitudinalCog.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LONGITUDINAL_COG__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LONGITUDINAL_COG__STRUCT_HPP_

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
# define DEPRECATED__ars548_interface__msg__AccelerationLongitudinalCog __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__AccelerationLongitudinalCog __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AccelerationLongitudinalCog_
{
  using Type = AccelerationLongitudinalCog_<ContainerAllocator>;

  explicit AccelerationLongitudinalCog_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->acceleration_longitudinal_err_amp = 0.0f;
      this->acceleration_longitudinal_err_amp_invalid_flag = 0;
      this->qualifier_acceleration_longitudinal = 0;
      this->acceleration_longitudinal = 0.0f;
      this->acceleration_longitudinal_invalid_flag = 0;
      this->acceleration_longitudinal_event_data_qualifier = 0;
    }
  }

  explicit AccelerationLongitudinalCog_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->acceleration_longitudinal_err_amp = 0.0f;
      this->acceleration_longitudinal_err_amp_invalid_flag = 0;
      this->qualifier_acceleration_longitudinal = 0;
      this->acceleration_longitudinal = 0.0f;
      this->acceleration_longitudinal_invalid_flag = 0;
      this->acceleration_longitudinal_event_data_qualifier = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _acceleration_longitudinal_err_amp_type =
    float;
  _acceleration_longitudinal_err_amp_type acceleration_longitudinal_err_amp;
  using _acceleration_longitudinal_err_amp_invalid_flag_type =
    uint8_t;
  _acceleration_longitudinal_err_amp_invalid_flag_type acceleration_longitudinal_err_amp_invalid_flag;
  using _qualifier_acceleration_longitudinal_type =
    uint8_t;
  _qualifier_acceleration_longitudinal_type qualifier_acceleration_longitudinal;
  using _acceleration_longitudinal_type =
    float;
  _acceleration_longitudinal_type acceleration_longitudinal;
  using _acceleration_longitudinal_invalid_flag_type =
    uint8_t;
  _acceleration_longitudinal_invalid_flag_type acceleration_longitudinal_invalid_flag;
  using _acceleration_longitudinal_event_data_qualifier_type =
    uint8_t;
  _acceleration_longitudinal_event_data_qualifier_type acceleration_longitudinal_event_data_qualifier;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__acceleration_longitudinal_err_amp(
    const float & _arg)
  {
    this->acceleration_longitudinal_err_amp = _arg;
    return *this;
  }
  Type & set__acceleration_longitudinal_err_amp_invalid_flag(
    const uint8_t & _arg)
  {
    this->acceleration_longitudinal_err_amp_invalid_flag = _arg;
    return *this;
  }
  Type & set__qualifier_acceleration_longitudinal(
    const uint8_t & _arg)
  {
    this->qualifier_acceleration_longitudinal = _arg;
    return *this;
  }
  Type & set__acceleration_longitudinal(
    const float & _arg)
  {
    this->acceleration_longitudinal = _arg;
    return *this;
  }
  Type & set__acceleration_longitudinal_invalid_flag(
    const uint8_t & _arg)
  {
    this->acceleration_longitudinal_invalid_flag = _arg;
    return *this;
  }
  Type & set__acceleration_longitudinal_event_data_qualifier(
    const uint8_t & _arg)
  {
    this->acceleration_longitudinal_event_data_qualifier = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__AccelerationLongitudinalCog
    std::shared_ptr<ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__AccelerationLongitudinalCog
    std::shared_ptr<ars548_interface::msg::AccelerationLongitudinalCog_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AccelerationLongitudinalCog_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->acceleration_longitudinal_err_amp != other.acceleration_longitudinal_err_amp) {
      return false;
    }
    if (this->acceleration_longitudinal_err_amp_invalid_flag != other.acceleration_longitudinal_err_amp_invalid_flag) {
      return false;
    }
    if (this->qualifier_acceleration_longitudinal != other.qualifier_acceleration_longitudinal) {
      return false;
    }
    if (this->acceleration_longitudinal != other.acceleration_longitudinal) {
      return false;
    }
    if (this->acceleration_longitudinal_invalid_flag != other.acceleration_longitudinal_invalid_flag) {
      return false;
    }
    if (this->acceleration_longitudinal_event_data_qualifier != other.acceleration_longitudinal_event_data_qualifier) {
      return false;
    }
    return true;
  }
  bool operator!=(const AccelerationLongitudinalCog_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AccelerationLongitudinalCog_

// alias to use template instance with default allocator
using AccelerationLongitudinalCog =
  ars548_interface::msg::AccelerationLongitudinalCog_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__ACCELERATION_LONGITUDINAL_COG__STRUCT_HPP_
