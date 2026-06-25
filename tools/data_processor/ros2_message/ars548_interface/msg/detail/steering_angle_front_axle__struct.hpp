// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/SteeringAngleFrontAxle.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__STRUCT_HPP_

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
# define DEPRECATED__ars548_interface__msg__SteeringAngleFrontAxle __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__SteeringAngleFrontAxle __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SteeringAngleFrontAxle_
{
  using Type = SteeringAngleFrontAxle_<ContainerAllocator>;

  explicit SteeringAngleFrontAxle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->qualifier_steering_angle_front_axle = 0;
      this->steering_angle_front_axle_err_amp = 0.0f;
      this->steering_angle_front_axle_err_amp_invalid_flag = 0;
      this->steering_angle_front_axle = 0.0f;
      this->steering_angle_front_axle_invalid_flag = 0;
      this->steering_angle_front_axle_event_data_qualifier = 0;
    }
  }

  explicit SteeringAngleFrontAxle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->qualifier_steering_angle_front_axle = 0;
      this->steering_angle_front_axle_err_amp = 0.0f;
      this->steering_angle_front_axle_err_amp_invalid_flag = 0;
      this->steering_angle_front_axle = 0.0f;
      this->steering_angle_front_axle_invalid_flag = 0;
      this->steering_angle_front_axle_event_data_qualifier = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _qualifier_steering_angle_front_axle_type =
    uint8_t;
  _qualifier_steering_angle_front_axle_type qualifier_steering_angle_front_axle;
  using _steering_angle_front_axle_err_amp_type =
    float;
  _steering_angle_front_axle_err_amp_type steering_angle_front_axle_err_amp;
  using _steering_angle_front_axle_err_amp_invalid_flag_type =
    uint8_t;
  _steering_angle_front_axle_err_amp_invalid_flag_type steering_angle_front_axle_err_amp_invalid_flag;
  using _steering_angle_front_axle_type =
    float;
  _steering_angle_front_axle_type steering_angle_front_axle;
  using _steering_angle_front_axle_invalid_flag_type =
    uint8_t;
  _steering_angle_front_axle_invalid_flag_type steering_angle_front_axle_invalid_flag;
  using _steering_angle_front_axle_event_data_qualifier_type =
    uint8_t;
  _steering_angle_front_axle_event_data_qualifier_type steering_angle_front_axle_event_data_qualifier;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__qualifier_steering_angle_front_axle(
    const uint8_t & _arg)
  {
    this->qualifier_steering_angle_front_axle = _arg;
    return *this;
  }
  Type & set__steering_angle_front_axle_err_amp(
    const float & _arg)
  {
    this->steering_angle_front_axle_err_amp = _arg;
    return *this;
  }
  Type & set__steering_angle_front_axle_err_amp_invalid_flag(
    const uint8_t & _arg)
  {
    this->steering_angle_front_axle_err_amp_invalid_flag = _arg;
    return *this;
  }
  Type & set__steering_angle_front_axle(
    const float & _arg)
  {
    this->steering_angle_front_axle = _arg;
    return *this;
  }
  Type & set__steering_angle_front_axle_invalid_flag(
    const uint8_t & _arg)
  {
    this->steering_angle_front_axle_invalid_flag = _arg;
    return *this;
  }
  Type & set__steering_angle_front_axle_event_data_qualifier(
    const uint8_t & _arg)
  {
    this->steering_angle_front_axle_event_data_qualifier = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__SteeringAngleFrontAxle
    std::shared_ptr<ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__SteeringAngleFrontAxle
    std::shared_ptr<ars548_interface::msg::SteeringAngleFrontAxle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SteeringAngleFrontAxle_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->qualifier_steering_angle_front_axle != other.qualifier_steering_angle_front_axle) {
      return false;
    }
    if (this->steering_angle_front_axle_err_amp != other.steering_angle_front_axle_err_amp) {
      return false;
    }
    if (this->steering_angle_front_axle_err_amp_invalid_flag != other.steering_angle_front_axle_err_amp_invalid_flag) {
      return false;
    }
    if (this->steering_angle_front_axle != other.steering_angle_front_axle) {
      return false;
    }
    if (this->steering_angle_front_axle_invalid_flag != other.steering_angle_front_axle_invalid_flag) {
      return false;
    }
    if (this->steering_angle_front_axle_event_data_qualifier != other.steering_angle_front_axle_event_data_qualifier) {
      return false;
    }
    return true;
  }
  bool operator!=(const SteeringAngleFrontAxle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SteeringAngleFrontAxle_

// alias to use template instance with default allocator
using SteeringAngleFrontAxle =
  ars548_interface::msg::SteeringAngleFrontAxle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__STEERING_ANGLE_FRONT_AXLE__STRUCT_HPP_
