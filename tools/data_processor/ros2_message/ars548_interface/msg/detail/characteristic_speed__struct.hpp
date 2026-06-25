// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/CharacteristicSpeed.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__STRUCT_HPP_

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
# define DEPRECATED__ars548_interface__msg__CharacteristicSpeed __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__CharacteristicSpeed __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CharacteristicSpeed_
{
  using Type = CharacteristicSpeed_<ContainerAllocator>;

  explicit CharacteristicSpeed_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->characteristic_speed_err_amp = 0;
      this->qualifier_characteristic_speed = 0;
      this->characteristic_speed = 0;
    }
  }

  explicit CharacteristicSpeed_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->characteristic_speed_err_amp = 0;
      this->qualifier_characteristic_speed = 0;
      this->characteristic_speed = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _characteristic_speed_err_amp_type =
    uint8_t;
  _characteristic_speed_err_amp_type characteristic_speed_err_amp;
  using _qualifier_characteristic_speed_type =
    uint8_t;
  _qualifier_characteristic_speed_type qualifier_characteristic_speed;
  using _characteristic_speed_type =
    uint8_t;
  _characteristic_speed_type characteristic_speed;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__characteristic_speed_err_amp(
    const uint8_t & _arg)
  {
    this->characteristic_speed_err_amp = _arg;
    return *this;
  }
  Type & set__qualifier_characteristic_speed(
    const uint8_t & _arg)
  {
    this->qualifier_characteristic_speed = _arg;
    return *this;
  }
  Type & set__characteristic_speed(
    const uint8_t & _arg)
  {
    this->characteristic_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__CharacteristicSpeed
    std::shared_ptr<ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__CharacteristicSpeed
    std::shared_ptr<ars548_interface::msg::CharacteristicSpeed_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CharacteristicSpeed_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->characteristic_speed_err_amp != other.characteristic_speed_err_amp) {
      return false;
    }
    if (this->qualifier_characteristic_speed != other.qualifier_characteristic_speed) {
      return false;
    }
    if (this->characteristic_speed != other.characteristic_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const CharacteristicSpeed_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CharacteristicSpeed_

// alias to use template instance with default allocator
using CharacteristicSpeed =
  ars548_interface::msg::CharacteristicSpeed_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__STRUCT_HPP_
