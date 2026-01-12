// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/DrivingDirection.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__STRUCT_HPP_

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
# define DEPRECATED__ars548_interface__msg__DrivingDirection __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__DrivingDirection __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DrivingDirection_
{
  using Type = DrivingDirection_<ContainerAllocator>;

  explicit DrivingDirection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->driving_direction_unconfirmed = 0;
      this->driving_direction_confirmed = 0;
    }
  }

  explicit DrivingDirection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->driving_direction_unconfirmed = 0;
      this->driving_direction_confirmed = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _driving_direction_unconfirmed_type =
    uint8_t;
  _driving_direction_unconfirmed_type driving_direction_unconfirmed;
  using _driving_direction_confirmed_type =
    uint8_t;
  _driving_direction_confirmed_type driving_direction_confirmed;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__driving_direction_unconfirmed(
    const uint8_t & _arg)
  {
    this->driving_direction_unconfirmed = _arg;
    return *this;
  }
  Type & set__driving_direction_confirmed(
    const uint8_t & _arg)
  {
    this->driving_direction_confirmed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::DrivingDirection_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::DrivingDirection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::DrivingDirection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::DrivingDirection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::DrivingDirection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::DrivingDirection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::DrivingDirection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::DrivingDirection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::DrivingDirection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::DrivingDirection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__DrivingDirection
    std::shared_ptr<ars548_interface::msg::DrivingDirection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__DrivingDirection
    std::shared_ptr<ars548_interface::msg::DrivingDirection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DrivingDirection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->driving_direction_unconfirmed != other.driving_direction_unconfirmed) {
      return false;
    }
    if (this->driving_direction_confirmed != other.driving_direction_confirmed) {
      return false;
    }
    return true;
  }
  bool operator!=(const DrivingDirection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DrivingDirection_

// alias to use template instance with default allocator
using DrivingDirection =
  ars548_interface::msg::DrivingDirection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__DRIVING_DIRECTION__STRUCT_HPP_
