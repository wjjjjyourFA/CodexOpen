// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/VelocityVehicle.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__STRUCT_HPP_

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
# define DEPRECATED__ars548_interface__msg__VelocityVehicle __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__VelocityVehicle __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelocityVehicle_
{
  using Type = VelocityVehicle_<ContainerAllocator>;

  explicit VelocityVehicle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status_velocity_near_standstill = 0;
      this->qualifier_velocity_vehicle = 0;
      this->velocity_vehicle_event_data_qualifier = 0;
      this->velocity_vehicle = 0.0f;
      this->velocity_vehicle_invalid_flag = 0;
    }
  }

  explicit VelocityVehicle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status_velocity_near_standstill = 0;
      this->qualifier_velocity_vehicle = 0;
      this->velocity_vehicle_event_data_qualifier = 0;
      this->velocity_vehicle = 0.0f;
      this->velocity_vehicle_invalid_flag = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_velocity_near_standstill_type =
    uint8_t;
  _status_velocity_near_standstill_type status_velocity_near_standstill;
  using _qualifier_velocity_vehicle_type =
    uint8_t;
  _qualifier_velocity_vehicle_type qualifier_velocity_vehicle;
  using _velocity_vehicle_event_data_qualifier_type =
    uint8_t;
  _velocity_vehicle_event_data_qualifier_type velocity_vehicle_event_data_qualifier;
  using _velocity_vehicle_type =
    float;
  _velocity_vehicle_type velocity_vehicle;
  using _velocity_vehicle_invalid_flag_type =
    uint8_t;
  _velocity_vehicle_invalid_flag_type velocity_vehicle_invalid_flag;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status_velocity_near_standstill(
    const uint8_t & _arg)
  {
    this->status_velocity_near_standstill = _arg;
    return *this;
  }
  Type & set__qualifier_velocity_vehicle(
    const uint8_t & _arg)
  {
    this->qualifier_velocity_vehicle = _arg;
    return *this;
  }
  Type & set__velocity_vehicle_event_data_qualifier(
    const uint8_t & _arg)
  {
    this->velocity_vehicle_event_data_qualifier = _arg;
    return *this;
  }
  Type & set__velocity_vehicle(
    const float & _arg)
  {
    this->velocity_vehicle = _arg;
    return *this;
  }
  Type & set__velocity_vehicle_invalid_flag(
    const uint8_t & _arg)
  {
    this->velocity_vehicle_invalid_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::VelocityVehicle_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::VelocityVehicle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::VelocityVehicle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::VelocityVehicle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::VelocityVehicle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::VelocityVehicle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::VelocityVehicle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::VelocityVehicle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::VelocityVehicle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::VelocityVehicle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__VelocityVehicle
    std::shared_ptr<ars548_interface::msg::VelocityVehicle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__VelocityVehicle
    std::shared_ptr<ars548_interface::msg::VelocityVehicle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelocityVehicle_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status_velocity_near_standstill != other.status_velocity_near_standstill) {
      return false;
    }
    if (this->qualifier_velocity_vehicle != other.qualifier_velocity_vehicle) {
      return false;
    }
    if (this->velocity_vehicle_event_data_qualifier != other.velocity_vehicle_event_data_qualifier) {
      return false;
    }
    if (this->velocity_vehicle != other.velocity_vehicle) {
      return false;
    }
    if (this->velocity_vehicle_invalid_flag != other.velocity_vehicle_invalid_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelocityVehicle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelocityVehicle_

// alias to use template instance with default allocator
using VelocityVehicle =
  ars548_interface::msg::VelocityVehicle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__VELOCITY_VEHICLE__STRUCT_HPP_
