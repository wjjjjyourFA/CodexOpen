// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from self_state:msg/VehicleMotion.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__STRUCT_HPP_
#define SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__self_state__msg__VehicleMotion __attribute__((deprecated))
#else
# define DEPRECATED__self_state__msg__VehicleMotion __declspec(deprecated)
#endif

namespace self_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VehicleMotion_
{
  using Type = VehicleMotion_<ContainerAllocator>;

  explicit VehicleMotion_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->longitudinal_speed = 0.0;
      this->lateral_speed = 0.0;
      this->vertical_speed = 0.0;
      this->angular_x = 0.0;
      this->angular_y = 0.0;
      this->angular_z = 0.0;
      this->acc_x = 0.0;
      this->acc_y = 0.0;
      this->acc_z = 0.0;
    }
  }

  explicit VehicleMotion_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->longitudinal_speed = 0.0;
      this->lateral_speed = 0.0;
      this->vertical_speed = 0.0;
      this->angular_x = 0.0;
      this->angular_y = 0.0;
      this->angular_z = 0.0;
      this->acc_x = 0.0;
      this->acc_y = 0.0;
      this->acc_z = 0.0;
    }
  }

  // field types and members
  using _local_time_type =
    double;
  _local_time_type local_time;
  using _utc_time_type =
    double;
  _utc_time_type utc_time;
  using _message_num_type =
    int32_t;
  _message_num_type message_num;
  using _longitudinal_speed_type =
    double;
  _longitudinal_speed_type longitudinal_speed;
  using _lateral_speed_type =
    double;
  _lateral_speed_type lateral_speed;
  using _vertical_speed_type =
    double;
  _vertical_speed_type vertical_speed;
  using _angular_x_type =
    double;
  _angular_x_type angular_x;
  using _angular_y_type =
    double;
  _angular_y_type angular_y;
  using _angular_z_type =
    double;
  _angular_z_type angular_z;
  using _acc_x_type =
    double;
  _acc_x_type acc_x;
  using _acc_y_type =
    double;
  _acc_y_type acc_y;
  using _acc_z_type =
    double;
  _acc_z_type acc_z;

  // setters for named parameter idiom
  Type & set__local_time(
    const double & _arg)
  {
    this->local_time = _arg;
    return *this;
  }
  Type & set__utc_time(
    const double & _arg)
  {
    this->utc_time = _arg;
    return *this;
  }
  Type & set__message_num(
    const int32_t & _arg)
  {
    this->message_num = _arg;
    return *this;
  }
  Type & set__longitudinal_speed(
    const double & _arg)
  {
    this->longitudinal_speed = _arg;
    return *this;
  }
  Type & set__lateral_speed(
    const double & _arg)
  {
    this->lateral_speed = _arg;
    return *this;
  }
  Type & set__vertical_speed(
    const double & _arg)
  {
    this->vertical_speed = _arg;
    return *this;
  }
  Type & set__angular_x(
    const double & _arg)
  {
    this->angular_x = _arg;
    return *this;
  }
  Type & set__angular_y(
    const double & _arg)
  {
    this->angular_y = _arg;
    return *this;
  }
  Type & set__angular_z(
    const double & _arg)
  {
    this->angular_z = _arg;
    return *this;
  }
  Type & set__acc_x(
    const double & _arg)
  {
    this->acc_x = _arg;
    return *this;
  }
  Type & set__acc_y(
    const double & _arg)
  {
    this->acc_y = _arg;
    return *this;
  }
  Type & set__acc_z(
    const double & _arg)
  {
    this->acc_z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    self_state::msg::VehicleMotion_<ContainerAllocator> *;
  using ConstRawPtr =
    const self_state::msg::VehicleMotion_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<self_state::msg::VehicleMotion_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<self_state::msg::VehicleMotion_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      self_state::msg::VehicleMotion_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::VehicleMotion_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      self_state::msg::VehicleMotion_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::VehicleMotion_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<self_state::msg::VehicleMotion_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<self_state::msg::VehicleMotion_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__self_state__msg__VehicleMotion
    std::shared_ptr<self_state::msg::VehicleMotion_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__self_state__msg__VehicleMotion
    std::shared_ptr<self_state::msg::VehicleMotion_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VehicleMotion_ & other) const
  {
    if (this->local_time != other.local_time) {
      return false;
    }
    if (this->utc_time != other.utc_time) {
      return false;
    }
    if (this->message_num != other.message_num) {
      return false;
    }
    if (this->longitudinal_speed != other.longitudinal_speed) {
      return false;
    }
    if (this->lateral_speed != other.lateral_speed) {
      return false;
    }
    if (this->vertical_speed != other.vertical_speed) {
      return false;
    }
    if (this->angular_x != other.angular_x) {
      return false;
    }
    if (this->angular_y != other.angular_y) {
      return false;
    }
    if (this->angular_z != other.angular_z) {
      return false;
    }
    if (this->acc_x != other.acc_x) {
      return false;
    }
    if (this->acc_y != other.acc_y) {
      return false;
    }
    if (this->acc_z != other.acc_z) {
      return false;
    }
    return true;
  }
  bool operator!=(const VehicleMotion_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VehicleMotion_

// alias to use template instance with default allocator
using VehicleMotion =
  self_state::msg::VehicleMotion_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__VEHICLE_MOTION__STRUCT_HPP_
