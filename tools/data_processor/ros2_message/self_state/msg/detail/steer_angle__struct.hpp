// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from self_state:msg/SteerAngle.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__STEER_ANGLE__STRUCT_HPP_
#define SELF_STATE__MSG__DETAIL__STEER_ANGLE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__self_state__msg__SteerAngle __attribute__((deprecated))
#else
# define DEPRECATED__self_state__msg__SteerAngle __declspec(deprecated)
#endif

namespace self_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SteerAngle_
{
  using Type = SteerAngle_<ContainerAllocator>;

  explicit SteerAngle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->actual_front_wheel_angle = 0.0;
      this->desired_front_wheel_angle = 0.0;
      this->actual_curvature = 0.0;
      this->desired_curvature = 0.0;
      this->bcan_control_flag = 0l;
      this->left_light_flag = 0l;
      this->right_light_flag = 0l;
    }
  }

  explicit SteerAngle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->actual_front_wheel_angle = 0.0;
      this->desired_front_wheel_angle = 0.0;
      this->actual_curvature = 0.0;
      this->desired_curvature = 0.0;
      this->bcan_control_flag = 0l;
      this->left_light_flag = 0l;
      this->right_light_flag = 0l;
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
  using _actual_front_wheel_angle_type =
    double;
  _actual_front_wheel_angle_type actual_front_wheel_angle;
  using _desired_front_wheel_angle_type =
    double;
  _desired_front_wheel_angle_type desired_front_wheel_angle;
  using _actual_curvature_type =
    double;
  _actual_curvature_type actual_curvature;
  using _desired_curvature_type =
    double;
  _desired_curvature_type desired_curvature;
  using _bcan_control_flag_type =
    int32_t;
  _bcan_control_flag_type bcan_control_flag;
  using _left_light_flag_type =
    int32_t;
  _left_light_flag_type left_light_flag;
  using _right_light_flag_type =
    int32_t;
  _right_light_flag_type right_light_flag;

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
  Type & set__actual_front_wheel_angle(
    const double & _arg)
  {
    this->actual_front_wheel_angle = _arg;
    return *this;
  }
  Type & set__desired_front_wheel_angle(
    const double & _arg)
  {
    this->desired_front_wheel_angle = _arg;
    return *this;
  }
  Type & set__actual_curvature(
    const double & _arg)
  {
    this->actual_curvature = _arg;
    return *this;
  }
  Type & set__desired_curvature(
    const double & _arg)
  {
    this->desired_curvature = _arg;
    return *this;
  }
  Type & set__bcan_control_flag(
    const int32_t & _arg)
  {
    this->bcan_control_flag = _arg;
    return *this;
  }
  Type & set__left_light_flag(
    const int32_t & _arg)
  {
    this->left_light_flag = _arg;
    return *this;
  }
  Type & set__right_light_flag(
    const int32_t & _arg)
  {
    this->right_light_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    self_state::msg::SteerAngle_<ContainerAllocator> *;
  using ConstRawPtr =
    const self_state::msg::SteerAngle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<self_state::msg::SteerAngle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<self_state::msg::SteerAngle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      self_state::msg::SteerAngle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::SteerAngle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      self_state::msg::SteerAngle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::SteerAngle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<self_state::msg::SteerAngle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<self_state::msg::SteerAngle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__self_state__msg__SteerAngle
    std::shared_ptr<self_state::msg::SteerAngle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__self_state__msg__SteerAngle
    std::shared_ptr<self_state::msg::SteerAngle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SteerAngle_ & other) const
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
    if (this->actual_front_wheel_angle != other.actual_front_wheel_angle) {
      return false;
    }
    if (this->desired_front_wheel_angle != other.desired_front_wheel_angle) {
      return false;
    }
    if (this->actual_curvature != other.actual_curvature) {
      return false;
    }
    if (this->desired_curvature != other.desired_curvature) {
      return false;
    }
    if (this->bcan_control_flag != other.bcan_control_flag) {
      return false;
    }
    if (this->left_light_flag != other.left_light_flag) {
      return false;
    }
    if (this->right_light_flag != other.right_light_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const SteerAngle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SteerAngle_

// alias to use template instance with default allocator
using SteerAngle =
  self_state::msg::SteerAngle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__STEER_ANGLE__STRUCT_HPP_
