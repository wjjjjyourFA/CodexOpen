// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from self_state:msg/SpeedStatus.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__SPEED_STATUS__STRUCT_HPP_
#define SELF_STATE__MSG__DETAIL__SPEED_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__self_state__msg__SpeedStatus __attribute__((deprecated))
#else
# define DEPRECATED__self_state__msg__SpeedStatus __declspec(deprecated)
#endif

namespace self_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SpeedStatus_
{
  using Type = SpeedStatus_<ContainerAllocator>;

  explicit SpeedStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->desired_speed = 0.0;
      this->desired_acc = 0.0;
      this->current_speed = 0.0;
      this->current_acc = 0.0;
      this->desired_brake = 0.0;
      this->current_brake = 0.0;
      this->desired_fuel = 0.0;
      this->current_fuel = 0.0;
      this->desired_trans_pos = 0l;
      this->current_trans_pos = 0l;
      this->hard_switch_on = 0l;
      this->emergence_flag = 0l;
      this->bcan_control_flag = 0l;
      this->horn_on_flag = 0l;
      this->emergency_lighton_flag = 0l;
    }
  }

  explicit SpeedStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->desired_speed = 0.0;
      this->desired_acc = 0.0;
      this->current_speed = 0.0;
      this->current_acc = 0.0;
      this->desired_brake = 0.0;
      this->current_brake = 0.0;
      this->desired_fuel = 0.0;
      this->current_fuel = 0.0;
      this->desired_trans_pos = 0l;
      this->current_trans_pos = 0l;
      this->hard_switch_on = 0l;
      this->emergence_flag = 0l;
      this->bcan_control_flag = 0l;
      this->horn_on_flag = 0l;
      this->emergency_lighton_flag = 0l;
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
  using _desired_speed_type =
    double;
  _desired_speed_type desired_speed;
  using _desired_acc_type =
    double;
  _desired_acc_type desired_acc;
  using _current_speed_type =
    double;
  _current_speed_type current_speed;
  using _current_acc_type =
    double;
  _current_acc_type current_acc;
  using _desired_brake_type =
    double;
  _desired_brake_type desired_brake;
  using _current_brake_type =
    double;
  _current_brake_type current_brake;
  using _desired_fuel_type =
    double;
  _desired_fuel_type desired_fuel;
  using _current_fuel_type =
    double;
  _current_fuel_type current_fuel;
  using _desired_trans_pos_type =
    int32_t;
  _desired_trans_pos_type desired_trans_pos;
  using _current_trans_pos_type =
    int32_t;
  _current_trans_pos_type current_trans_pos;
  using _hard_switch_on_type =
    int32_t;
  _hard_switch_on_type hard_switch_on;
  using _emergence_flag_type =
    int32_t;
  _emergence_flag_type emergence_flag;
  using _bcan_control_flag_type =
    int32_t;
  _bcan_control_flag_type bcan_control_flag;
  using _horn_on_flag_type =
    int32_t;
  _horn_on_flag_type horn_on_flag;
  using _emergency_lighton_flag_type =
    int32_t;
  _emergency_lighton_flag_type emergency_lighton_flag;

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
  Type & set__desired_speed(
    const double & _arg)
  {
    this->desired_speed = _arg;
    return *this;
  }
  Type & set__desired_acc(
    const double & _arg)
  {
    this->desired_acc = _arg;
    return *this;
  }
  Type & set__current_speed(
    const double & _arg)
  {
    this->current_speed = _arg;
    return *this;
  }
  Type & set__current_acc(
    const double & _arg)
  {
    this->current_acc = _arg;
    return *this;
  }
  Type & set__desired_brake(
    const double & _arg)
  {
    this->desired_brake = _arg;
    return *this;
  }
  Type & set__current_brake(
    const double & _arg)
  {
    this->current_brake = _arg;
    return *this;
  }
  Type & set__desired_fuel(
    const double & _arg)
  {
    this->desired_fuel = _arg;
    return *this;
  }
  Type & set__current_fuel(
    const double & _arg)
  {
    this->current_fuel = _arg;
    return *this;
  }
  Type & set__desired_trans_pos(
    const int32_t & _arg)
  {
    this->desired_trans_pos = _arg;
    return *this;
  }
  Type & set__current_trans_pos(
    const int32_t & _arg)
  {
    this->current_trans_pos = _arg;
    return *this;
  }
  Type & set__hard_switch_on(
    const int32_t & _arg)
  {
    this->hard_switch_on = _arg;
    return *this;
  }
  Type & set__emergence_flag(
    const int32_t & _arg)
  {
    this->emergence_flag = _arg;
    return *this;
  }
  Type & set__bcan_control_flag(
    const int32_t & _arg)
  {
    this->bcan_control_flag = _arg;
    return *this;
  }
  Type & set__horn_on_flag(
    const int32_t & _arg)
  {
    this->horn_on_flag = _arg;
    return *this;
  }
  Type & set__emergency_lighton_flag(
    const int32_t & _arg)
  {
    this->emergency_lighton_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    self_state::msg::SpeedStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const self_state::msg::SpeedStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<self_state::msg::SpeedStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<self_state::msg::SpeedStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      self_state::msg::SpeedStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::SpeedStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      self_state::msg::SpeedStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::SpeedStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<self_state::msg::SpeedStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<self_state::msg::SpeedStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__self_state__msg__SpeedStatus
    std::shared_ptr<self_state::msg::SpeedStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__self_state__msg__SpeedStatus
    std::shared_ptr<self_state::msg::SpeedStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpeedStatus_ & other) const
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
    if (this->desired_speed != other.desired_speed) {
      return false;
    }
    if (this->desired_acc != other.desired_acc) {
      return false;
    }
    if (this->current_speed != other.current_speed) {
      return false;
    }
    if (this->current_acc != other.current_acc) {
      return false;
    }
    if (this->desired_brake != other.desired_brake) {
      return false;
    }
    if (this->current_brake != other.current_brake) {
      return false;
    }
    if (this->desired_fuel != other.desired_fuel) {
      return false;
    }
    if (this->current_fuel != other.current_fuel) {
      return false;
    }
    if (this->desired_trans_pos != other.desired_trans_pos) {
      return false;
    }
    if (this->current_trans_pos != other.current_trans_pos) {
      return false;
    }
    if (this->hard_switch_on != other.hard_switch_on) {
      return false;
    }
    if (this->emergence_flag != other.emergence_flag) {
      return false;
    }
    if (this->bcan_control_flag != other.bcan_control_flag) {
      return false;
    }
    if (this->horn_on_flag != other.horn_on_flag) {
      return false;
    }
    if (this->emergency_lighton_flag != other.emergency_lighton_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpeedStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpeedStatus_

// alias to use template instance with default allocator
using SpeedStatus =
  self_state::msg::SpeedStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__SPEED_STATUS__STRUCT_HPP_
