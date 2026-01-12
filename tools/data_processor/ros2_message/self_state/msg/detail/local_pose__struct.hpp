// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from self_state:msg/LocalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__LOCAL_POSE__STRUCT_HPP_
#define SELF_STATE__MSG__DETAIL__LOCAL_POSE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__self_state__msg__LocalPose __attribute__((deprecated))
#else
# define DEPRECATED__self_state__msg__LocalPose __declspec(deprecated)
#endif

namespace self_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalPose_
{
  using Type = LocalPose_<ContainerAllocator>;

  explicit LocalPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->dr_x = 0.0;
      this->dr_y = 0.0;
      this->dr_z = 0.0;
      this->dr_roll = 0.0;
      this->dr_pitch = 0.0;
      this->dr_heading = 0.0;
      this->vehicle_speed = 0.0;
      this->speed_x = 0.0;
      this->speed_y = 0.0;
      this->speed_z = 0.0;
      this->driving_direction = 0;
      std::fill<typename std::array<double, 4>::iterator, double>(this->reserved.begin(), this->reserved.end(), 0.0);
    }
  }

  explicit LocalPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reserved(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->dr_x = 0.0;
      this->dr_y = 0.0;
      this->dr_z = 0.0;
      this->dr_roll = 0.0;
      this->dr_pitch = 0.0;
      this->dr_heading = 0.0;
      this->vehicle_speed = 0.0;
      this->speed_x = 0.0;
      this->speed_y = 0.0;
      this->speed_z = 0.0;
      this->driving_direction = 0;
      std::fill<typename std::array<double, 4>::iterator, double>(this->reserved.begin(), this->reserved.end(), 0.0);
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
  using _dr_x_type =
    double;
  _dr_x_type dr_x;
  using _dr_y_type =
    double;
  _dr_y_type dr_y;
  using _dr_z_type =
    double;
  _dr_z_type dr_z;
  using _dr_roll_type =
    double;
  _dr_roll_type dr_roll;
  using _dr_pitch_type =
    double;
  _dr_pitch_type dr_pitch;
  using _dr_heading_type =
    double;
  _dr_heading_type dr_heading;
  using _vehicle_speed_type =
    double;
  _vehicle_speed_type vehicle_speed;
  using _speed_x_type =
    double;
  _speed_x_type speed_x;
  using _speed_y_type =
    double;
  _speed_y_type speed_y;
  using _speed_z_type =
    double;
  _speed_z_type speed_z;
  using _driving_direction_type =
    int8_t;
  _driving_direction_type driving_direction;
  using _reserved_type =
    std::array<double, 4>;
  _reserved_type reserved;

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
  Type & set__dr_x(
    const double & _arg)
  {
    this->dr_x = _arg;
    return *this;
  }
  Type & set__dr_y(
    const double & _arg)
  {
    this->dr_y = _arg;
    return *this;
  }
  Type & set__dr_z(
    const double & _arg)
  {
    this->dr_z = _arg;
    return *this;
  }
  Type & set__dr_roll(
    const double & _arg)
  {
    this->dr_roll = _arg;
    return *this;
  }
  Type & set__dr_pitch(
    const double & _arg)
  {
    this->dr_pitch = _arg;
    return *this;
  }
  Type & set__dr_heading(
    const double & _arg)
  {
    this->dr_heading = _arg;
    return *this;
  }
  Type & set__vehicle_speed(
    const double & _arg)
  {
    this->vehicle_speed = _arg;
    return *this;
  }
  Type & set__speed_x(
    const double & _arg)
  {
    this->speed_x = _arg;
    return *this;
  }
  Type & set__speed_y(
    const double & _arg)
  {
    this->speed_y = _arg;
    return *this;
  }
  Type & set__speed_z(
    const double & _arg)
  {
    this->speed_z = _arg;
    return *this;
  }
  Type & set__driving_direction(
    const int8_t & _arg)
  {
    this->driving_direction = _arg;
    return *this;
  }
  Type & set__reserved(
    const std::array<double, 4> & _arg)
  {
    this->reserved = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t STATUS_PARK =
    0;
  static constexpr int8_t STATUS_FORWARD =
    1;
  static constexpr int8_t STATUS_BACKWARD =
    -1;

  // pointer types
  using RawPtr =
    self_state::msg::LocalPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const self_state::msg::LocalPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<self_state::msg::LocalPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<self_state::msg::LocalPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      self_state::msg::LocalPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::LocalPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      self_state::msg::LocalPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::LocalPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<self_state::msg::LocalPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<self_state::msg::LocalPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__self_state__msg__LocalPose
    std::shared_ptr<self_state::msg::LocalPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__self_state__msg__LocalPose
    std::shared_ptr<self_state::msg::LocalPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalPose_ & other) const
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
    if (this->dr_x != other.dr_x) {
      return false;
    }
    if (this->dr_y != other.dr_y) {
      return false;
    }
    if (this->dr_z != other.dr_z) {
      return false;
    }
    if (this->dr_roll != other.dr_roll) {
      return false;
    }
    if (this->dr_pitch != other.dr_pitch) {
      return false;
    }
    if (this->dr_heading != other.dr_heading) {
      return false;
    }
    if (this->vehicle_speed != other.vehicle_speed) {
      return false;
    }
    if (this->speed_x != other.speed_x) {
      return false;
    }
    if (this->speed_y != other.speed_y) {
      return false;
    }
    if (this->speed_z != other.speed_z) {
      return false;
    }
    if (this->driving_direction != other.driving_direction) {
      return false;
    }
    if (this->reserved != other.reserved) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalPose_

// alias to use template instance with default allocator
using LocalPose =
  self_state::msg::LocalPose_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t LocalPose_<ContainerAllocator>::STATUS_PARK;
template<typename ContainerAllocator>
constexpr int8_t LocalPose_<ContainerAllocator>::STATUS_FORWARD;
template<typename ContainerAllocator>
constexpr int8_t LocalPose_<ContainerAllocator>::STATUS_BACKWARD;

}  // namespace msg

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__LOCAL_POSE__STRUCT_HPP_
