// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from self_state:msg/LidarGlobalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__LIDAR_GLOBAL_POSE__STRUCT_HPP_
#define SELF_STATE__MSG__DETAIL__LIDAR_GLOBAL_POSE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'local_pose'
#include "self_state/msg/detail/local_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__self_state__msg__LidarGlobalPose __attribute__((deprecated))
#else
# define DEPRECATED__self_state__msg__LidarGlobalPose __declspec(deprecated)
#endif

namespace self_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LidarGlobalPose_
{
  using Type = LidarGlobalPose_<ContainerAllocator>;

  explicit LidarGlobalPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : local_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->pos_type = 0l;
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->x_speed = 0.0;
      this->y_speed = 0.0;
      this->z_speed = 0.0;
      this->azimuth = 0.0;
      this->pitch = 0.0;
      this->roll = 0.0;
    }
  }

  explicit LidarGlobalPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : local_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->pos_type = 0l;
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->x_speed = 0.0;
      this->y_speed = 0.0;
      this->z_speed = 0.0;
      this->azimuth = 0.0;
      this->pitch = 0.0;
      this->roll = 0.0;
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
  using _pos_type_type =
    int32_t;
  _pos_type_type pos_type;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _z_type =
    double;
  _z_type z;
  using _x_speed_type =
    double;
  _x_speed_type x_speed;
  using _y_speed_type =
    double;
  _y_speed_type y_speed;
  using _z_speed_type =
    double;
  _z_speed_type z_speed;
  using _azimuth_type =
    double;
  _azimuth_type azimuth;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _roll_type =
    double;
  _roll_type roll;
  using _local_pose_type =
    self_state::msg::LocalPose_<ContainerAllocator>;
  _local_pose_type local_pose;

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
  Type & set__pos_type(
    const int32_t & _arg)
  {
    this->pos_type = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const double & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__x_speed(
    const double & _arg)
  {
    this->x_speed = _arg;
    return *this;
  }
  Type & set__y_speed(
    const double & _arg)
  {
    this->y_speed = _arg;
    return *this;
  }
  Type & set__z_speed(
    const double & _arg)
  {
    this->z_speed = _arg;
    return *this;
  }
  Type & set__azimuth(
    const double & _arg)
  {
    this->azimuth = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__local_pose(
    const self_state::msg::LocalPose_<ContainerAllocator> & _arg)
  {
    this->local_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    self_state::msg::LidarGlobalPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const self_state::msg::LidarGlobalPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<self_state::msg::LidarGlobalPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<self_state::msg::LidarGlobalPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      self_state::msg::LidarGlobalPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::LidarGlobalPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      self_state::msg::LidarGlobalPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::LidarGlobalPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<self_state::msg::LidarGlobalPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<self_state::msg::LidarGlobalPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__self_state__msg__LidarGlobalPose
    std::shared_ptr<self_state::msg::LidarGlobalPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__self_state__msg__LidarGlobalPose
    std::shared_ptr<self_state::msg::LidarGlobalPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarGlobalPose_ & other) const
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
    if (this->pos_type != other.pos_type) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->x_speed != other.x_speed) {
      return false;
    }
    if (this->y_speed != other.y_speed) {
      return false;
    }
    if (this->z_speed != other.z_speed) {
      return false;
    }
    if (this->azimuth != other.azimuth) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->local_pose != other.local_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarGlobalPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarGlobalPose_

// alias to use template instance with default allocator
using LidarGlobalPose =
  self_state::msg::LidarGlobalPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__LIDAR_GLOBAL_POSE__STRUCT_HPP_
