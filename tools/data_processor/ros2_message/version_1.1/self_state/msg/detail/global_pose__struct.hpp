// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from self_state:msg/GlobalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__GLOBAL_POSE__STRUCT_HPP_
#define SELF_STATE__MSG__DETAIL__GLOBAL_POSE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'ins_status'
#include "self_state/msg/detail/ins_status__struct.hpp"
// Member 'pos_type'
#include "sensor/msg/detail/pos_type__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__self_state__msg__GlobalPose __attribute__((deprecated))
#else
# define DEPRECATED__self_state__msg__GlobalPose __declspec(deprecated)
#endif

namespace self_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GlobalPose_
{
  using Type = GlobalPose_<ContainerAllocator>;

  explicit GlobalPose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ins_status(_init),
    pos_type(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->gauss_x = 0.0;
      this->gauss_y = 0.0;
      this->height = 0.0;
      this->v_north = 0.0;
      this->v_east = 0.0;
      this->v_up = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->azimuth = 0.0;
      this->dev_gauss_x = 0.0;
      this->dev_gauss_y = 0.0;
      this->dev_height = 0.0;
      this->dev_v_north = 0.0;
      this->dev_v_east = 0.0;
      this->dev_v_up = 0.0;
      this->dev_roll = 0.0;
      this->dev_pitch = 0.0;
      this->dev_azimuth = 0.0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      std::fill<typename std::array<double, 4>::iterator, double>(this->reserved.begin(), this->reserved.end(), 0.0);
    }
  }

  explicit GlobalPose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : ins_status(_alloc, _init),
    pos_type(_alloc, _init),
    reserved(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->gauss_x = 0.0;
      this->gauss_y = 0.0;
      this->height = 0.0;
      this->v_north = 0.0;
      this->v_east = 0.0;
      this->v_up = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->azimuth = 0.0;
      this->dev_gauss_x = 0.0;
      this->dev_gauss_y = 0.0;
      this->dev_height = 0.0;
      this->dev_v_north = 0.0;
      this->dev_v_east = 0.0;
      this->dev_v_up = 0.0;
      this->dev_roll = 0.0;
      this->dev_pitch = 0.0;
      this->dev_azimuth = 0.0;
      this->latitude = 0.0;
      this->longitude = 0.0;
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
  using _ins_status_type =
    self_state::msg::InsStatus_<ContainerAllocator>;
  _ins_status_type ins_status;
  using _pos_type_type =
    sensor::msg::PosType_<ContainerAllocator>;
  _pos_type_type pos_type;
  using _gauss_x_type =
    double;
  _gauss_x_type gauss_x;
  using _gauss_y_type =
    double;
  _gauss_y_type gauss_y;
  using _height_type =
    double;
  _height_type height;
  using _v_north_type =
    double;
  _v_north_type v_north;
  using _v_east_type =
    double;
  _v_east_type v_east;
  using _v_up_type =
    double;
  _v_up_type v_up;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _azimuth_type =
    double;
  _azimuth_type azimuth;
  using _dev_gauss_x_type =
    double;
  _dev_gauss_x_type dev_gauss_x;
  using _dev_gauss_y_type =
    double;
  _dev_gauss_y_type dev_gauss_y;
  using _dev_height_type =
    double;
  _dev_height_type dev_height;
  using _dev_v_north_type =
    double;
  _dev_v_north_type dev_v_north;
  using _dev_v_east_type =
    double;
  _dev_v_east_type dev_v_east;
  using _dev_v_up_type =
    double;
  _dev_v_up_type dev_v_up;
  using _dev_roll_type =
    double;
  _dev_roll_type dev_roll;
  using _dev_pitch_type =
    double;
  _dev_pitch_type dev_pitch;
  using _dev_azimuth_type =
    double;
  _dev_azimuth_type dev_azimuth;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
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
  Type & set__ins_status(
    const self_state::msg::InsStatus_<ContainerAllocator> & _arg)
  {
    this->ins_status = _arg;
    return *this;
  }
  Type & set__pos_type(
    const sensor::msg::PosType_<ContainerAllocator> & _arg)
  {
    this->pos_type = _arg;
    return *this;
  }
  Type & set__gauss_x(
    const double & _arg)
  {
    this->gauss_x = _arg;
    return *this;
  }
  Type & set__gauss_y(
    const double & _arg)
  {
    this->gauss_y = _arg;
    return *this;
  }
  Type & set__height(
    const double & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__v_north(
    const double & _arg)
  {
    this->v_north = _arg;
    return *this;
  }
  Type & set__v_east(
    const double & _arg)
  {
    this->v_east = _arg;
    return *this;
  }
  Type & set__v_up(
    const double & _arg)
  {
    this->v_up = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__azimuth(
    const double & _arg)
  {
    this->azimuth = _arg;
    return *this;
  }
  Type & set__dev_gauss_x(
    const double & _arg)
  {
    this->dev_gauss_x = _arg;
    return *this;
  }
  Type & set__dev_gauss_y(
    const double & _arg)
  {
    this->dev_gauss_y = _arg;
    return *this;
  }
  Type & set__dev_height(
    const double & _arg)
  {
    this->dev_height = _arg;
    return *this;
  }
  Type & set__dev_v_north(
    const double & _arg)
  {
    this->dev_v_north = _arg;
    return *this;
  }
  Type & set__dev_v_east(
    const double & _arg)
  {
    this->dev_v_east = _arg;
    return *this;
  }
  Type & set__dev_v_up(
    const double & _arg)
  {
    this->dev_v_up = _arg;
    return *this;
  }
  Type & set__dev_roll(
    const double & _arg)
  {
    this->dev_roll = _arg;
    return *this;
  }
  Type & set__dev_pitch(
    const double & _arg)
  {
    this->dev_pitch = _arg;
    return *this;
  }
  Type & set__dev_azimuth(
    const double & _arg)
  {
    this->dev_azimuth = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__reserved(
    const std::array<double, 4> & _arg)
  {
    this->reserved = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    self_state::msg::GlobalPose_<ContainerAllocator> *;
  using ConstRawPtr =
    const self_state::msg::GlobalPose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<self_state::msg::GlobalPose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<self_state::msg::GlobalPose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      self_state::msg::GlobalPose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::GlobalPose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      self_state::msg::GlobalPose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::GlobalPose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<self_state::msg::GlobalPose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<self_state::msg::GlobalPose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__self_state__msg__GlobalPose
    std::shared_ptr<self_state::msg::GlobalPose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__self_state__msg__GlobalPose
    std::shared_ptr<self_state::msg::GlobalPose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GlobalPose_ & other) const
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
    if (this->ins_status != other.ins_status) {
      return false;
    }
    if (this->pos_type != other.pos_type) {
      return false;
    }
    if (this->gauss_x != other.gauss_x) {
      return false;
    }
    if (this->gauss_y != other.gauss_y) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->v_north != other.v_north) {
      return false;
    }
    if (this->v_east != other.v_east) {
      return false;
    }
    if (this->v_up != other.v_up) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->azimuth != other.azimuth) {
      return false;
    }
    if (this->dev_gauss_x != other.dev_gauss_x) {
      return false;
    }
    if (this->dev_gauss_y != other.dev_gauss_y) {
      return false;
    }
    if (this->dev_height != other.dev_height) {
      return false;
    }
    if (this->dev_v_north != other.dev_v_north) {
      return false;
    }
    if (this->dev_v_east != other.dev_v_east) {
      return false;
    }
    if (this->dev_v_up != other.dev_v_up) {
      return false;
    }
    if (this->dev_roll != other.dev_roll) {
      return false;
    }
    if (this->dev_pitch != other.dev_pitch) {
      return false;
    }
    if (this->dev_azimuth != other.dev_azimuth) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->reserved != other.reserved) {
      return false;
    }
    return true;
  }
  bool operator!=(const GlobalPose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GlobalPose_

// alias to use template instance with default allocator
using GlobalPose =
  self_state::msg::GlobalPose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__GLOBAL_POSE__STRUCT_HPP_
