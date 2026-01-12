// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor:msg/EsrRadarObject.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__ESR_RADAR_OBJECT__STRUCT_HPP_
#define SENSOR__MSG__DETAIL__ESR_RADAR_OBJECT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__sensor__msg__EsrRadarObject __attribute__((deprecated))
#else
# define DEPRECATED__sensor__msg__EsrRadarObject __declspec(deprecated)
#endif

namespace sensor
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EsrRadarObject_
{
  using Type = EsrRadarObject_<ContainerAllocator>;

  explicit EsrRadarObject_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_i_d = 0;
      this->range = 0.0;
      this->angle = 0.0;
      this->front_distance = 0.0;
      this->left_distance = 0.0;
      this->speed = 0.0;
      this->height = 0.0;
      this->width = 0.0;
      this->range_rate = 0.0;
      this->lat_rate = 0.0;
      this->track_status = 0;
      this->is_acc_target = 0;
      this->is_cmbb_target = 0;
      this->is_fcw_target = 0;
      this->type = 0;
      this->confidence = 0;
      this->rcsvalue = 0;
    }
  }

  explicit EsrRadarObject_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->target_i_d = 0;
      this->range = 0.0;
      this->angle = 0.0;
      this->front_distance = 0.0;
      this->left_distance = 0.0;
      this->speed = 0.0;
      this->height = 0.0;
      this->width = 0.0;
      this->range_rate = 0.0;
      this->lat_rate = 0.0;
      this->track_status = 0;
      this->is_acc_target = 0;
      this->is_cmbb_target = 0;
      this->is_fcw_target = 0;
      this->type = 0;
      this->confidence = 0;
      this->rcsvalue = 0;
    }
  }

  // field types and members
  using _target_i_d_type =
    int8_t;
  _target_i_d_type target_i_d;
  using _range_type =
    double;
  _range_type range;
  using _angle_type =
    double;
  _angle_type angle;
  using _front_distance_type =
    double;
  _front_distance_type front_distance;
  using _left_distance_type =
    double;
  _left_distance_type left_distance;
  using _speed_type =
    double;
  _speed_type speed;
  using _height_type =
    double;
  _height_type height;
  using _width_type =
    double;
  _width_type width;
  using _range_rate_type =
    double;
  _range_rate_type range_rate;
  using _lat_rate_type =
    double;
  _lat_rate_type lat_rate;
  using _track_status_type =
    int8_t;
  _track_status_type track_status;
  using _is_acc_target_type =
    int8_t;
  _is_acc_target_type is_acc_target;
  using _is_cmbb_target_type =
    int8_t;
  _is_cmbb_target_type is_cmbb_target;
  using _is_fcw_target_type =
    int8_t;
  _is_fcw_target_type is_fcw_target;
  using _type_type =
    int8_t;
  _type_type type;
  using _confidence_type =
    int8_t;
  _confidence_type confidence;
  using _rcsvalue_type =
    int8_t;
  _rcsvalue_type rcsvalue;

  // setters for named parameter idiom
  Type & set__target_i_d(
    const int8_t & _arg)
  {
    this->target_i_d = _arg;
    return *this;
  }
  Type & set__range(
    const double & _arg)
  {
    this->range = _arg;
    return *this;
  }
  Type & set__angle(
    const double & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__front_distance(
    const double & _arg)
  {
    this->front_distance = _arg;
    return *this;
  }
  Type & set__left_distance(
    const double & _arg)
  {
    this->left_distance = _arg;
    return *this;
  }
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__height(
    const double & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__width(
    const double & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__range_rate(
    const double & _arg)
  {
    this->range_rate = _arg;
    return *this;
  }
  Type & set__lat_rate(
    const double & _arg)
  {
    this->lat_rate = _arg;
    return *this;
  }
  Type & set__track_status(
    const int8_t & _arg)
  {
    this->track_status = _arg;
    return *this;
  }
  Type & set__is_acc_target(
    const int8_t & _arg)
  {
    this->is_acc_target = _arg;
    return *this;
  }
  Type & set__is_cmbb_target(
    const int8_t & _arg)
  {
    this->is_cmbb_target = _arg;
    return *this;
  }
  Type & set__is_fcw_target(
    const int8_t & _arg)
  {
    this->is_fcw_target = _arg;
    return *this;
  }
  Type & set__type(
    const int8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__confidence(
    const int8_t & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__rcsvalue(
    const int8_t & _arg)
  {
    this->rcsvalue = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor::msg::EsrRadarObject_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor::msg::EsrRadarObject_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor::msg::EsrRadarObject_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor::msg::EsrRadarObject_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor::msg::EsrRadarObject_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::EsrRadarObject_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor::msg::EsrRadarObject_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::EsrRadarObject_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor::msg::EsrRadarObject_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor::msg::EsrRadarObject_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor__msg__EsrRadarObject
    std::shared_ptr<sensor::msg::EsrRadarObject_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor__msg__EsrRadarObject
    std::shared_ptr<sensor::msg::EsrRadarObject_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EsrRadarObject_ & other) const
  {
    if (this->target_i_d != other.target_i_d) {
      return false;
    }
    if (this->range != other.range) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->front_distance != other.front_distance) {
      return false;
    }
    if (this->left_distance != other.left_distance) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->range_rate != other.range_rate) {
      return false;
    }
    if (this->lat_rate != other.lat_rate) {
      return false;
    }
    if (this->track_status != other.track_status) {
      return false;
    }
    if (this->is_acc_target != other.is_acc_target) {
      return false;
    }
    if (this->is_cmbb_target != other.is_cmbb_target) {
      return false;
    }
    if (this->is_fcw_target != other.is_fcw_target) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->rcsvalue != other.rcsvalue) {
      return false;
    }
    return true;
  }
  bool operator!=(const EsrRadarObject_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EsrRadarObject_

// alias to use template instance with default allocator
using EsrRadarObject =
  sensor::msg::EsrRadarObject_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__ESR_RADAR_OBJECT__STRUCT_HPP_
