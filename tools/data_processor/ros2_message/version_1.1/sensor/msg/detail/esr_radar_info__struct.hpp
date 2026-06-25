// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor:msg/EsrRadarInfo.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__ESR_RADAR_INFO__STRUCT_HPP_
#define SENSOR__MSG__DETAIL__ESR_RADAR_INFO__STRUCT_HPP_

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
// Member 'localpose_stamped'
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
// Member 'object_data'
#include "sensor/msg/detail/esr_radar_object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sensor__msg__EsrRadarInfo __attribute__((deprecated))
#else
# define DEPRECATED__sensor__msg__EsrRadarInfo __declspec(deprecated)
#endif

namespace sensor
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EsrRadarInfo_
{
  using Type = EsrRadarInfo_<ContainerAllocator>;

  explicit EsrRadarInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    localpose_stamped(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->gps_time = 0.0;
      this->radar_id = 0l;
      this->object_num = 0l;
    }
  }

  explicit EsrRadarInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    localpose_stamped(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->gps_time = 0.0;
      this->radar_id = 0l;
      this->object_num = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _local_time_type =
    double;
  _local_time_type local_time;
  using _gps_time_type =
    double;
  _gps_time_type gps_time;
  using _localpose_stamped_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _localpose_stamped_type localpose_stamped;
  using _radar_id_type =
    int32_t;
  _radar_id_type radar_id;
  using _object_num_type =
    int32_t;
  _object_num_type object_num;
  using _object_data_type =
    std::vector<sensor::msg::EsrRadarObject_<ContainerAllocator>, typename ContainerAllocator::template rebind<sensor::msg::EsrRadarObject_<ContainerAllocator>>::other>;
  _object_data_type object_data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__local_time(
    const double & _arg)
  {
    this->local_time = _arg;
    return *this;
  }
  Type & set__gps_time(
    const double & _arg)
  {
    this->gps_time = _arg;
    return *this;
  }
  Type & set__localpose_stamped(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->localpose_stamped = _arg;
    return *this;
  }
  Type & set__radar_id(
    const int32_t & _arg)
  {
    this->radar_id = _arg;
    return *this;
  }
  Type & set__object_num(
    const int32_t & _arg)
  {
    this->object_num = _arg;
    return *this;
  }
  Type & set__object_data(
    const std::vector<sensor::msg::EsrRadarObject_<ContainerAllocator>, typename ContainerAllocator::template rebind<sensor::msg::EsrRadarObject_<ContainerAllocator>>::other> & _arg)
  {
    this->object_data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor::msg::EsrRadarInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor::msg::EsrRadarInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor::msg::EsrRadarInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor::msg::EsrRadarInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor::msg::EsrRadarInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::EsrRadarInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor::msg::EsrRadarInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::EsrRadarInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor::msg::EsrRadarInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor::msg::EsrRadarInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor__msg__EsrRadarInfo
    std::shared_ptr<sensor::msg::EsrRadarInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor__msg__EsrRadarInfo
    std::shared_ptr<sensor::msg::EsrRadarInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EsrRadarInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->local_time != other.local_time) {
      return false;
    }
    if (this->gps_time != other.gps_time) {
      return false;
    }
    if (this->localpose_stamped != other.localpose_stamped) {
      return false;
    }
    if (this->radar_id != other.radar_id) {
      return false;
    }
    if (this->object_num != other.object_num) {
      return false;
    }
    if (this->object_data != other.object_data) {
      return false;
    }
    return true;
  }
  bool operator!=(const EsrRadarInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EsrRadarInfo_

// alias to use template instance with default allocator
using EsrRadarInfo =
  sensor::msg::EsrRadarInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__ESR_RADAR_INFO__STRUCT_HPP_
