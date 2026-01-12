// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor:msg/BestGnss.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__BEST_GNSS__STRUCT_HPP_
#define SENSOR__MSG__DETAIL__BEST_GNSS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'sol_status'
#include "sensor/msg/detail/gnss_solution_status__struct.hpp"
// Member 'pos_type'
#include "sensor/msg/detail/pos_type__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sensor__msg__BestGnss __attribute__((deprecated))
#else
# define DEPRECATED__sensor__msg__BestGnss __declspec(deprecated)
#endif

namespace sensor
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BestGnss_
{
  using Type = BestGnss_<ContainerAllocator>;

  explicit BestGnss_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sol_status(_init),
    pos_type(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->u_t_c_time = 0.0;
      this->message_num = 0l;
      this->num_satellite_tracked = 0.0;
      this->num_satellite_used = 0.0;
      this->latitude_gnss = 0.0;
      this->longitude_gnss = 0.0;
      this->height_gnss = 0.0;
    }
  }

  explicit BestGnss_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : sol_status(_alloc, _init),
    pos_type(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->u_t_c_time = 0.0;
      this->message_num = 0l;
      this->num_satellite_tracked = 0.0;
      this->num_satellite_used = 0.0;
      this->latitude_gnss = 0.0;
      this->longitude_gnss = 0.0;
      this->height_gnss = 0.0;
    }
  }

  // field types and members
  using _local_time_type =
    double;
  _local_time_type local_time;
  using _u_t_c_time_type =
    double;
  _u_t_c_time_type u_t_c_time;
  using _message_num_type =
    int32_t;
  _message_num_type message_num;
  using _sol_status_type =
    sensor::msg::GnssSolutionStatus_<ContainerAllocator>;
  _sol_status_type sol_status;
  using _pos_type_type =
    sensor::msg::PosType_<ContainerAllocator>;
  _pos_type_type pos_type;
  using _num_satellite_tracked_type =
    double;
  _num_satellite_tracked_type num_satellite_tracked;
  using _num_satellite_used_type =
    double;
  _num_satellite_used_type num_satellite_used;
  using _latitude_gnss_type =
    double;
  _latitude_gnss_type latitude_gnss;
  using _longitude_gnss_type =
    double;
  _longitude_gnss_type longitude_gnss;
  using _height_gnss_type =
    double;
  _height_gnss_type height_gnss;

  // setters for named parameter idiom
  Type & set__local_time(
    const double & _arg)
  {
    this->local_time = _arg;
    return *this;
  }
  Type & set__u_t_c_time(
    const double & _arg)
  {
    this->u_t_c_time = _arg;
    return *this;
  }
  Type & set__message_num(
    const int32_t & _arg)
  {
    this->message_num = _arg;
    return *this;
  }
  Type & set__sol_status(
    const sensor::msg::GnssSolutionStatus_<ContainerAllocator> & _arg)
  {
    this->sol_status = _arg;
    return *this;
  }
  Type & set__pos_type(
    const sensor::msg::PosType_<ContainerAllocator> & _arg)
  {
    this->pos_type = _arg;
    return *this;
  }
  Type & set__num_satellite_tracked(
    const double & _arg)
  {
    this->num_satellite_tracked = _arg;
    return *this;
  }
  Type & set__num_satellite_used(
    const double & _arg)
  {
    this->num_satellite_used = _arg;
    return *this;
  }
  Type & set__latitude_gnss(
    const double & _arg)
  {
    this->latitude_gnss = _arg;
    return *this;
  }
  Type & set__longitude_gnss(
    const double & _arg)
  {
    this->longitude_gnss = _arg;
    return *this;
  }
  Type & set__height_gnss(
    const double & _arg)
  {
    this->height_gnss = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensor::msg::BestGnss_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor::msg::BestGnss_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor::msg::BestGnss_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor::msg::BestGnss_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor::msg::BestGnss_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::BestGnss_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor::msg::BestGnss_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::BestGnss_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor::msg::BestGnss_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor::msg::BestGnss_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor__msg__BestGnss
    std::shared_ptr<sensor::msg::BestGnss_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor__msg__BestGnss
    std::shared_ptr<sensor::msg::BestGnss_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BestGnss_ & other) const
  {
    if (this->local_time != other.local_time) {
      return false;
    }
    if (this->u_t_c_time != other.u_t_c_time) {
      return false;
    }
    if (this->message_num != other.message_num) {
      return false;
    }
    if (this->sol_status != other.sol_status) {
      return false;
    }
    if (this->pos_type != other.pos_type) {
      return false;
    }
    if (this->num_satellite_tracked != other.num_satellite_tracked) {
      return false;
    }
    if (this->num_satellite_used != other.num_satellite_used) {
      return false;
    }
    if (this->latitude_gnss != other.latitude_gnss) {
      return false;
    }
    if (this->longitude_gnss != other.longitude_gnss) {
      return false;
    }
    if (this->height_gnss != other.height_gnss) {
      return false;
    }
    return true;
  }
  bool operator!=(const BestGnss_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BestGnss_

// alias to use template instance with default allocator
using BestGnss =
  sensor::msg::BestGnss_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__BEST_GNSS__STRUCT_HPP_
