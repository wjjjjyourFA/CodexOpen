// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DETECTION__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__DETECTION__STRUCT_HPP_

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
# define DEPRECATED__ars548_interface__msg__Detection __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__Detection __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection_
{
  using Type = Detection_<ContainerAllocator>;

  explicit Detection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->f_x = 0.0f;
      this->f_y = 0.0f;
      this->f_z = 0.0f;
      this->f_azimuth_angle = 0.0f;
      this->f_azimuth_angle_std = 0.0f;
      this->u_invalid_flags = 0;
      this->f_elevation_angle = 0.0f;
      this->f_elevation_angle_std = 0.0f;
      this->f_range = 0.0f;
      this->f_range_std = 0.0f;
      this->f_range_rate = 0.0f;
      this->f_range_rate_std = 0.0f;
      this->s_rcs = 0;
      this->u_measurement_id = 0;
      this->u_positive_predictive_value = 0;
      this->u_classification = 0;
      this->u_multi_target_probability = 0;
      this->u_object_id = 0;
      this->u_ambiguity_flag = 0;
      this->u_sort_index = 0;
    }
  }

  explicit Detection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->f_x = 0.0f;
      this->f_y = 0.0f;
      this->f_z = 0.0f;
      this->f_azimuth_angle = 0.0f;
      this->f_azimuth_angle_std = 0.0f;
      this->u_invalid_flags = 0;
      this->f_elevation_angle = 0.0f;
      this->f_elevation_angle_std = 0.0f;
      this->f_range = 0.0f;
      this->f_range_std = 0.0f;
      this->f_range_rate = 0.0f;
      this->f_range_rate_std = 0.0f;
      this->s_rcs = 0;
      this->u_measurement_id = 0;
      this->u_positive_predictive_value = 0;
      this->u_classification = 0;
      this->u_multi_target_probability = 0;
      this->u_object_id = 0;
      this->u_ambiguity_flag = 0;
      this->u_sort_index = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _f_x_type =
    float;
  _f_x_type f_x;
  using _f_y_type =
    float;
  _f_y_type f_y;
  using _f_z_type =
    float;
  _f_z_type f_z;
  using _f_azimuth_angle_type =
    float;
  _f_azimuth_angle_type f_azimuth_angle;
  using _f_azimuth_angle_std_type =
    float;
  _f_azimuth_angle_std_type f_azimuth_angle_std;
  using _u_invalid_flags_type =
    uint8_t;
  _u_invalid_flags_type u_invalid_flags;
  using _f_elevation_angle_type =
    float;
  _f_elevation_angle_type f_elevation_angle;
  using _f_elevation_angle_std_type =
    float;
  _f_elevation_angle_std_type f_elevation_angle_std;
  using _f_range_type =
    float;
  _f_range_type f_range;
  using _f_range_std_type =
    float;
  _f_range_std_type f_range_std;
  using _f_range_rate_type =
    float;
  _f_range_rate_type f_range_rate;
  using _f_range_rate_std_type =
    float;
  _f_range_rate_std_type f_range_rate_std;
  using _s_rcs_type =
    int8_t;
  _s_rcs_type s_rcs;
  using _u_measurement_id_type =
    uint16_t;
  _u_measurement_id_type u_measurement_id;
  using _u_positive_predictive_value_type =
    uint8_t;
  _u_positive_predictive_value_type u_positive_predictive_value;
  using _u_classification_type =
    uint8_t;
  _u_classification_type u_classification;
  using _u_multi_target_probability_type =
    uint8_t;
  _u_multi_target_probability_type u_multi_target_probability;
  using _u_object_id_type =
    uint16_t;
  _u_object_id_type u_object_id;
  using _u_ambiguity_flag_type =
    uint8_t;
  _u_ambiguity_flag_type u_ambiguity_flag;
  using _u_sort_index_type =
    uint16_t;
  _u_sort_index_type u_sort_index;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__f_x(
    const float & _arg)
  {
    this->f_x = _arg;
    return *this;
  }
  Type & set__f_y(
    const float & _arg)
  {
    this->f_y = _arg;
    return *this;
  }
  Type & set__f_z(
    const float & _arg)
  {
    this->f_z = _arg;
    return *this;
  }
  Type & set__f_azimuth_angle(
    const float & _arg)
  {
    this->f_azimuth_angle = _arg;
    return *this;
  }
  Type & set__f_azimuth_angle_std(
    const float & _arg)
  {
    this->f_azimuth_angle_std = _arg;
    return *this;
  }
  Type & set__u_invalid_flags(
    const uint8_t & _arg)
  {
    this->u_invalid_flags = _arg;
    return *this;
  }
  Type & set__f_elevation_angle(
    const float & _arg)
  {
    this->f_elevation_angle = _arg;
    return *this;
  }
  Type & set__f_elevation_angle_std(
    const float & _arg)
  {
    this->f_elevation_angle_std = _arg;
    return *this;
  }
  Type & set__f_range(
    const float & _arg)
  {
    this->f_range = _arg;
    return *this;
  }
  Type & set__f_range_std(
    const float & _arg)
  {
    this->f_range_std = _arg;
    return *this;
  }
  Type & set__f_range_rate(
    const float & _arg)
  {
    this->f_range_rate = _arg;
    return *this;
  }
  Type & set__f_range_rate_std(
    const float & _arg)
  {
    this->f_range_rate_std = _arg;
    return *this;
  }
  Type & set__s_rcs(
    const int8_t & _arg)
  {
    this->s_rcs = _arg;
    return *this;
  }
  Type & set__u_measurement_id(
    const uint16_t & _arg)
  {
    this->u_measurement_id = _arg;
    return *this;
  }
  Type & set__u_positive_predictive_value(
    const uint8_t & _arg)
  {
    this->u_positive_predictive_value = _arg;
    return *this;
  }
  Type & set__u_classification(
    const uint8_t & _arg)
  {
    this->u_classification = _arg;
    return *this;
  }
  Type & set__u_multi_target_probability(
    const uint8_t & _arg)
  {
    this->u_multi_target_probability = _arg;
    return *this;
  }
  Type & set__u_object_id(
    const uint16_t & _arg)
  {
    this->u_object_id = _arg;
    return *this;
  }
  Type & set__u_ambiguity_flag(
    const uint8_t & _arg)
  {
    this->u_ambiguity_flag = _arg;
    return *this;
  }
  Type & set__u_sort_index(
    const uint16_t & _arg)
  {
    this->u_sort_index = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::Detection_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::Detection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::Detection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::Detection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::Detection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::Detection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::Detection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::Detection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::Detection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::Detection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__Detection
    std::shared_ptr<ars548_interface::msg::Detection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__Detection
    std::shared_ptr<ars548_interface::msg::Detection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->f_x != other.f_x) {
      return false;
    }
    if (this->f_y != other.f_y) {
      return false;
    }
    if (this->f_z != other.f_z) {
      return false;
    }
    if (this->f_azimuth_angle != other.f_azimuth_angle) {
      return false;
    }
    if (this->f_azimuth_angle_std != other.f_azimuth_angle_std) {
      return false;
    }
    if (this->u_invalid_flags != other.u_invalid_flags) {
      return false;
    }
    if (this->f_elevation_angle != other.f_elevation_angle) {
      return false;
    }
    if (this->f_elevation_angle_std != other.f_elevation_angle_std) {
      return false;
    }
    if (this->f_range != other.f_range) {
      return false;
    }
    if (this->f_range_std != other.f_range_std) {
      return false;
    }
    if (this->f_range_rate != other.f_range_rate) {
      return false;
    }
    if (this->f_range_rate_std != other.f_range_rate_std) {
      return false;
    }
    if (this->s_rcs != other.s_rcs) {
      return false;
    }
    if (this->u_measurement_id != other.u_measurement_id) {
      return false;
    }
    if (this->u_positive_predictive_value != other.u_positive_predictive_value) {
      return false;
    }
    if (this->u_classification != other.u_classification) {
      return false;
    }
    if (this->u_multi_target_probability != other.u_multi_target_probability) {
      return false;
    }
    if (this->u_object_id != other.u_object_id) {
      return false;
    }
    if (this->u_ambiguity_flag != other.u_ambiguity_flag) {
      return false;
    }
    if (this->u_sort_index != other.u_sort_index) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection_

// alias to use template instance with default allocator
using Detection =
  ars548_interface::msg::Detection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__DETECTION__STRUCT_HPP_
