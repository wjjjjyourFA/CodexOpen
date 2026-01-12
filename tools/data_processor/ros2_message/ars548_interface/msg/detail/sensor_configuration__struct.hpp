// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/SensorConfiguration.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__STRUCT_HPP_

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
# define DEPRECATED__ars548_interface__msg__SensorConfiguration __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__SensorConfiguration __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorConfiguration_
{
  using Type = SensorConfiguration_<ContainerAllocator>;

  explicit SensorConfiguration_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->longitudinal = 0.0f;
      this->lateral = 0.0f;
      this->vertical = 0.0f;
      this->yaw = 0.0f;
      this->pitch = 0.0f;
      this->plug_orientation = 0;
      this->length = 0.0f;
      this->width = 0.0f;
      this->height = 0.0f;
      this->wheelbase = 0.0f;
      this->maximum_distance = 0;
      this->frequency_slot = 0;
      this->cycle_time = 0;
      this->time_slot = 0;
      this->hcc = 0;
      this->powersave_standstill = 0;
      this->sensor_ip_address_0 = 0ul;
      this->sensor_ip_address_1 = 0ul;
      this->new_sensor_mounting = 0;
      this->new_vehicle_parameters = 0;
      this->new_radar_parameters = 0;
      this->new_network_configuration = 0;
    }
  }

  explicit SensorConfiguration_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->longitudinal = 0.0f;
      this->lateral = 0.0f;
      this->vertical = 0.0f;
      this->yaw = 0.0f;
      this->pitch = 0.0f;
      this->plug_orientation = 0;
      this->length = 0.0f;
      this->width = 0.0f;
      this->height = 0.0f;
      this->wheelbase = 0.0f;
      this->maximum_distance = 0;
      this->frequency_slot = 0;
      this->cycle_time = 0;
      this->time_slot = 0;
      this->hcc = 0;
      this->powersave_standstill = 0;
      this->sensor_ip_address_0 = 0ul;
      this->sensor_ip_address_1 = 0ul;
      this->new_sensor_mounting = 0;
      this->new_vehicle_parameters = 0;
      this->new_radar_parameters = 0;
      this->new_network_configuration = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _longitudinal_type =
    float;
  _longitudinal_type longitudinal;
  using _lateral_type =
    float;
  _lateral_type lateral;
  using _vertical_type =
    float;
  _vertical_type vertical;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _plug_orientation_type =
    uint8_t;
  _plug_orientation_type plug_orientation;
  using _length_type =
    float;
  _length_type length;
  using _width_type =
    float;
  _width_type width;
  using _height_type =
    float;
  _height_type height;
  using _wheelbase_type =
    float;
  _wheelbase_type wheelbase;
  using _maximum_distance_type =
    uint16_t;
  _maximum_distance_type maximum_distance;
  using _frequency_slot_type =
    uint8_t;
  _frequency_slot_type frequency_slot;
  using _cycle_time_type =
    uint8_t;
  _cycle_time_type cycle_time;
  using _time_slot_type =
    uint8_t;
  _time_slot_type time_slot;
  using _hcc_type =
    uint8_t;
  _hcc_type hcc;
  using _powersave_standstill_type =
    uint8_t;
  _powersave_standstill_type powersave_standstill;
  using _sensor_ip_address_0_type =
    uint32_t;
  _sensor_ip_address_0_type sensor_ip_address_0;
  using _sensor_ip_address_1_type =
    uint32_t;
  _sensor_ip_address_1_type sensor_ip_address_1;
  using _new_sensor_mounting_type =
    uint8_t;
  _new_sensor_mounting_type new_sensor_mounting;
  using _new_vehicle_parameters_type =
    uint8_t;
  _new_vehicle_parameters_type new_vehicle_parameters;
  using _new_radar_parameters_type =
    uint8_t;
  _new_radar_parameters_type new_radar_parameters;
  using _new_network_configuration_type =
    uint8_t;
  _new_network_configuration_type new_network_configuration;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__longitudinal(
    const float & _arg)
  {
    this->longitudinal = _arg;
    return *this;
  }
  Type & set__lateral(
    const float & _arg)
  {
    this->lateral = _arg;
    return *this;
  }
  Type & set__vertical(
    const float & _arg)
  {
    this->vertical = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__plug_orientation(
    const uint8_t & _arg)
  {
    this->plug_orientation = _arg;
    return *this;
  }
  Type & set__length(
    const float & _arg)
  {
    this->length = _arg;
    return *this;
  }
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__wheelbase(
    const float & _arg)
  {
    this->wheelbase = _arg;
    return *this;
  }
  Type & set__maximum_distance(
    const uint16_t & _arg)
  {
    this->maximum_distance = _arg;
    return *this;
  }
  Type & set__frequency_slot(
    const uint8_t & _arg)
  {
    this->frequency_slot = _arg;
    return *this;
  }
  Type & set__cycle_time(
    const uint8_t & _arg)
  {
    this->cycle_time = _arg;
    return *this;
  }
  Type & set__time_slot(
    const uint8_t & _arg)
  {
    this->time_slot = _arg;
    return *this;
  }
  Type & set__hcc(
    const uint8_t & _arg)
  {
    this->hcc = _arg;
    return *this;
  }
  Type & set__powersave_standstill(
    const uint8_t & _arg)
  {
    this->powersave_standstill = _arg;
    return *this;
  }
  Type & set__sensor_ip_address_0(
    const uint32_t & _arg)
  {
    this->sensor_ip_address_0 = _arg;
    return *this;
  }
  Type & set__sensor_ip_address_1(
    const uint32_t & _arg)
  {
    this->sensor_ip_address_1 = _arg;
    return *this;
  }
  Type & set__new_sensor_mounting(
    const uint8_t & _arg)
  {
    this->new_sensor_mounting = _arg;
    return *this;
  }
  Type & set__new_vehicle_parameters(
    const uint8_t & _arg)
  {
    this->new_vehicle_parameters = _arg;
    return *this;
  }
  Type & set__new_radar_parameters(
    const uint8_t & _arg)
  {
    this->new_radar_parameters = _arg;
    return *this;
  }
  Type & set__new_network_configuration(
    const uint8_t & _arg)
  {
    this->new_network_configuration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::SensorConfiguration_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::SensorConfiguration_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::SensorConfiguration_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::SensorConfiguration_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::SensorConfiguration_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::SensorConfiguration_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::SensorConfiguration_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::SensorConfiguration_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::SensorConfiguration_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::SensorConfiguration_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__SensorConfiguration
    std::shared_ptr<ars548_interface::msg::SensorConfiguration_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__SensorConfiguration
    std::shared_ptr<ars548_interface::msg::SensorConfiguration_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorConfiguration_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->longitudinal != other.longitudinal) {
      return false;
    }
    if (this->lateral != other.lateral) {
      return false;
    }
    if (this->vertical != other.vertical) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->plug_orientation != other.plug_orientation) {
      return false;
    }
    if (this->length != other.length) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->wheelbase != other.wheelbase) {
      return false;
    }
    if (this->maximum_distance != other.maximum_distance) {
      return false;
    }
    if (this->frequency_slot != other.frequency_slot) {
      return false;
    }
    if (this->cycle_time != other.cycle_time) {
      return false;
    }
    if (this->time_slot != other.time_slot) {
      return false;
    }
    if (this->hcc != other.hcc) {
      return false;
    }
    if (this->powersave_standstill != other.powersave_standstill) {
      return false;
    }
    if (this->sensor_ip_address_0 != other.sensor_ip_address_0) {
      return false;
    }
    if (this->sensor_ip_address_1 != other.sensor_ip_address_1) {
      return false;
    }
    if (this->new_sensor_mounting != other.new_sensor_mounting) {
      return false;
    }
    if (this->new_vehicle_parameters != other.new_vehicle_parameters) {
      return false;
    }
    if (this->new_radar_parameters != other.new_radar_parameters) {
      return false;
    }
    if (this->new_network_configuration != other.new_network_configuration) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorConfiguration_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorConfiguration_

// alias to use template instance with default allocator
using SensorConfiguration =
  ars548_interface::msg::SensorConfiguration_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__SENSOR_CONFIGURATION__STRUCT_HPP_
