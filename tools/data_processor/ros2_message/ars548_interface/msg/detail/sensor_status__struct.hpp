// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/SensorStatus.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__STRUCT_HPP_

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
# define DEPRECATED__ars548_interface__msg__SensorStatus __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__SensorStatus __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorStatus_
{
  using Type = SensorStatus_<ContainerAllocator>;

  explicit SensorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp_nanoseconds = 0ul;
      this->timestamp_seconds = 0ul;
      this->timestamp_sync_status = 0;
      this->sw_version_major = 0;
      this->sw_version_minor = 0;
      this->sw_version_patch = 0;
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
      this->configuration_counter = 0;
      this->status_longitudinal_velocity = 0;
      this->status_longitudinal_acceleration = 0;
      this->status_lateral_acceleration = 0;
      this->status_yaw_rate = 0;
      this->status_steering_angle = 0;
      this->status_driving_direction = 0;
      this->status_characteristic_speed = 0;
      this->status_radar_status = 0;
      this->status_voltage_status = 0;
      this->status_temperature_status = 0;
      this->status_blockage_status = 0;
    }
  }

  explicit SensorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp_nanoseconds = 0ul;
      this->timestamp_seconds = 0ul;
      this->timestamp_sync_status = 0;
      this->sw_version_major = 0;
      this->sw_version_minor = 0;
      this->sw_version_patch = 0;
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
      this->configuration_counter = 0;
      this->status_longitudinal_velocity = 0;
      this->status_longitudinal_acceleration = 0;
      this->status_lateral_acceleration = 0;
      this->status_yaw_rate = 0;
      this->status_steering_angle = 0;
      this->status_driving_direction = 0;
      this->status_characteristic_speed = 0;
      this->status_radar_status = 0;
      this->status_voltage_status = 0;
      this->status_temperature_status = 0;
      this->status_blockage_status = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _timestamp_nanoseconds_type =
    uint32_t;
  _timestamp_nanoseconds_type timestamp_nanoseconds;
  using _timestamp_seconds_type =
    uint32_t;
  _timestamp_seconds_type timestamp_seconds;
  using _timestamp_sync_status_type =
    uint8_t;
  _timestamp_sync_status_type timestamp_sync_status;
  using _sw_version_major_type =
    uint8_t;
  _sw_version_major_type sw_version_major;
  using _sw_version_minor_type =
    uint8_t;
  _sw_version_minor_type sw_version_minor;
  using _sw_version_patch_type =
    uint8_t;
  _sw_version_patch_type sw_version_patch;
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
  using _configuration_counter_type =
    uint8_t;
  _configuration_counter_type configuration_counter;
  using _status_longitudinal_velocity_type =
    uint8_t;
  _status_longitudinal_velocity_type status_longitudinal_velocity;
  using _status_longitudinal_acceleration_type =
    uint8_t;
  _status_longitudinal_acceleration_type status_longitudinal_acceleration;
  using _status_lateral_acceleration_type =
    uint8_t;
  _status_lateral_acceleration_type status_lateral_acceleration;
  using _status_yaw_rate_type =
    uint8_t;
  _status_yaw_rate_type status_yaw_rate;
  using _status_steering_angle_type =
    uint8_t;
  _status_steering_angle_type status_steering_angle;
  using _status_driving_direction_type =
    uint8_t;
  _status_driving_direction_type status_driving_direction;
  using _status_characteristic_speed_type =
    uint8_t;
  _status_characteristic_speed_type status_characteristic_speed;
  using _status_radar_status_type =
    uint8_t;
  _status_radar_status_type status_radar_status;
  using _status_voltage_status_type =
    uint8_t;
  _status_voltage_status_type status_voltage_status;
  using _status_temperature_status_type =
    uint8_t;
  _status_temperature_status_type status_temperature_status;
  using _status_blockage_status_type =
    uint8_t;
  _status_blockage_status_type status_blockage_status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__timestamp_nanoseconds(
    const uint32_t & _arg)
  {
    this->timestamp_nanoseconds = _arg;
    return *this;
  }
  Type & set__timestamp_seconds(
    const uint32_t & _arg)
  {
    this->timestamp_seconds = _arg;
    return *this;
  }
  Type & set__timestamp_sync_status(
    const uint8_t & _arg)
  {
    this->timestamp_sync_status = _arg;
    return *this;
  }
  Type & set__sw_version_major(
    const uint8_t & _arg)
  {
    this->sw_version_major = _arg;
    return *this;
  }
  Type & set__sw_version_minor(
    const uint8_t & _arg)
  {
    this->sw_version_minor = _arg;
    return *this;
  }
  Type & set__sw_version_patch(
    const uint8_t & _arg)
  {
    this->sw_version_patch = _arg;
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
  Type & set__configuration_counter(
    const uint8_t & _arg)
  {
    this->configuration_counter = _arg;
    return *this;
  }
  Type & set__status_longitudinal_velocity(
    const uint8_t & _arg)
  {
    this->status_longitudinal_velocity = _arg;
    return *this;
  }
  Type & set__status_longitudinal_acceleration(
    const uint8_t & _arg)
  {
    this->status_longitudinal_acceleration = _arg;
    return *this;
  }
  Type & set__status_lateral_acceleration(
    const uint8_t & _arg)
  {
    this->status_lateral_acceleration = _arg;
    return *this;
  }
  Type & set__status_yaw_rate(
    const uint8_t & _arg)
  {
    this->status_yaw_rate = _arg;
    return *this;
  }
  Type & set__status_steering_angle(
    const uint8_t & _arg)
  {
    this->status_steering_angle = _arg;
    return *this;
  }
  Type & set__status_driving_direction(
    const uint8_t & _arg)
  {
    this->status_driving_direction = _arg;
    return *this;
  }
  Type & set__status_characteristic_speed(
    const uint8_t & _arg)
  {
    this->status_characteristic_speed = _arg;
    return *this;
  }
  Type & set__status_radar_status(
    const uint8_t & _arg)
  {
    this->status_radar_status = _arg;
    return *this;
  }
  Type & set__status_voltage_status(
    const uint8_t & _arg)
  {
    this->status_voltage_status = _arg;
    return *this;
  }
  Type & set__status_temperature_status(
    const uint8_t & _arg)
  {
    this->status_temperature_status = _arg;
    return *this;
  }
  Type & set__status_blockage_status(
    const uint8_t & _arg)
  {
    this->status_blockage_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::SensorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::SensorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::SensorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::SensorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::SensorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::SensorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::SensorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::SensorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::SensorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::SensorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__SensorStatus
    std::shared_ptr<ars548_interface::msg::SensorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__SensorStatus
    std::shared_ptr<ars548_interface::msg::SensorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->timestamp_nanoseconds != other.timestamp_nanoseconds) {
      return false;
    }
    if (this->timestamp_seconds != other.timestamp_seconds) {
      return false;
    }
    if (this->timestamp_sync_status != other.timestamp_sync_status) {
      return false;
    }
    if (this->sw_version_major != other.sw_version_major) {
      return false;
    }
    if (this->sw_version_minor != other.sw_version_minor) {
      return false;
    }
    if (this->sw_version_patch != other.sw_version_patch) {
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
    if (this->configuration_counter != other.configuration_counter) {
      return false;
    }
    if (this->status_longitudinal_velocity != other.status_longitudinal_velocity) {
      return false;
    }
    if (this->status_longitudinal_acceleration != other.status_longitudinal_acceleration) {
      return false;
    }
    if (this->status_lateral_acceleration != other.status_lateral_acceleration) {
      return false;
    }
    if (this->status_yaw_rate != other.status_yaw_rate) {
      return false;
    }
    if (this->status_steering_angle != other.status_steering_angle) {
      return false;
    }
    if (this->status_driving_direction != other.status_driving_direction) {
      return false;
    }
    if (this->status_characteristic_speed != other.status_characteristic_speed) {
      return false;
    }
    if (this->status_radar_status != other.status_radar_status) {
      return false;
    }
    if (this->status_voltage_status != other.status_voltage_status) {
      return false;
    }
    if (this->status_temperature_status != other.status_temperature_status) {
      return false;
    }
    if (this->status_blockage_status != other.status_blockage_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorStatus_

// alias to use template instance with default allocator
using SensorStatus =
  ars548_interface::msg::SensorStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__SENSOR_STATUS__STRUCT_HPP_
