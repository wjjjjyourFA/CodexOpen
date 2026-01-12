// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/DetectionList.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__STRUCT_HPP_

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
// Member 'detections'
#include "ars548_interface/msg/detail/detection__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ars548_interface__msg__DetectionList __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__DetectionList __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectionList_
{
  using Type = DetectionList_<ContainerAllocator>;

  explicit DetectionList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->service_id = 0;
      this->method_id = 0;
      this->data_length = 0ul;
      this->client_id = 0;
      this->session_id = 0;
      this->protocol_version = 0;
      this->interface_version = 0;
      this->message_type = 0;
      this->return_code = 0;
      this->crc = 0ull;
      this->length = 0ul;
      this->sqc = 0ul;
      this->data_id = 0ul;
      this->timestamp_nanoseconds = 0ul;
      this->timestamp_seconds = 0ul;
      this->timestamp_sync_status = 0;
      this->event_data_qualifier = 0ul;
      this->extended_qualifier = 0;
      this->origin_invalid_flags = 0;
      this->origin_pos_x = 0.0f;
      this->origin_pos_y = 0.0f;
      this->origin_pos_z = 0.0f;
      this->origin_x_std = 0.0f;
      this->origin_y_std = 0.0f;
      this->origin_z_std = 0.0f;
      this->origin_roll = 0.0f;
      this->origin_pitch = 0.0f;
      this->origin_yaw = 0.0f;
      this->origin_roll_std = 0.0f;
      this->origin_pitch_std = 0.0f;
      this->origin_yaw_std = 0.0f;
      this->list_invalid_flags = 0;
      this->list_rad_vel_domain_min = 0.0f;
      this->list_rad_vel_domain_max = 0.0f;
      this->list_num_of_detections = 0ul;
      this->aln_azimuth_correction = 0.0f;
      this->aln_elevation_correction = 0.0f;
      this->aln_status = 0;
    }
  }

  explicit DetectionList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->service_id = 0;
      this->method_id = 0;
      this->data_length = 0ul;
      this->client_id = 0;
      this->session_id = 0;
      this->protocol_version = 0;
      this->interface_version = 0;
      this->message_type = 0;
      this->return_code = 0;
      this->crc = 0ull;
      this->length = 0ul;
      this->sqc = 0ul;
      this->data_id = 0ul;
      this->timestamp_nanoseconds = 0ul;
      this->timestamp_seconds = 0ul;
      this->timestamp_sync_status = 0;
      this->event_data_qualifier = 0ul;
      this->extended_qualifier = 0;
      this->origin_invalid_flags = 0;
      this->origin_pos_x = 0.0f;
      this->origin_pos_y = 0.0f;
      this->origin_pos_z = 0.0f;
      this->origin_x_std = 0.0f;
      this->origin_y_std = 0.0f;
      this->origin_z_std = 0.0f;
      this->origin_roll = 0.0f;
      this->origin_pitch = 0.0f;
      this->origin_yaw = 0.0f;
      this->origin_roll_std = 0.0f;
      this->origin_pitch_std = 0.0f;
      this->origin_yaw_std = 0.0f;
      this->list_invalid_flags = 0;
      this->list_rad_vel_domain_min = 0.0f;
      this->list_rad_vel_domain_max = 0.0f;
      this->list_num_of_detections = 0ul;
      this->aln_azimuth_correction = 0.0f;
      this->aln_elevation_correction = 0.0f;
      this->aln_status = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _service_id_type =
    uint16_t;
  _service_id_type service_id;
  using _method_id_type =
    uint16_t;
  _method_id_type method_id;
  using _data_length_type =
    uint32_t;
  _data_length_type data_length;
  using _client_id_type =
    uint16_t;
  _client_id_type client_id;
  using _session_id_type =
    uint16_t;
  _session_id_type session_id;
  using _protocol_version_type =
    uint8_t;
  _protocol_version_type protocol_version;
  using _interface_version_type =
    uint8_t;
  _interface_version_type interface_version;
  using _message_type_type =
    uint8_t;
  _message_type_type message_type;
  using _return_code_type =
    uint8_t;
  _return_code_type return_code;
  using _crc_type =
    uint64_t;
  _crc_type crc;
  using _length_type =
    uint32_t;
  _length_type length;
  using _sqc_type =
    uint32_t;
  _sqc_type sqc;
  using _data_id_type =
    uint32_t;
  _data_id_type data_id;
  using _timestamp_nanoseconds_type =
    uint32_t;
  _timestamp_nanoseconds_type timestamp_nanoseconds;
  using _timestamp_seconds_type =
    uint32_t;
  _timestamp_seconds_type timestamp_seconds;
  using _timestamp_sync_status_type =
    uint8_t;
  _timestamp_sync_status_type timestamp_sync_status;
  using _event_data_qualifier_type =
    uint32_t;
  _event_data_qualifier_type event_data_qualifier;
  using _extended_qualifier_type =
    uint8_t;
  _extended_qualifier_type extended_qualifier;
  using _origin_invalid_flags_type =
    uint16_t;
  _origin_invalid_flags_type origin_invalid_flags;
  using _origin_pos_x_type =
    float;
  _origin_pos_x_type origin_pos_x;
  using _origin_pos_y_type =
    float;
  _origin_pos_y_type origin_pos_y;
  using _origin_pos_z_type =
    float;
  _origin_pos_z_type origin_pos_z;
  using _origin_x_std_type =
    float;
  _origin_x_std_type origin_x_std;
  using _origin_y_std_type =
    float;
  _origin_y_std_type origin_y_std;
  using _origin_z_std_type =
    float;
  _origin_z_std_type origin_z_std;
  using _origin_roll_type =
    float;
  _origin_roll_type origin_roll;
  using _origin_pitch_type =
    float;
  _origin_pitch_type origin_pitch;
  using _origin_yaw_type =
    float;
  _origin_yaw_type origin_yaw;
  using _origin_roll_std_type =
    float;
  _origin_roll_std_type origin_roll_std;
  using _origin_pitch_std_type =
    float;
  _origin_pitch_std_type origin_pitch_std;
  using _origin_yaw_std_type =
    float;
  _origin_yaw_std_type origin_yaw_std;
  using _list_invalid_flags_type =
    uint8_t;
  _list_invalid_flags_type list_invalid_flags;
  using _detections_type =
    std::vector<ars548_interface::msg::Detection_<ContainerAllocator>, typename ContainerAllocator::template rebind<ars548_interface::msg::Detection_<ContainerAllocator>>::other>;
  _detections_type detections;
  using _list_rad_vel_domain_min_type =
    float;
  _list_rad_vel_domain_min_type list_rad_vel_domain_min;
  using _list_rad_vel_domain_max_type =
    float;
  _list_rad_vel_domain_max_type list_rad_vel_domain_max;
  using _list_num_of_detections_type =
    uint32_t;
  _list_num_of_detections_type list_num_of_detections;
  using _aln_azimuth_correction_type =
    float;
  _aln_azimuth_correction_type aln_azimuth_correction;
  using _aln_elevation_correction_type =
    float;
  _aln_elevation_correction_type aln_elevation_correction;
  using _aln_status_type =
    uint8_t;
  _aln_status_type aln_status;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__service_id(
    const uint16_t & _arg)
  {
    this->service_id = _arg;
    return *this;
  }
  Type & set__method_id(
    const uint16_t & _arg)
  {
    this->method_id = _arg;
    return *this;
  }
  Type & set__data_length(
    const uint32_t & _arg)
  {
    this->data_length = _arg;
    return *this;
  }
  Type & set__client_id(
    const uint16_t & _arg)
  {
    this->client_id = _arg;
    return *this;
  }
  Type & set__session_id(
    const uint16_t & _arg)
  {
    this->session_id = _arg;
    return *this;
  }
  Type & set__protocol_version(
    const uint8_t & _arg)
  {
    this->protocol_version = _arg;
    return *this;
  }
  Type & set__interface_version(
    const uint8_t & _arg)
  {
    this->interface_version = _arg;
    return *this;
  }
  Type & set__message_type(
    const uint8_t & _arg)
  {
    this->message_type = _arg;
    return *this;
  }
  Type & set__return_code(
    const uint8_t & _arg)
  {
    this->return_code = _arg;
    return *this;
  }
  Type & set__crc(
    const uint64_t & _arg)
  {
    this->crc = _arg;
    return *this;
  }
  Type & set__length(
    const uint32_t & _arg)
  {
    this->length = _arg;
    return *this;
  }
  Type & set__sqc(
    const uint32_t & _arg)
  {
    this->sqc = _arg;
    return *this;
  }
  Type & set__data_id(
    const uint32_t & _arg)
  {
    this->data_id = _arg;
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
  Type & set__event_data_qualifier(
    const uint32_t & _arg)
  {
    this->event_data_qualifier = _arg;
    return *this;
  }
  Type & set__extended_qualifier(
    const uint8_t & _arg)
  {
    this->extended_qualifier = _arg;
    return *this;
  }
  Type & set__origin_invalid_flags(
    const uint16_t & _arg)
  {
    this->origin_invalid_flags = _arg;
    return *this;
  }
  Type & set__origin_pos_x(
    const float & _arg)
  {
    this->origin_pos_x = _arg;
    return *this;
  }
  Type & set__origin_pos_y(
    const float & _arg)
  {
    this->origin_pos_y = _arg;
    return *this;
  }
  Type & set__origin_pos_z(
    const float & _arg)
  {
    this->origin_pos_z = _arg;
    return *this;
  }
  Type & set__origin_x_std(
    const float & _arg)
  {
    this->origin_x_std = _arg;
    return *this;
  }
  Type & set__origin_y_std(
    const float & _arg)
  {
    this->origin_y_std = _arg;
    return *this;
  }
  Type & set__origin_z_std(
    const float & _arg)
  {
    this->origin_z_std = _arg;
    return *this;
  }
  Type & set__origin_roll(
    const float & _arg)
  {
    this->origin_roll = _arg;
    return *this;
  }
  Type & set__origin_pitch(
    const float & _arg)
  {
    this->origin_pitch = _arg;
    return *this;
  }
  Type & set__origin_yaw(
    const float & _arg)
  {
    this->origin_yaw = _arg;
    return *this;
  }
  Type & set__origin_roll_std(
    const float & _arg)
  {
    this->origin_roll_std = _arg;
    return *this;
  }
  Type & set__origin_pitch_std(
    const float & _arg)
  {
    this->origin_pitch_std = _arg;
    return *this;
  }
  Type & set__origin_yaw_std(
    const float & _arg)
  {
    this->origin_yaw_std = _arg;
    return *this;
  }
  Type & set__list_invalid_flags(
    const uint8_t & _arg)
  {
    this->list_invalid_flags = _arg;
    return *this;
  }
  Type & set__detections(
    const std::vector<ars548_interface::msg::Detection_<ContainerAllocator>, typename ContainerAllocator::template rebind<ars548_interface::msg::Detection_<ContainerAllocator>>::other> & _arg)
  {
    this->detections = _arg;
    return *this;
  }
  Type & set__list_rad_vel_domain_min(
    const float & _arg)
  {
    this->list_rad_vel_domain_min = _arg;
    return *this;
  }
  Type & set__list_rad_vel_domain_max(
    const float & _arg)
  {
    this->list_rad_vel_domain_max = _arg;
    return *this;
  }
  Type & set__list_num_of_detections(
    const uint32_t & _arg)
  {
    this->list_num_of_detections = _arg;
    return *this;
  }
  Type & set__aln_azimuth_correction(
    const float & _arg)
  {
    this->aln_azimuth_correction = _arg;
    return *this;
  }
  Type & set__aln_elevation_correction(
    const float & _arg)
  {
    this->aln_elevation_correction = _arg;
    return *this;
  }
  Type & set__aln_status(
    const uint8_t & _arg)
  {
    this->aln_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::DetectionList_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::DetectionList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::DetectionList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::DetectionList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::DetectionList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::DetectionList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::DetectionList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::DetectionList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::DetectionList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::DetectionList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__DetectionList
    std::shared_ptr<ars548_interface::msg::DetectionList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__DetectionList
    std::shared_ptr<ars548_interface::msg::DetectionList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectionList_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->service_id != other.service_id) {
      return false;
    }
    if (this->method_id != other.method_id) {
      return false;
    }
    if (this->data_length != other.data_length) {
      return false;
    }
    if (this->client_id != other.client_id) {
      return false;
    }
    if (this->session_id != other.session_id) {
      return false;
    }
    if (this->protocol_version != other.protocol_version) {
      return false;
    }
    if (this->interface_version != other.interface_version) {
      return false;
    }
    if (this->message_type != other.message_type) {
      return false;
    }
    if (this->return_code != other.return_code) {
      return false;
    }
    if (this->crc != other.crc) {
      return false;
    }
    if (this->length != other.length) {
      return false;
    }
    if (this->sqc != other.sqc) {
      return false;
    }
    if (this->data_id != other.data_id) {
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
    if (this->event_data_qualifier != other.event_data_qualifier) {
      return false;
    }
    if (this->extended_qualifier != other.extended_qualifier) {
      return false;
    }
    if (this->origin_invalid_flags != other.origin_invalid_flags) {
      return false;
    }
    if (this->origin_pos_x != other.origin_pos_x) {
      return false;
    }
    if (this->origin_pos_y != other.origin_pos_y) {
      return false;
    }
    if (this->origin_pos_z != other.origin_pos_z) {
      return false;
    }
    if (this->origin_x_std != other.origin_x_std) {
      return false;
    }
    if (this->origin_y_std != other.origin_y_std) {
      return false;
    }
    if (this->origin_z_std != other.origin_z_std) {
      return false;
    }
    if (this->origin_roll != other.origin_roll) {
      return false;
    }
    if (this->origin_pitch != other.origin_pitch) {
      return false;
    }
    if (this->origin_yaw != other.origin_yaw) {
      return false;
    }
    if (this->origin_roll_std != other.origin_roll_std) {
      return false;
    }
    if (this->origin_pitch_std != other.origin_pitch_std) {
      return false;
    }
    if (this->origin_yaw_std != other.origin_yaw_std) {
      return false;
    }
    if (this->list_invalid_flags != other.list_invalid_flags) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    if (this->list_rad_vel_domain_min != other.list_rad_vel_domain_min) {
      return false;
    }
    if (this->list_rad_vel_domain_max != other.list_rad_vel_domain_max) {
      return false;
    }
    if (this->list_num_of_detections != other.list_num_of_detections) {
      return false;
    }
    if (this->aln_azimuth_correction != other.aln_azimuth_correction) {
      return false;
    }
    if (this->aln_elevation_correction != other.aln_elevation_correction) {
      return false;
    }
    if (this->aln_status != other.aln_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectionList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectionList_

// alias to use template instance with default allocator
using DetectionList =
  ars548_interface::msg::DetectionList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__STRUCT_HPP_
