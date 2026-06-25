// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ars548_interface:msg/ObjectList.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__STRUCT_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__STRUCT_HPP_

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
// Member 'objects'
#include "ars548_interface/msg/detail/object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ars548_interface__msg__ObjectList __attribute__((deprecated))
#else
# define DEPRECATED__ars548_interface__msg__ObjectList __declspec(deprecated)
#endif

namespace ars548_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObjectList_
{
  using Type = ObjectList_<ContainerAllocator>;

  explicit ObjectList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->object_list_num_of_objects = 0;
    }
  }

  explicit ObjectList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
      this->object_list_num_of_objects = 0;
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
  using _object_list_num_of_objects_type =
    uint8_t;
  _object_list_num_of_objects_type object_list_num_of_objects;
  using _objects_type =
    std::vector<ars548_interface::msg::Object_<ContainerAllocator>, typename ContainerAllocator::template rebind<ars548_interface::msg::Object_<ContainerAllocator>>::other>;
  _objects_type objects;

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
  Type & set__object_list_num_of_objects(
    const uint8_t & _arg)
  {
    this->object_list_num_of_objects = _arg;
    return *this;
  }
  Type & set__objects(
    const std::vector<ars548_interface::msg::Object_<ContainerAllocator>, typename ContainerAllocator::template rebind<ars548_interface::msg::Object_<ContainerAllocator>>::other> & _arg)
  {
    this->objects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ars548_interface::msg::ObjectList_<ContainerAllocator> *;
  using ConstRawPtr =
    const ars548_interface::msg::ObjectList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ars548_interface::msg::ObjectList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ars548_interface::msg::ObjectList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::ObjectList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::ObjectList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ars548_interface::msg::ObjectList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ars548_interface::msg::ObjectList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ars548_interface::msg::ObjectList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ars548_interface::msg::ObjectList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ars548_interface__msg__ObjectList
    std::shared_ptr<ars548_interface::msg::ObjectList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ars548_interface__msg__ObjectList
    std::shared_ptr<ars548_interface::msg::ObjectList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObjectList_ & other) const
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
    if (this->object_list_num_of_objects != other.object_list_num_of_objects) {
      return false;
    }
    if (this->objects != other.objects) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObjectList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObjectList_

// alias to use template instance with default allocator
using ObjectList =
  ars548_interface::msg::ObjectList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__STRUCT_HPP_
