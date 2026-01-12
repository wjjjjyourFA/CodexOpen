// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/ObjectList.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__BUILDER_HPP_

#include "ars548_interface/msg/detail/object_list__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_ObjectList_objects
{
public:
  explicit Init_ObjectList_objects(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::ObjectList objects(::ars548_interface::msg::ObjectList::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_object_list_num_of_objects
{
public:
  explicit Init_ObjectList_object_list_num_of_objects(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_objects object_list_num_of_objects(::ars548_interface::msg::ObjectList::_object_list_num_of_objects_type arg)
  {
    msg_.object_list_num_of_objects = std::move(arg);
    return Init_ObjectList_objects(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_extended_qualifier
{
public:
  explicit Init_ObjectList_extended_qualifier(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_object_list_num_of_objects extended_qualifier(::ars548_interface::msg::ObjectList::_extended_qualifier_type arg)
  {
    msg_.extended_qualifier = std::move(arg);
    return Init_ObjectList_object_list_num_of_objects(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_event_data_qualifier
{
public:
  explicit Init_ObjectList_event_data_qualifier(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_extended_qualifier event_data_qualifier(::ars548_interface::msg::ObjectList::_event_data_qualifier_type arg)
  {
    msg_.event_data_qualifier = std::move(arg);
    return Init_ObjectList_extended_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_timestamp_sync_status
{
public:
  explicit Init_ObjectList_timestamp_sync_status(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_event_data_qualifier timestamp_sync_status(::ars548_interface::msg::ObjectList::_timestamp_sync_status_type arg)
  {
    msg_.timestamp_sync_status = std::move(arg);
    return Init_ObjectList_event_data_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_timestamp_seconds
{
public:
  explicit Init_ObjectList_timestamp_seconds(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_timestamp_sync_status timestamp_seconds(::ars548_interface::msg::ObjectList::_timestamp_seconds_type arg)
  {
    msg_.timestamp_seconds = std::move(arg);
    return Init_ObjectList_timestamp_sync_status(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_timestamp_nanoseconds
{
public:
  explicit Init_ObjectList_timestamp_nanoseconds(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_timestamp_seconds timestamp_nanoseconds(::ars548_interface::msg::ObjectList::_timestamp_nanoseconds_type arg)
  {
    msg_.timestamp_nanoseconds = std::move(arg);
    return Init_ObjectList_timestamp_seconds(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_data_id
{
public:
  explicit Init_ObjectList_data_id(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_timestamp_nanoseconds data_id(::ars548_interface::msg::ObjectList::_data_id_type arg)
  {
    msg_.data_id = std::move(arg);
    return Init_ObjectList_timestamp_nanoseconds(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_sqc
{
public:
  explicit Init_ObjectList_sqc(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_data_id sqc(::ars548_interface::msg::ObjectList::_sqc_type arg)
  {
    msg_.sqc = std::move(arg);
    return Init_ObjectList_data_id(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_length
{
public:
  explicit Init_ObjectList_length(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_sqc length(::ars548_interface::msg::ObjectList::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_ObjectList_sqc(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_crc
{
public:
  explicit Init_ObjectList_crc(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_length crc(::ars548_interface::msg::ObjectList::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return Init_ObjectList_length(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_return_code
{
public:
  explicit Init_ObjectList_return_code(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_crc return_code(::ars548_interface::msg::ObjectList::_return_code_type arg)
  {
    msg_.return_code = std::move(arg);
    return Init_ObjectList_crc(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_message_type
{
public:
  explicit Init_ObjectList_message_type(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_return_code message_type(::ars548_interface::msg::ObjectList::_message_type_type arg)
  {
    msg_.message_type = std::move(arg);
    return Init_ObjectList_return_code(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_interface_version
{
public:
  explicit Init_ObjectList_interface_version(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_message_type interface_version(::ars548_interface::msg::ObjectList::_interface_version_type arg)
  {
    msg_.interface_version = std::move(arg);
    return Init_ObjectList_message_type(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_protocol_version
{
public:
  explicit Init_ObjectList_protocol_version(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_interface_version protocol_version(::ars548_interface::msg::ObjectList::_protocol_version_type arg)
  {
    msg_.protocol_version = std::move(arg);
    return Init_ObjectList_interface_version(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_session_id
{
public:
  explicit Init_ObjectList_session_id(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_protocol_version session_id(::ars548_interface::msg::ObjectList::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_ObjectList_protocol_version(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_client_id
{
public:
  explicit Init_ObjectList_client_id(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_session_id client_id(::ars548_interface::msg::ObjectList::_client_id_type arg)
  {
    msg_.client_id = std::move(arg);
    return Init_ObjectList_session_id(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_data_length
{
public:
  explicit Init_ObjectList_data_length(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_client_id data_length(::ars548_interface::msg::ObjectList::_data_length_type arg)
  {
    msg_.data_length = std::move(arg);
    return Init_ObjectList_client_id(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_method_id
{
public:
  explicit Init_ObjectList_method_id(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_data_length method_id(::ars548_interface::msg::ObjectList::_method_id_type arg)
  {
    msg_.method_id = std::move(arg);
    return Init_ObjectList_data_length(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_service_id
{
public:
  explicit Init_ObjectList_service_id(::ars548_interface::msg::ObjectList & msg)
  : msg_(msg)
  {}
  Init_ObjectList_method_id service_id(::ars548_interface::msg::ObjectList::_service_id_type arg)
  {
    msg_.service_id = std::move(arg);
    return Init_ObjectList_method_id(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

class Init_ObjectList_header
{
public:
  Init_ObjectList_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectList_service_id header(::ars548_interface::msg::ObjectList::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectList_service_id(msg_);
  }

private:
  ::ars548_interface::msg::ObjectList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::ObjectList>()
{
  return ars548_interface::msg::builder::Init_ObjectList_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__BUILDER_HPP_
