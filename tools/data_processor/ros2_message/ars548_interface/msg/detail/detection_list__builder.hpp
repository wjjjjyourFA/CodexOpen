// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/DetectionList.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__BUILDER_HPP_

#include "ars548_interface/msg/detail/detection_list__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_DetectionList_aln_status
{
public:
  explicit Init_DetectionList_aln_status(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::DetectionList aln_status(::ars548_interface::msg::DetectionList::_aln_status_type arg)
  {
    msg_.aln_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_aln_elevation_correction
{
public:
  explicit Init_DetectionList_aln_elevation_correction(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_aln_status aln_elevation_correction(::ars548_interface::msg::DetectionList::_aln_elevation_correction_type arg)
  {
    msg_.aln_elevation_correction = std::move(arg);
    return Init_DetectionList_aln_status(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_aln_azimuth_correction
{
public:
  explicit Init_DetectionList_aln_azimuth_correction(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_aln_elevation_correction aln_azimuth_correction(::ars548_interface::msg::DetectionList::_aln_azimuth_correction_type arg)
  {
    msg_.aln_azimuth_correction = std::move(arg);
    return Init_DetectionList_aln_elevation_correction(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_list_num_of_detections
{
public:
  explicit Init_DetectionList_list_num_of_detections(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_aln_azimuth_correction list_num_of_detections(::ars548_interface::msg::DetectionList::_list_num_of_detections_type arg)
  {
    msg_.list_num_of_detections = std::move(arg);
    return Init_DetectionList_aln_azimuth_correction(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_list_rad_vel_domain_max
{
public:
  explicit Init_DetectionList_list_rad_vel_domain_max(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_list_num_of_detections list_rad_vel_domain_max(::ars548_interface::msg::DetectionList::_list_rad_vel_domain_max_type arg)
  {
    msg_.list_rad_vel_domain_max = std::move(arg);
    return Init_DetectionList_list_num_of_detections(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_list_rad_vel_domain_min
{
public:
  explicit Init_DetectionList_list_rad_vel_domain_min(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_list_rad_vel_domain_max list_rad_vel_domain_min(::ars548_interface::msg::DetectionList::_list_rad_vel_domain_min_type arg)
  {
    msg_.list_rad_vel_domain_min = std::move(arg);
    return Init_DetectionList_list_rad_vel_domain_max(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_detections
{
public:
  explicit Init_DetectionList_detections(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_list_rad_vel_domain_min detections(::ars548_interface::msg::DetectionList::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return Init_DetectionList_list_rad_vel_domain_min(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_list_invalid_flags
{
public:
  explicit Init_DetectionList_list_invalid_flags(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_detections list_invalid_flags(::ars548_interface::msg::DetectionList::_list_invalid_flags_type arg)
  {
    msg_.list_invalid_flags = std::move(arg);
    return Init_DetectionList_detections(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_yaw_std
{
public:
  explicit Init_DetectionList_origin_yaw_std(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_list_invalid_flags origin_yaw_std(::ars548_interface::msg::DetectionList::_origin_yaw_std_type arg)
  {
    msg_.origin_yaw_std = std::move(arg);
    return Init_DetectionList_list_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_pitch_std
{
public:
  explicit Init_DetectionList_origin_pitch_std(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_yaw_std origin_pitch_std(::ars548_interface::msg::DetectionList::_origin_pitch_std_type arg)
  {
    msg_.origin_pitch_std = std::move(arg);
    return Init_DetectionList_origin_yaw_std(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_roll_std
{
public:
  explicit Init_DetectionList_origin_roll_std(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_pitch_std origin_roll_std(::ars548_interface::msg::DetectionList::_origin_roll_std_type arg)
  {
    msg_.origin_roll_std = std::move(arg);
    return Init_DetectionList_origin_pitch_std(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_yaw
{
public:
  explicit Init_DetectionList_origin_yaw(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_roll_std origin_yaw(::ars548_interface::msg::DetectionList::_origin_yaw_type arg)
  {
    msg_.origin_yaw = std::move(arg);
    return Init_DetectionList_origin_roll_std(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_pitch
{
public:
  explicit Init_DetectionList_origin_pitch(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_yaw origin_pitch(::ars548_interface::msg::DetectionList::_origin_pitch_type arg)
  {
    msg_.origin_pitch = std::move(arg);
    return Init_DetectionList_origin_yaw(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_roll
{
public:
  explicit Init_DetectionList_origin_roll(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_pitch origin_roll(::ars548_interface::msg::DetectionList::_origin_roll_type arg)
  {
    msg_.origin_roll = std::move(arg);
    return Init_DetectionList_origin_pitch(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_z_std
{
public:
  explicit Init_DetectionList_origin_z_std(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_roll origin_z_std(::ars548_interface::msg::DetectionList::_origin_z_std_type arg)
  {
    msg_.origin_z_std = std::move(arg);
    return Init_DetectionList_origin_roll(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_y_std
{
public:
  explicit Init_DetectionList_origin_y_std(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_z_std origin_y_std(::ars548_interface::msg::DetectionList::_origin_y_std_type arg)
  {
    msg_.origin_y_std = std::move(arg);
    return Init_DetectionList_origin_z_std(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_x_std
{
public:
  explicit Init_DetectionList_origin_x_std(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_y_std origin_x_std(::ars548_interface::msg::DetectionList::_origin_x_std_type arg)
  {
    msg_.origin_x_std = std::move(arg);
    return Init_DetectionList_origin_y_std(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_pos_z
{
public:
  explicit Init_DetectionList_origin_pos_z(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_x_std origin_pos_z(::ars548_interface::msg::DetectionList::_origin_pos_z_type arg)
  {
    msg_.origin_pos_z = std::move(arg);
    return Init_DetectionList_origin_x_std(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_pos_y
{
public:
  explicit Init_DetectionList_origin_pos_y(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_pos_z origin_pos_y(::ars548_interface::msg::DetectionList::_origin_pos_y_type arg)
  {
    msg_.origin_pos_y = std::move(arg);
    return Init_DetectionList_origin_pos_z(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_pos_x
{
public:
  explicit Init_DetectionList_origin_pos_x(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_pos_y origin_pos_x(::ars548_interface::msg::DetectionList::_origin_pos_x_type arg)
  {
    msg_.origin_pos_x = std::move(arg);
    return Init_DetectionList_origin_pos_y(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_origin_invalid_flags
{
public:
  explicit Init_DetectionList_origin_invalid_flags(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_pos_x origin_invalid_flags(::ars548_interface::msg::DetectionList::_origin_invalid_flags_type arg)
  {
    msg_.origin_invalid_flags = std::move(arg);
    return Init_DetectionList_origin_pos_x(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_extended_qualifier
{
public:
  explicit Init_DetectionList_extended_qualifier(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_origin_invalid_flags extended_qualifier(::ars548_interface::msg::DetectionList::_extended_qualifier_type arg)
  {
    msg_.extended_qualifier = std::move(arg);
    return Init_DetectionList_origin_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_event_data_qualifier
{
public:
  explicit Init_DetectionList_event_data_qualifier(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_extended_qualifier event_data_qualifier(::ars548_interface::msg::DetectionList::_event_data_qualifier_type arg)
  {
    msg_.event_data_qualifier = std::move(arg);
    return Init_DetectionList_extended_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_timestamp_sync_status
{
public:
  explicit Init_DetectionList_timestamp_sync_status(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_event_data_qualifier timestamp_sync_status(::ars548_interface::msg::DetectionList::_timestamp_sync_status_type arg)
  {
    msg_.timestamp_sync_status = std::move(arg);
    return Init_DetectionList_event_data_qualifier(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_timestamp_seconds
{
public:
  explicit Init_DetectionList_timestamp_seconds(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_timestamp_sync_status timestamp_seconds(::ars548_interface::msg::DetectionList::_timestamp_seconds_type arg)
  {
    msg_.timestamp_seconds = std::move(arg);
    return Init_DetectionList_timestamp_sync_status(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_timestamp_nanoseconds
{
public:
  explicit Init_DetectionList_timestamp_nanoseconds(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_timestamp_seconds timestamp_nanoseconds(::ars548_interface::msg::DetectionList::_timestamp_nanoseconds_type arg)
  {
    msg_.timestamp_nanoseconds = std::move(arg);
    return Init_DetectionList_timestamp_seconds(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_data_id
{
public:
  explicit Init_DetectionList_data_id(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_timestamp_nanoseconds data_id(::ars548_interface::msg::DetectionList::_data_id_type arg)
  {
    msg_.data_id = std::move(arg);
    return Init_DetectionList_timestamp_nanoseconds(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_sqc
{
public:
  explicit Init_DetectionList_sqc(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_data_id sqc(::ars548_interface::msg::DetectionList::_sqc_type arg)
  {
    msg_.sqc = std::move(arg);
    return Init_DetectionList_data_id(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_length
{
public:
  explicit Init_DetectionList_length(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_sqc length(::ars548_interface::msg::DetectionList::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_DetectionList_sqc(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_crc
{
public:
  explicit Init_DetectionList_crc(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_length crc(::ars548_interface::msg::DetectionList::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return Init_DetectionList_length(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_return_code
{
public:
  explicit Init_DetectionList_return_code(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_crc return_code(::ars548_interface::msg::DetectionList::_return_code_type arg)
  {
    msg_.return_code = std::move(arg);
    return Init_DetectionList_crc(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_message_type
{
public:
  explicit Init_DetectionList_message_type(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_return_code message_type(::ars548_interface::msg::DetectionList::_message_type_type arg)
  {
    msg_.message_type = std::move(arg);
    return Init_DetectionList_return_code(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_interface_version
{
public:
  explicit Init_DetectionList_interface_version(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_message_type interface_version(::ars548_interface::msg::DetectionList::_interface_version_type arg)
  {
    msg_.interface_version = std::move(arg);
    return Init_DetectionList_message_type(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_protocol_version
{
public:
  explicit Init_DetectionList_protocol_version(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_interface_version protocol_version(::ars548_interface::msg::DetectionList::_protocol_version_type arg)
  {
    msg_.protocol_version = std::move(arg);
    return Init_DetectionList_interface_version(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_session_id
{
public:
  explicit Init_DetectionList_session_id(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_protocol_version session_id(::ars548_interface::msg::DetectionList::_session_id_type arg)
  {
    msg_.session_id = std::move(arg);
    return Init_DetectionList_protocol_version(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_client_id
{
public:
  explicit Init_DetectionList_client_id(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_session_id client_id(::ars548_interface::msg::DetectionList::_client_id_type arg)
  {
    msg_.client_id = std::move(arg);
    return Init_DetectionList_session_id(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_data_length
{
public:
  explicit Init_DetectionList_data_length(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_client_id data_length(::ars548_interface::msg::DetectionList::_data_length_type arg)
  {
    msg_.data_length = std::move(arg);
    return Init_DetectionList_client_id(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_method_id
{
public:
  explicit Init_DetectionList_method_id(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_data_length method_id(::ars548_interface::msg::DetectionList::_method_id_type arg)
  {
    msg_.method_id = std::move(arg);
    return Init_DetectionList_data_length(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_service_id
{
public:
  explicit Init_DetectionList_service_id(::ars548_interface::msg::DetectionList & msg)
  : msg_(msg)
  {}
  Init_DetectionList_method_id service_id(::ars548_interface::msg::DetectionList::_service_id_type arg)
  {
    msg_.service_id = std::move(arg);
    return Init_DetectionList_method_id(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

class Init_DetectionList_header
{
public:
  Init_DetectionList_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionList_service_id header(::ars548_interface::msg::DetectionList::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectionList_service_id(msg_);
  }

private:
  ::ars548_interface::msg::DetectionList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::DetectionList>()
{
  return ars548_interface::msg::builder::Init_DetectionList_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__BUILDER_HPP_
