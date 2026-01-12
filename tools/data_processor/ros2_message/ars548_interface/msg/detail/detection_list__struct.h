// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/DetectionList.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'detections'
#include "ars548_interface/msg/detail/detection__struct.h"

// Struct defined in msg/DetectionList in the package ars548_interface.
typedef struct ars548_interface__msg__DetectionList
{
  std_msgs__msg__Header header;
  uint16_t service_id;
  uint16_t method_id;
  uint32_t data_length;
  uint16_t client_id;
  uint16_t session_id;
  uint8_t protocol_version;
  uint8_t interface_version;
  uint8_t message_type;
  uint8_t return_code;
  uint64_t crc;
  uint32_t length;
  uint32_t sqc;
  uint32_t data_id;
  uint32_t timestamp_nanoseconds;
  uint32_t timestamp_seconds;
  uint8_t timestamp_sync_status;
  uint32_t event_data_qualifier;
  uint8_t extended_qualifier;
  uint16_t origin_invalid_flags;
  float origin_pos_x;
  float origin_pos_y;
  float origin_pos_z;
  float origin_x_std;
  float origin_y_std;
  float origin_z_std;
  float origin_roll;
  float origin_pitch;
  float origin_yaw;
  float origin_roll_std;
  float origin_pitch_std;
  float origin_yaw_std;
  uint8_t list_invalid_flags;
  ars548_interface__msg__Detection__Sequence detections;
  float list_rad_vel_domain_min;
  float list_rad_vel_domain_max;
  uint32_t list_num_of_detections;
  float aln_azimuth_correction;
  float aln_elevation_correction;
  uint8_t aln_status;
} ars548_interface__msg__DetectionList;

// Struct for a sequence of ars548_interface__msg__DetectionList.
typedef struct ars548_interface__msg__DetectionList__Sequence
{
  ars548_interface__msg__DetectionList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__DetectionList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__DETECTION_LIST__STRUCT_H_
