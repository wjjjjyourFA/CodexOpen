// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/ObjectList.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__STRUCT_H_

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
// Member 'objects'
#include "ars548_interface/msg/detail/object__struct.h"

// Struct defined in msg/ObjectList in the package ars548_interface.
typedef struct ars548_interface__msg__ObjectList
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
  uint8_t object_list_num_of_objects;
  ars548_interface__msg__Object__Sequence objects;
} ars548_interface__msg__ObjectList;

// Struct for a sequence of ars548_interface__msg__ObjectList.
typedef struct ars548_interface__msg__ObjectList__Sequence
{
  ars548_interface__msg__ObjectList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__ObjectList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__OBJECT_LIST__STRUCT_H_
