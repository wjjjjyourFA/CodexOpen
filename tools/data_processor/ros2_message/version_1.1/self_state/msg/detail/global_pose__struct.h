// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from self_state:msg/GlobalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__GLOBAL_POSE__STRUCT_H_
#define SELF_STATE__MSG__DETAIL__GLOBAL_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ins_status'
#include "self_state/msg/detail/ins_status__struct.h"
// Member 'pos_type'
#include "sensor/msg/detail/pos_type__struct.h"

// Struct defined in msg/GlobalPose in the package self_state.
typedef struct self_state__msg__GlobalPose
{
  double local_time;
  double utc_time;
  int32_t message_num;
  self_state__msg__InsStatus ins_status;
  sensor__msg__PosType pos_type;
  double gauss_x;
  double gauss_y;
  double height;
  double v_north;
  double v_east;
  double v_up;
  double roll;
  double pitch;
  double azimuth;
  double dev_gauss_x;
  double dev_gauss_y;
  double dev_height;
  double dev_v_north;
  double dev_v_east;
  double dev_v_up;
  double dev_roll;
  double dev_pitch;
  double dev_azimuth;
  double latitude;
  double longitude;
  double reserved[4];
} self_state__msg__GlobalPose;

// Struct for a sequence of self_state__msg__GlobalPose.
typedef struct self_state__msg__GlobalPose__Sequence
{
  self_state__msg__GlobalPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} self_state__msg__GlobalPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__GLOBAL_POSE__STRUCT_H_
