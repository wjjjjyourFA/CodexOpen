// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from self_state:msg/LidarLocalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__LIDAR_LOCAL_POSE__STRUCT_H_
#define SELF_STATE__MSG__DETAIL__LIDAR_LOCAL_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'local_pose'
#include "self_state/msg/detail/local_pose__struct.h"

// Struct defined in msg/LidarLocalPose in the package self_state.
typedef struct self_state__msg__LidarLocalPose
{
  double local_time;
  double utc_time;
  int32_t message_num;
  int32_t pos_type;
  double x;
  double y;
  double z;
  double x_speed;
  double y_speed;
  double z_speed;
  double azimuth;
  double pitch;
  double roll;
  self_state__msg__LocalPose local_pose;
} self_state__msg__LidarLocalPose;

// Struct for a sequence of self_state__msg__LidarLocalPose.
typedef struct self_state__msg__LidarLocalPose__Sequence
{
  self_state__msg__LidarLocalPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} self_state__msg__LidarLocalPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__LIDAR_LOCAL_POSE__STRUCT_H_
