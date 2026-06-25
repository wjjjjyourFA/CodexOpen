// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from self_state:msg/LocalPose.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__LOCAL_POSE__STRUCT_H_
#define SELF_STATE__MSG__DETAIL__LOCAL_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATUS_PARK'.
enum
{
  self_state__msg__LocalPose__STATUS_PARK = 0
};

/// Constant 'STATUS_FORWARD'.
enum
{
  self_state__msg__LocalPose__STATUS_FORWARD = 1
};

/// Constant 'STATUS_BACKWARD'.
enum
{
  self_state__msg__LocalPose__STATUS_BACKWARD = -1
};

// Struct defined in msg/LocalPose in the package self_state.
typedef struct self_state__msg__LocalPose
{
  double local_time;
  double utc_time;
  int32_t message_num;
  double dr_x;
  double dr_y;
  double dr_z;
  double dr_roll;
  double dr_pitch;
  double dr_heading;
  double vehicle_speed;
  double speed_x;
  double speed_y;
  double speed_z;
  int8_t driving_direction;
  double reserved[4];
} self_state__msg__LocalPose;

// Struct for a sequence of self_state__msg__LocalPose.
typedef struct self_state__msg__LocalPose__Sequence
{
  self_state__msg__LocalPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} self_state__msg__LocalPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SELF_STATE__MSG__DETAIL__LOCAL_POSE__STRUCT_H_
