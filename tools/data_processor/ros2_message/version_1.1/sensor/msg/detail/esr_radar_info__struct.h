// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sensor:msg/EsrRadarInfo.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__ESR_RADAR_INFO__STRUCT_H_
#define SENSOR__MSG__DETAIL__ESR_RADAR_INFO__STRUCT_H_

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
// Member 'localpose_stamped'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"
// Member 'object_data'
#include "sensor/msg/detail/esr_radar_object__struct.h"

// Struct defined in msg/EsrRadarInfo in the package sensor.
typedef struct sensor__msg__EsrRadarInfo
{
  std_msgs__msg__Header header;
  double local_time;
  double gps_time;
  geometry_msgs__msg__Pose2D localpose_stamped;
  int32_t radar_id;
  int32_t object_num;
  sensor__msg__EsrRadarObject__Sequence object_data;
} sensor__msg__EsrRadarInfo;

// Struct for a sequence of sensor__msg__EsrRadarInfo.
typedef struct sensor__msg__EsrRadarInfo__Sequence
{
  sensor__msg__EsrRadarInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sensor__msg__EsrRadarInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SENSOR__MSG__DETAIL__ESR_RADAR_INFO__STRUCT_H_
