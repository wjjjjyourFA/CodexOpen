// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sensor:msg/ContiRadarObject.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__CONTI_RADAR_OBJECT__STRUCT_H_
#define SENSOR__MSG__DETAIL__CONTI_RADAR_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/ContiRadarObject in the package sensor.
typedef struct sensor__msg__ContiRadarObject
{
  int8_t target_id;
  double range;
  double angle;
  double x;
  double y;
  double z;
  double speed;
  double speed_x;
  double speed_y;
  double height;
  double width;
  double range_rate;
  double lat_rate;
  int8_t track_status;
  int8_t is_acc_target;
  int8_t is_cmbb_target;
  int8_t is_fcw_target;
  int8_t type;
  int8_t confidence;
  int8_t rcsvalue;
} sensor__msg__ContiRadarObject;

// Struct for a sequence of sensor__msg__ContiRadarObject.
typedef struct sensor__msg__ContiRadarObject__Sequence
{
  sensor__msg__ContiRadarObject * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sensor__msg__ContiRadarObject__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SENSOR__MSG__DETAIL__CONTI_RADAR_OBJECT__STRUCT_H_
