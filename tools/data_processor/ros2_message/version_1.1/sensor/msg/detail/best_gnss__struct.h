// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sensor:msg/BestGnss.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__BEST_GNSS__STRUCT_H_
#define SENSOR__MSG__DETAIL__BEST_GNSS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'sol_status'
#include "sensor/msg/detail/gnss_solution_status__struct.h"
// Member 'pos_type'
#include "sensor/msg/detail/pos_type__struct.h"

// Struct defined in msg/BestGnss in the package sensor.
typedef struct sensor__msg__BestGnss
{
  double local_time;
  double u_t_c_time;
  int32_t message_num;
  sensor__msg__GnssSolutionStatus sol_status;
  sensor__msg__PosType pos_type;
  double num_satellite_tracked;
  double num_satellite_used;
  double latitude_gnss;
  double longitude_gnss;
  double height_gnss;
} sensor__msg__BestGnss;

// Struct for a sequence of sensor__msg__BestGnss.
typedef struct sensor__msg__BestGnss__Sequence
{
  sensor__msg__BestGnss * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sensor__msg__BestGnss__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SENSOR__MSG__DETAIL__BEST_GNSS__STRUCT_H_
