// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ars548_interface:msg/CharacteristicSpeed.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__STRUCT_H_
#define ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__STRUCT_H_

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

// Struct defined in msg/CharacteristicSpeed in the package ars548_interface.
typedef struct ars548_interface__msg__CharacteristicSpeed
{
  std_msgs__msg__Header header;
  uint8_t characteristic_speed_err_amp;
  uint8_t qualifier_characteristic_speed;
  uint8_t characteristic_speed;
} ars548_interface__msg__CharacteristicSpeed;

// Struct for a sequence of ars548_interface__msg__CharacteristicSpeed.
typedef struct ars548_interface__msg__CharacteristicSpeed__Sequence
{
  ars548_interface__msg__CharacteristicSpeed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ars548_interface__msg__CharacteristicSpeed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARS548_INTERFACE__MSG__DETAIL__CHARACTERISTIC_SPEED__STRUCT_H_
