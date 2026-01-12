// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sensor:msg/PosType.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__POS_TYPE__STRUCT_H_
#define SENSOR__MSG__DETAIL__POS_TYPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NONE'.
enum
{
  sensor__msg__PosType__NONE = 0
};

/// Constant 'FIXEDPOS'.
enum
{
  sensor__msg__PosType__FIXEDPOS = 1
};

/// Constant 'FIXEDHEIGHT'.
enum
{
  sensor__msg__PosType__FIXEDHEIGHT = 2
};

/// Constant 'FLOATCONV'.
enum
{
  sensor__msg__PosType__FLOATCONV = 4
};

/// Constant 'WIDELANE'.
enum
{
  sensor__msg__PosType__WIDELANE = 5
};

/// Constant 'NARROWLANE'.
enum
{
  sensor__msg__PosType__NARROWLANE = 6
};

/// Constant 'DOPPLER_VELOCITY'.
enum
{
  sensor__msg__PosType__DOPPLER_VELOCITY = 8
};

/// Constant 'SINGLE'.
enum
{
  sensor__msg__PosType__SINGLE = 16
};

/// Constant 'PSRDIFF'.
enum
{
  sensor__msg__PosType__PSRDIFF = 17
};

/// Constant 'WAAS'.
enum
{
  sensor__msg__PosType__WAAS = 18
};

/// Constant 'PROPOGATED'.
enum
{
  sensor__msg__PosType__PROPOGATED = 19
};

/// Constant 'OMNISTAR'.
enum
{
  sensor__msg__PosType__OMNISTAR = 20
};

/// Constant 'L1_FLOAT'.
enum
{
  sensor__msg__PosType__L1_FLOAT = 32
};

/// Constant 'IONOFREE_FLOAT'.
enum
{
  sensor__msg__PosType__IONOFREE_FLOAT = 33
};

/// Constant 'NARROW_FLOAT'.
enum
{
  sensor__msg__PosType__NARROW_FLOAT = 34
};

/// Constant 'L1_INT'.
enum
{
  sensor__msg__PosType__L1_INT = 48
};

/// Constant 'WIDE_INT'.
enum
{
  sensor__msg__PosType__WIDE_INT = 49
};

/// Constant 'NARROW_INT'.
enum
{
  sensor__msg__PosType__NARROW_INT = 50
};

/// Constant 'RTK_DIRECT_INS'.
enum
{
  sensor__msg__PosType__RTK_DIRECT_INS = 51
};

/// Constant 'INS_SBAS'.
enum
{
  sensor__msg__PosType__INS_SBAS = 52
};

/// Constant 'INS_PSRSP'.
enum
{
  sensor__msg__PosType__INS_PSRSP = 53
};

/// Constant 'INS_PSRDIFF'.
enum
{
  sensor__msg__PosType__INS_PSRDIFF = 54
};

/// Constant 'INS_RTKFLOAT'.
enum
{
  sensor__msg__PosType__INS_RTKFLOAT = 55
};

/// Constant 'INS_RTKFIXED'.
enum
{
  sensor__msg__PosType__INS_RTKFIXED = 56
};

/// Constant 'INS_OMNISTAR'.
enum
{
  sensor__msg__PosType__INS_OMNISTAR = 57
};

/// Constant 'INS_OMNISTAR_HP'.
enum
{
  sensor__msg__PosType__INS_OMNISTAR_HP = 58
};

/// Constant 'INS_OMNISTAR_XP'.
enum
{
  sensor__msg__PosType__INS_OMNISTAR_XP = 59
};

/// Constant 'OMNISTAR_HP'.
enum
{
  sensor__msg__PosType__OMNISTAR_HP = 64
};

/// Constant 'OMNISTAR_XP'.
enum
{
  sensor__msg__PosType__OMNISTAR_XP = 65
};

/// Constant 'PPP_CONVERGING'.
enum
{
  sensor__msg__PosType__PPP_CONVERGING = 68
};

/// Constant 'PPP'.
enum
{
  sensor__msg__PosType__PPP = 69
};

/// Constant 'INS_PPP_CONVERGING'.
enum
{
  sensor__msg__PosType__INS_PPP_CONVERGING = 73
};

/// Constant 'INS_PPP'.
enum
{
  sensor__msg__PosType__INS_PPP = 74
};

// Struct defined in msg/PosType in the package sensor.
typedef struct sensor__msg__PosType
{
  int8_t pos_type;
} sensor__msg__PosType;

// Struct for a sequence of sensor__msg__PosType.
typedef struct sensor__msg__PosType__Sequence
{
  sensor__msg__PosType * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sensor__msg__PosType__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SENSOR__MSG__DETAIL__POS_TYPE__STRUCT_H_
