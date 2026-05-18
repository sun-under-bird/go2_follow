// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice

#ifndef UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__STRUCT_H_
#define UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LibAoaRobotMsg in the package uwb_aoa_pkg.
typedef struct uwb_aoa_pkg__msg__LibAoaRobotMsg
{
  double r;
  double a;
  double x;
  double y;
  int8_t state;
  int8_t rssi[6];
  uint8_t pos_confidence;
} uwb_aoa_pkg__msg__LibAoaRobotMsg;

// Struct for a sequence of uwb_aoa_pkg__msg__LibAoaRobotMsg.
typedef struct uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence
{
  uwb_aoa_pkg__msg__LibAoaRobotMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__STRUCT_H_
