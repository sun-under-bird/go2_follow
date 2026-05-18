// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__rosidl_typesupport_introspection_c.h"
#include "uwb_aoa_pkg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__functions.h"
#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  uwb_aoa_pkg__msg__LibAoaRobotMsg__init(message_memory);
}

void uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_fini_function(void * message_memory)
{
  uwb_aoa_pkg__msg__LibAoaRobotMsg__fini(message_memory);
}

size_t uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__size_function__LibAoaRobotMsg__rssi(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__get_const_function__LibAoaRobotMsg__rssi(
  const void * untyped_member, size_t index)
{
  const int8_t * member =
    (const int8_t *)(untyped_member);
  return &member[index];
}

void * uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__get_function__LibAoaRobotMsg__rssi(
  void * untyped_member, size_t index)
{
  int8_t * member =
    (int8_t *)(untyped_member);
  return &member[index];
}

void uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__fetch_function__LibAoaRobotMsg__rssi(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int8_t * item =
    ((const int8_t *)
    uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__get_const_function__LibAoaRobotMsg__rssi(untyped_member, index));
  int8_t * value =
    (int8_t *)(untyped_value);
  *value = *item;
}

void uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__assign_function__LibAoaRobotMsg__rssi(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int8_t * item =
    ((int8_t *)
    uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__get_function__LibAoaRobotMsg__rssi(untyped_member, index));
  const int8_t * value =
    (const int8_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_message_member_array[7] = {
  {
    "r",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg__msg__LibAoaRobotMsg, r),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "a",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg__msg__LibAoaRobotMsg, a),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg__msg__LibAoaRobotMsg, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg__msg__LibAoaRobotMsg, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg__msg__LibAoaRobotMsg, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rssi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg__msg__LibAoaRobotMsg, rssi),  // bytes offset in struct
    NULL,  // default value
    uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__size_function__LibAoaRobotMsg__rssi,  // size() function pointer
    uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__get_const_function__LibAoaRobotMsg__rssi,  // get_const(index) function pointer
    uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__get_function__LibAoaRobotMsg__rssi,  // get(index) function pointer
    uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__fetch_function__LibAoaRobotMsg__rssi,  // fetch(index, &value) function pointer
    uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__assign_function__LibAoaRobotMsg__rssi,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos_confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg__msg__LibAoaRobotMsg, pos_confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_message_members = {
  "uwb_aoa_pkg__msg",  // message namespace
  "LibAoaRobotMsg",  // message name
  7,  // number of fields
  sizeof(uwb_aoa_pkg__msg__LibAoaRobotMsg),
  uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_message_member_array,  // message members
  uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_message_type_support_handle = {
  0,
  &uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_uwb_aoa_pkg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, uwb_aoa_pkg, msg, LibAoaRobotMsg)() {
  if (!uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_message_type_support_handle.typesupport_identifier) {
    uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &uwb_aoa_pkg__msg__LibAoaRobotMsg__rosidl_typesupport_introspection_c__LibAoaRobotMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
