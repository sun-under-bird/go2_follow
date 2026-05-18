// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice
#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "uwb_aoa_pkg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__struct.h"
#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _LibAoaRobotMsg__ros_msg_type = uwb_aoa_pkg__msg__LibAoaRobotMsg;

static bool _LibAoaRobotMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LibAoaRobotMsg__ros_msg_type * ros_message = static_cast<const _LibAoaRobotMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: r
  {
    cdr << ros_message->r;
  }

  // Field name: a
  {
    cdr << ros_message->a;
  }

  // Field name: x
  {
    cdr << ros_message->x;
  }

  // Field name: y
  {
    cdr << ros_message->y;
  }

  // Field name: state
  {
    cdr << ros_message->state;
  }

  // Field name: rssi
  {
    size_t size = 6;
    auto array_ptr = ros_message->rssi;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: pos_confidence
  {
    cdr << ros_message->pos_confidence;
  }

  return true;
}

static bool _LibAoaRobotMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LibAoaRobotMsg__ros_msg_type * ros_message = static_cast<_LibAoaRobotMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: r
  {
    cdr >> ros_message->r;
  }

  // Field name: a
  {
    cdr >> ros_message->a;
  }

  // Field name: x
  {
    cdr >> ros_message->x;
  }

  // Field name: y
  {
    cdr >> ros_message->y;
  }

  // Field name: state
  {
    cdr >> ros_message->state;
  }

  // Field name: rssi
  {
    size_t size = 6;
    auto array_ptr = ros_message->rssi;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: pos_confidence
  {
    cdr >> ros_message->pos_confidence;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uwb_aoa_pkg
size_t get_serialized_size_uwb_aoa_pkg__msg__LibAoaRobotMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LibAoaRobotMsg__ros_msg_type * ros_message = static_cast<const _LibAoaRobotMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name r
  {
    size_t item_size = sizeof(ros_message->r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name a
  {
    size_t item_size = sizeof(ros_message->a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name x
  {
    size_t item_size = sizeof(ros_message->x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y
  {
    size_t item_size = sizeof(ros_message->y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name state
  {
    size_t item_size = sizeof(ros_message->state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rssi
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->rssi;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pos_confidence
  {
    size_t item_size = sizeof(ros_message->pos_confidence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LibAoaRobotMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_uwb_aoa_pkg__msg__LibAoaRobotMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_uwb_aoa_pkg
size_t max_serialized_size_uwb_aoa_pkg__msg__LibAoaRobotMsg(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: a
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rssi
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pos_confidence
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = uwb_aoa_pkg__msg__LibAoaRobotMsg;
    is_plain =
      (
      offsetof(DataType, pos_confidence) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _LibAoaRobotMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_uwb_aoa_pkg__msg__LibAoaRobotMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LibAoaRobotMsg = {
  "uwb_aoa_pkg::msg",
  "LibAoaRobotMsg",
  _LibAoaRobotMsg__cdr_serialize,
  _LibAoaRobotMsg__cdr_deserialize,
  _LibAoaRobotMsg__get_serialized_size,
  _LibAoaRobotMsg__max_serialized_size
};

static rosidl_message_type_support_t _LibAoaRobotMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LibAoaRobotMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, uwb_aoa_pkg, msg, LibAoaRobotMsg)() {
  return &_LibAoaRobotMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
