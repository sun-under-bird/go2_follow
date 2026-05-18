// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace uwb_aoa_pkg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LibAoaRobotMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) uwb_aoa_pkg::msg::LibAoaRobotMsg(_init);
}

void LibAoaRobotMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<uwb_aoa_pkg::msg::LibAoaRobotMsg *>(message_memory);
  typed_message->~LibAoaRobotMsg();
}

size_t size_function__LibAoaRobotMsg__rssi(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__LibAoaRobotMsg__rssi(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<int8_t, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__LibAoaRobotMsg__rssi(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<int8_t, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__LibAoaRobotMsg__rssi(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int8_t *>(
    get_const_function__LibAoaRobotMsg__rssi(untyped_member, index));
  auto & value = *reinterpret_cast<int8_t *>(untyped_value);
  value = item;
}

void assign_function__LibAoaRobotMsg__rssi(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int8_t *>(
    get_function__LibAoaRobotMsg__rssi(untyped_member, index));
  const auto & value = *reinterpret_cast<const int8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LibAoaRobotMsg_message_member_array[7] = {
  {
    "r",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg::msg::LibAoaRobotMsg, r),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "a",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg::msg::LibAoaRobotMsg, a),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg::msg::LibAoaRobotMsg, x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg::msg::LibAoaRobotMsg, y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg::msg::LibAoaRobotMsg, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rssi",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg::msg::LibAoaRobotMsg, rssi),  // bytes offset in struct
    nullptr,  // default value
    size_function__LibAoaRobotMsg__rssi,  // size() function pointer
    get_const_function__LibAoaRobotMsg__rssi,  // get_const(index) function pointer
    get_function__LibAoaRobotMsg__rssi,  // get(index) function pointer
    fetch_function__LibAoaRobotMsg__rssi,  // fetch(index, &value) function pointer
    assign_function__LibAoaRobotMsg__rssi,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pos_confidence",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(uwb_aoa_pkg::msg::LibAoaRobotMsg, pos_confidence),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LibAoaRobotMsg_message_members = {
  "uwb_aoa_pkg::msg",  // message namespace
  "LibAoaRobotMsg",  // message name
  7,  // number of fields
  sizeof(uwb_aoa_pkg::msg::LibAoaRobotMsg),
  LibAoaRobotMsg_message_member_array,  // message members
  LibAoaRobotMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  LibAoaRobotMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LibAoaRobotMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LibAoaRobotMsg_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace uwb_aoa_pkg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<uwb_aoa_pkg::msg::LibAoaRobotMsg>()
{
  return &::uwb_aoa_pkg::msg::rosidl_typesupport_introspection_cpp::LibAoaRobotMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, uwb_aoa_pkg, msg, LibAoaRobotMsg)() {
  return &::uwb_aoa_pkg::msg::rosidl_typesupport_introspection_cpp::LibAoaRobotMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
