// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice

#ifndef UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__TRAITS_HPP_
#define UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace uwb_aoa_pkg
{

namespace msg
{

inline void to_flow_style_yaml(
  const LibAoaRobotMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: a
  {
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: rssi
  {
    if (msg.rssi.size() == 0) {
      out << "rssi: []";
    } else {
      out << "rssi: [";
      size_t pending_items = msg.rssi.size();
      for (auto item : msg.rssi) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pos_confidence
  {
    out << "pos_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_confidence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LibAoaRobotMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "a: ";
    rosidl_generator_traits::value_to_yaml(msg.a, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.rssi.size() == 0) {
      out << "rssi: []\n";
    } else {
      out << "rssi:\n";
      for (auto item : msg.rssi) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: pos_confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_confidence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LibAoaRobotMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace uwb_aoa_pkg

namespace rosidl_generator_traits
{

[[deprecated("use uwb_aoa_pkg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const uwb_aoa_pkg::msg::LibAoaRobotMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  uwb_aoa_pkg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use uwb_aoa_pkg::msg::to_yaml() instead")]]
inline std::string to_yaml(const uwb_aoa_pkg::msg::LibAoaRobotMsg & msg)
{
  return uwb_aoa_pkg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<uwb_aoa_pkg::msg::LibAoaRobotMsg>()
{
  return "uwb_aoa_pkg::msg::LibAoaRobotMsg";
}

template<>
inline const char * name<uwb_aoa_pkg::msg::LibAoaRobotMsg>()
{
  return "uwb_aoa_pkg/msg/LibAoaRobotMsg";
}

template<>
struct has_fixed_size<uwb_aoa_pkg::msg::LibAoaRobotMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<uwb_aoa_pkg::msg::LibAoaRobotMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<uwb_aoa_pkg::msg::LibAoaRobotMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__TRAITS_HPP_
