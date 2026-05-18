// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice

#ifndef UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__BUILDER_HPP_
#define UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace uwb_aoa_pkg
{

namespace msg
{

namespace builder
{

class Init_LibAoaRobotMsg_pos_confidence
{
public:
  explicit Init_LibAoaRobotMsg_pos_confidence(::uwb_aoa_pkg::msg::LibAoaRobotMsg & msg)
  : msg_(msg)
  {}
  ::uwb_aoa_pkg::msg::LibAoaRobotMsg pos_confidence(::uwb_aoa_pkg::msg::LibAoaRobotMsg::_pos_confidence_type arg)
  {
    msg_.pos_confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::uwb_aoa_pkg::msg::LibAoaRobotMsg msg_;
};

class Init_LibAoaRobotMsg_rssi
{
public:
  explicit Init_LibAoaRobotMsg_rssi(::uwb_aoa_pkg::msg::LibAoaRobotMsg & msg)
  : msg_(msg)
  {}
  Init_LibAoaRobotMsg_pos_confidence rssi(::uwb_aoa_pkg::msg::LibAoaRobotMsg::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return Init_LibAoaRobotMsg_pos_confidence(msg_);
  }

private:
  ::uwb_aoa_pkg::msg::LibAoaRobotMsg msg_;
};

class Init_LibAoaRobotMsg_state
{
public:
  explicit Init_LibAoaRobotMsg_state(::uwb_aoa_pkg::msg::LibAoaRobotMsg & msg)
  : msg_(msg)
  {}
  Init_LibAoaRobotMsg_rssi state(::uwb_aoa_pkg::msg::LibAoaRobotMsg::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_LibAoaRobotMsg_rssi(msg_);
  }

private:
  ::uwb_aoa_pkg::msg::LibAoaRobotMsg msg_;
};

class Init_LibAoaRobotMsg_y
{
public:
  explicit Init_LibAoaRobotMsg_y(::uwb_aoa_pkg::msg::LibAoaRobotMsg & msg)
  : msg_(msg)
  {}
  Init_LibAoaRobotMsg_state y(::uwb_aoa_pkg::msg::LibAoaRobotMsg::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_LibAoaRobotMsg_state(msg_);
  }

private:
  ::uwb_aoa_pkg::msg::LibAoaRobotMsg msg_;
};

class Init_LibAoaRobotMsg_x
{
public:
  explicit Init_LibAoaRobotMsg_x(::uwb_aoa_pkg::msg::LibAoaRobotMsg & msg)
  : msg_(msg)
  {}
  Init_LibAoaRobotMsg_y x(::uwb_aoa_pkg::msg::LibAoaRobotMsg::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_LibAoaRobotMsg_y(msg_);
  }

private:
  ::uwb_aoa_pkg::msg::LibAoaRobotMsg msg_;
};

class Init_LibAoaRobotMsg_a
{
public:
  explicit Init_LibAoaRobotMsg_a(::uwb_aoa_pkg::msg::LibAoaRobotMsg & msg)
  : msg_(msg)
  {}
  Init_LibAoaRobotMsg_x a(::uwb_aoa_pkg::msg::LibAoaRobotMsg::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_LibAoaRobotMsg_x(msg_);
  }

private:
  ::uwb_aoa_pkg::msg::LibAoaRobotMsg msg_;
};

class Init_LibAoaRobotMsg_r
{
public:
  Init_LibAoaRobotMsg_r()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LibAoaRobotMsg_a r(::uwb_aoa_pkg::msg::LibAoaRobotMsg::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_LibAoaRobotMsg_a(msg_);
  }

private:
  ::uwb_aoa_pkg::msg::LibAoaRobotMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::uwb_aoa_pkg::msg::LibAoaRobotMsg>()
{
  return uwb_aoa_pkg::msg::builder::Init_LibAoaRobotMsg_r();
}

}  // namespace uwb_aoa_pkg

#endif  // UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__BUILDER_HPP_
