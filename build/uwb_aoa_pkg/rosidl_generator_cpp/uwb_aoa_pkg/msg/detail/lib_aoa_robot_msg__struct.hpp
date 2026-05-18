// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice

#ifndef UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__STRUCT_HPP_
#define UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__uwb_aoa_pkg__msg__LibAoaRobotMsg __attribute__((deprecated))
#else
# define DEPRECATED__uwb_aoa_pkg__msg__LibAoaRobotMsg __declspec(deprecated)
#endif

namespace uwb_aoa_pkg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LibAoaRobotMsg_
{
  using Type = LibAoaRobotMsg_<ContainerAllocator>;

  explicit LibAoaRobotMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0.0;
      this->a = 0.0;
      this->x = 0.0;
      this->y = 0.0;
      this->state = 0;
      std::fill<typename std::array<int8_t, 6>::iterator, int8_t>(this->rssi.begin(), this->rssi.end(), 0);
      this->pos_confidence = 0;
    }
  }

  explicit LibAoaRobotMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rssi(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->r = 0.0;
      this->a = 0.0;
      this->x = 0.0;
      this->y = 0.0;
      this->state = 0;
      std::fill<typename std::array<int8_t, 6>::iterator, int8_t>(this->rssi.begin(), this->rssi.end(), 0);
      this->pos_confidence = 0;
    }
  }

  // field types and members
  using _r_type =
    double;
  _r_type r;
  using _a_type =
    double;
  _a_type a;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _state_type =
    int8_t;
  _state_type state;
  using _rssi_type =
    std::array<int8_t, 6>;
  _rssi_type rssi;
  using _pos_confidence_type =
    uint8_t;
  _pos_confidence_type pos_confidence;

  // setters for named parameter idiom
  Type & set__r(
    const double & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__a(
    const double & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__state(
    const int8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__rssi(
    const std::array<int8_t, 6> & _arg)
  {
    this->rssi = _arg;
    return *this;
  }
  Type & set__pos_confidence(
    const uint8_t & _arg)
  {
    this->pos_confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uwb_aoa_pkg__msg__LibAoaRobotMsg
    std::shared_ptr<uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uwb_aoa_pkg__msg__LibAoaRobotMsg
    std::shared_ptr<uwb_aoa_pkg::msg::LibAoaRobotMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LibAoaRobotMsg_ & other) const
  {
    if (this->r != other.r) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->rssi != other.rssi) {
      return false;
    }
    if (this->pos_confidence != other.pos_confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const LibAoaRobotMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LibAoaRobotMsg_

// alias to use template instance with default allocator
using LibAoaRobotMsg =
  uwb_aoa_pkg::msg::LibAoaRobotMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uwb_aoa_pkg

#endif  // UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__STRUCT_HPP_
