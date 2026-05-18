// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice

#ifndef UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__FUNCTIONS_H_
#define UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "uwb_aoa_pkg/msg/rosidl_generator_c__visibility_control.h"

#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__struct.h"

/// Initialize msg/LibAoaRobotMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * uwb_aoa_pkg__msg__LibAoaRobotMsg
 * )) before or use
 * uwb_aoa_pkg__msg__LibAoaRobotMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__init(uwb_aoa_pkg__msg__LibAoaRobotMsg * msg);

/// Finalize msg/LibAoaRobotMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
void
uwb_aoa_pkg__msg__LibAoaRobotMsg__fini(uwb_aoa_pkg__msg__LibAoaRobotMsg * msg);

/// Create msg/LibAoaRobotMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * uwb_aoa_pkg__msg__LibAoaRobotMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
uwb_aoa_pkg__msg__LibAoaRobotMsg *
uwb_aoa_pkg__msg__LibAoaRobotMsg__create();

/// Destroy msg/LibAoaRobotMsg message.
/**
 * It calls
 * uwb_aoa_pkg__msg__LibAoaRobotMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
void
uwb_aoa_pkg__msg__LibAoaRobotMsg__destroy(uwb_aoa_pkg__msg__LibAoaRobotMsg * msg);

/// Check for msg/LibAoaRobotMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__are_equal(const uwb_aoa_pkg__msg__LibAoaRobotMsg * lhs, const uwb_aoa_pkg__msg__LibAoaRobotMsg * rhs);

/// Copy a msg/LibAoaRobotMsg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__copy(
  const uwb_aoa_pkg__msg__LibAoaRobotMsg * input,
  uwb_aoa_pkg__msg__LibAoaRobotMsg * output);

/// Initialize array of msg/LibAoaRobotMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * uwb_aoa_pkg__msg__LibAoaRobotMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__init(uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * array, size_t size);

/// Finalize array of msg/LibAoaRobotMsg messages.
/**
 * It calls
 * uwb_aoa_pkg__msg__LibAoaRobotMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
void
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__fini(uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * array);

/// Create array of msg/LibAoaRobotMsg messages.
/**
 * It allocates the memory for the array and calls
 * uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence *
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__create(size_t size);

/// Destroy array of msg/LibAoaRobotMsg messages.
/**
 * It calls
 * uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
void
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__destroy(uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * array);

/// Check for msg/LibAoaRobotMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__are_equal(const uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * lhs, const uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * rhs);

/// Copy an array of msg/LibAoaRobotMsg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_uwb_aoa_pkg
bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__copy(
  const uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * input,
  uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UWB_AOA_PKG__MSG__DETAIL__LIB_AOA_ROBOT_MSG__FUNCTIONS_H_
