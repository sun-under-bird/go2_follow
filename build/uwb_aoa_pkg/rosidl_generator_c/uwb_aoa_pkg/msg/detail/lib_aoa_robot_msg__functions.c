// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from uwb_aoa_pkg:msg/LibAoaRobotMsg.idl
// generated code does not contain a copyright notice
#include "uwb_aoa_pkg/msg/detail/lib_aoa_robot_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__init(uwb_aoa_pkg__msg__LibAoaRobotMsg * msg)
{
  if (!msg) {
    return false;
  }
  // r
  // a
  // x
  // y
  // state
  // rssi
  // pos_confidence
  return true;
}

void
uwb_aoa_pkg__msg__LibAoaRobotMsg__fini(uwb_aoa_pkg__msg__LibAoaRobotMsg * msg)
{
  if (!msg) {
    return;
  }
  // r
  // a
  // x
  // y
  // state
  // rssi
  // pos_confidence
}

bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__are_equal(const uwb_aoa_pkg__msg__LibAoaRobotMsg * lhs, const uwb_aoa_pkg__msg__LibAoaRobotMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // r
  if (lhs->r != rhs->r) {
    return false;
  }
  // a
  if (lhs->a != rhs->a) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // rssi
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->rssi[i] != rhs->rssi[i]) {
      return false;
    }
  }
  // pos_confidence
  if (lhs->pos_confidence != rhs->pos_confidence) {
    return false;
  }
  return true;
}

bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__copy(
  const uwb_aoa_pkg__msg__LibAoaRobotMsg * input,
  uwb_aoa_pkg__msg__LibAoaRobotMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // r
  output->r = input->r;
  // a
  output->a = input->a;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // state
  output->state = input->state;
  // rssi
  for (size_t i = 0; i < 6; ++i) {
    output->rssi[i] = input->rssi[i];
  }
  // pos_confidence
  output->pos_confidence = input->pos_confidence;
  return true;
}

uwb_aoa_pkg__msg__LibAoaRobotMsg *
uwb_aoa_pkg__msg__LibAoaRobotMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uwb_aoa_pkg__msg__LibAoaRobotMsg * msg = (uwb_aoa_pkg__msg__LibAoaRobotMsg *)allocator.allocate(sizeof(uwb_aoa_pkg__msg__LibAoaRobotMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(uwb_aoa_pkg__msg__LibAoaRobotMsg));
  bool success = uwb_aoa_pkg__msg__LibAoaRobotMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
uwb_aoa_pkg__msg__LibAoaRobotMsg__destroy(uwb_aoa_pkg__msg__LibAoaRobotMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    uwb_aoa_pkg__msg__LibAoaRobotMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__init(uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uwb_aoa_pkg__msg__LibAoaRobotMsg * data = NULL;

  if (size) {
    data = (uwb_aoa_pkg__msg__LibAoaRobotMsg *)allocator.zero_allocate(size, sizeof(uwb_aoa_pkg__msg__LibAoaRobotMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = uwb_aoa_pkg__msg__LibAoaRobotMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        uwb_aoa_pkg__msg__LibAoaRobotMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__fini(uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      uwb_aoa_pkg__msg__LibAoaRobotMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence *
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * array = (uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence *)allocator.allocate(sizeof(uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__destroy(uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__are_equal(const uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * lhs, const uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!uwb_aoa_pkg__msg__LibAoaRobotMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__copy(
  const uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * input,
  uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(uwb_aoa_pkg__msg__LibAoaRobotMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    uwb_aoa_pkg__msg__LibAoaRobotMsg * data =
      (uwb_aoa_pkg__msg__LibAoaRobotMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!uwb_aoa_pkg__msg__LibAoaRobotMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          uwb_aoa_pkg__msg__LibAoaRobotMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!uwb_aoa_pkg__msg__LibAoaRobotMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
