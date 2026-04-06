// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cpp08_armor_detector:msg/ArmorTarget.idl
// generated code does not contain a copyright notice
#include "cpp08_armor_detector/msg/detail/armor_target__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
cpp08_armor_detector__msg__ArmorTarget__init(cpp08_armor_detector__msg__ArmorTarget * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    cpp08_armor_detector__msg__ArmorTarget__fini(msg);
    return false;
  }
  // is_detected
  // yaw
  // pitch
  // distance
  // filtered_yaw
  // center_x
  // center_y
  return true;
}

void
cpp08_armor_detector__msg__ArmorTarget__fini(cpp08_armor_detector__msg__ArmorTarget * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_detected
  // yaw
  // pitch
  // distance
  // filtered_yaw
  // center_x
  // center_y
}

bool
cpp08_armor_detector__msg__ArmorTarget__are_equal(const cpp08_armor_detector__msg__ArmorTarget * lhs, const cpp08_armor_detector__msg__ArmorTarget * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // is_detected
  if (lhs->is_detected != rhs->is_detected) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // filtered_yaw
  if (lhs->filtered_yaw != rhs->filtered_yaw) {
    return false;
  }
  // center_x
  if (lhs->center_x != rhs->center_x) {
    return false;
  }
  // center_y
  if (lhs->center_y != rhs->center_y) {
    return false;
  }
  return true;
}

bool
cpp08_armor_detector__msg__ArmorTarget__copy(
  const cpp08_armor_detector__msg__ArmorTarget * input,
  cpp08_armor_detector__msg__ArmorTarget * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // is_detected
  output->is_detected = input->is_detected;
  // yaw
  output->yaw = input->yaw;
  // pitch
  output->pitch = input->pitch;
  // distance
  output->distance = input->distance;
  // filtered_yaw
  output->filtered_yaw = input->filtered_yaw;
  // center_x
  output->center_x = input->center_x;
  // center_y
  output->center_y = input->center_y;
  return true;
}

cpp08_armor_detector__msg__ArmorTarget *
cpp08_armor_detector__msg__ArmorTarget__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp08_armor_detector__msg__ArmorTarget * msg = (cpp08_armor_detector__msg__ArmorTarget *)allocator.allocate(sizeof(cpp08_armor_detector__msg__ArmorTarget), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp08_armor_detector__msg__ArmorTarget));
  bool success = cpp08_armor_detector__msg__ArmorTarget__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cpp08_armor_detector__msg__ArmorTarget__destroy(cpp08_armor_detector__msg__ArmorTarget * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cpp08_armor_detector__msg__ArmorTarget__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cpp08_armor_detector__msg__ArmorTarget__Sequence__init(cpp08_armor_detector__msg__ArmorTarget__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp08_armor_detector__msg__ArmorTarget * data = NULL;

  if (size) {
    data = (cpp08_armor_detector__msg__ArmorTarget *)allocator.zero_allocate(size, sizeof(cpp08_armor_detector__msg__ArmorTarget), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp08_armor_detector__msg__ArmorTarget__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp08_armor_detector__msg__ArmorTarget__fini(&data[i - 1]);
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
cpp08_armor_detector__msg__ArmorTarget__Sequence__fini(cpp08_armor_detector__msg__ArmorTarget__Sequence * array)
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
      cpp08_armor_detector__msg__ArmorTarget__fini(&array->data[i]);
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

cpp08_armor_detector__msg__ArmorTarget__Sequence *
cpp08_armor_detector__msg__ArmorTarget__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp08_armor_detector__msg__ArmorTarget__Sequence * array = (cpp08_armor_detector__msg__ArmorTarget__Sequence *)allocator.allocate(sizeof(cpp08_armor_detector__msg__ArmorTarget__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cpp08_armor_detector__msg__ArmorTarget__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cpp08_armor_detector__msg__ArmorTarget__Sequence__destroy(cpp08_armor_detector__msg__ArmorTarget__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cpp08_armor_detector__msg__ArmorTarget__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cpp08_armor_detector__msg__ArmorTarget__Sequence__are_equal(const cpp08_armor_detector__msg__ArmorTarget__Sequence * lhs, const cpp08_armor_detector__msg__ArmorTarget__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp08_armor_detector__msg__ArmorTarget__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp08_armor_detector__msg__ArmorTarget__Sequence__copy(
  const cpp08_armor_detector__msg__ArmorTarget__Sequence * input,
  cpp08_armor_detector__msg__ArmorTarget__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp08_armor_detector__msg__ArmorTarget);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cpp08_armor_detector__msg__ArmorTarget * data =
      (cpp08_armor_detector__msg__ArmorTarget *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp08_armor_detector__msg__ArmorTarget__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cpp08_armor_detector__msg__ArmorTarget__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp08_armor_detector__msg__ArmorTarget__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
