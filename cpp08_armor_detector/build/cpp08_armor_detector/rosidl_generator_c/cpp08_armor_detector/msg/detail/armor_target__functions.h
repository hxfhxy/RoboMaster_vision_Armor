// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from cpp08_armor_detector:msg/ArmorTarget.idl
// generated code does not contain a copyright notice

#ifndef CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__FUNCTIONS_H_
#define CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "cpp08_armor_detector/msg/rosidl_generator_c__visibility_control.h"

#include "cpp08_armor_detector/msg/detail/armor_target__struct.h"

/// Initialize msg/ArmorTarget message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cpp08_armor_detector__msg__ArmorTarget
 * )) before or use
 * cpp08_armor_detector__msg__ArmorTarget__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
bool
cpp08_armor_detector__msg__ArmorTarget__init(cpp08_armor_detector__msg__ArmorTarget * msg);

/// Finalize msg/ArmorTarget message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
void
cpp08_armor_detector__msg__ArmorTarget__fini(cpp08_armor_detector__msg__ArmorTarget * msg);

/// Create msg/ArmorTarget message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cpp08_armor_detector__msg__ArmorTarget__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
cpp08_armor_detector__msg__ArmorTarget *
cpp08_armor_detector__msg__ArmorTarget__create();

/// Destroy msg/ArmorTarget message.
/**
 * It calls
 * cpp08_armor_detector__msg__ArmorTarget__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
void
cpp08_armor_detector__msg__ArmorTarget__destroy(cpp08_armor_detector__msg__ArmorTarget * msg);

/// Check for msg/ArmorTarget message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
bool
cpp08_armor_detector__msg__ArmorTarget__are_equal(const cpp08_armor_detector__msg__ArmorTarget * lhs, const cpp08_armor_detector__msg__ArmorTarget * rhs);

/// Copy a msg/ArmorTarget message.
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
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
bool
cpp08_armor_detector__msg__ArmorTarget__copy(
  const cpp08_armor_detector__msg__ArmorTarget * input,
  cpp08_armor_detector__msg__ArmorTarget * output);

/// Initialize array of msg/ArmorTarget messages.
/**
 * It allocates the memory for the number of elements and calls
 * cpp08_armor_detector__msg__ArmorTarget__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
bool
cpp08_armor_detector__msg__ArmorTarget__Sequence__init(cpp08_armor_detector__msg__ArmorTarget__Sequence * array, size_t size);

/// Finalize array of msg/ArmorTarget messages.
/**
 * It calls
 * cpp08_armor_detector__msg__ArmorTarget__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
void
cpp08_armor_detector__msg__ArmorTarget__Sequence__fini(cpp08_armor_detector__msg__ArmorTarget__Sequence * array);

/// Create array of msg/ArmorTarget messages.
/**
 * It allocates the memory for the array and calls
 * cpp08_armor_detector__msg__ArmorTarget__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
cpp08_armor_detector__msg__ArmorTarget__Sequence *
cpp08_armor_detector__msg__ArmorTarget__Sequence__create(size_t size);

/// Destroy array of msg/ArmorTarget messages.
/**
 * It calls
 * cpp08_armor_detector__msg__ArmorTarget__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
void
cpp08_armor_detector__msg__ArmorTarget__Sequence__destroy(cpp08_armor_detector__msg__ArmorTarget__Sequence * array);

/// Check for msg/ArmorTarget message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
bool
cpp08_armor_detector__msg__ArmorTarget__Sequence__are_equal(const cpp08_armor_detector__msg__ArmorTarget__Sequence * lhs, const cpp08_armor_detector__msg__ArmorTarget__Sequence * rhs);

/// Copy an array of msg/ArmorTarget messages.
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
ROSIDL_GENERATOR_C_PUBLIC_cpp08_armor_detector
bool
cpp08_armor_detector__msg__ArmorTarget__Sequence__copy(
  const cpp08_armor_detector__msg__ArmorTarget__Sequence * input,
  cpp08_armor_detector__msg__ArmorTarget__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__FUNCTIONS_H_
