// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from cpp08_armor_detector:msg/ArmorTarget.idl
// generated code does not contain a copyright notice

#ifndef CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "cpp08_armor_detector/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "cpp08_armor_detector/msg/detail/armor_target__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace cpp08_armor_detector
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cpp08_armor_detector
cdr_serialize(
  const cpp08_armor_detector::msg::ArmorTarget & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cpp08_armor_detector
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  cpp08_armor_detector::msg::ArmorTarget & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cpp08_armor_detector
get_serialized_size(
  const cpp08_armor_detector::msg::ArmorTarget & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cpp08_armor_detector
max_serialized_size_ArmorTarget(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace cpp08_armor_detector

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cpp08_armor_detector
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cpp08_armor_detector, msg, ArmorTarget)();

#ifdef __cplusplus
}
#endif

#endif  // CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
