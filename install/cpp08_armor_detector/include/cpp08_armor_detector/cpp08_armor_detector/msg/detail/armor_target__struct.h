// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cpp08_armor_detector:msg/ArmorTarget.idl
// generated code does not contain a copyright notice

#ifndef CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__STRUCT_H_
#define CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ArmorTarget in the package cpp08_armor_detector.
/**
  * 装甲板识别结果消息
 */
typedef struct cpp08_armor_detector__msg__ArmorTarget
{
  std_msgs__msg__Header header;
  /// 是否识别到装甲板
  bool is_detected;
  /// 偏航角（度）
  float yaw;
  /// 俯仰角（度）
  float pitch;
  /// 距离（mm）
  float distance;
  /// 卡尔曼滤波后的偏航角
  float filtered_yaw;
  /// 装甲板中心像素X
  float center_x;
  /// 装甲板中心像素Y
  float center_y;
} cpp08_armor_detector__msg__ArmorTarget;

// Struct for a sequence of cpp08_armor_detector__msg__ArmorTarget.
typedef struct cpp08_armor_detector__msg__ArmorTarget__Sequence
{
  cpp08_armor_detector__msg__ArmorTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp08_armor_detector__msg__ArmorTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__STRUCT_H_
