// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cpp08_armor_detector:msg/ArmorTarget.idl
// generated code does not contain a copyright notice

#ifndef CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__TRAITS_HPP_
#define CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cpp08_armor_detector/msg/detail/armor_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace cpp08_armor_detector
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArmorTarget & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_detected
  {
    out << "is_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_detected, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: filtered_yaw
  {
    out << "filtered_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.filtered_yaw, out);
    out << ", ";
  }

  // member: center_x
  {
    out << "center_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_x, out);
    out << ", ";
  }

  // member: center_y
  {
    out << "center_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmorTarget & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: is_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.is_detected, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: filtered_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filtered_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.filtered_yaw, out);
    out << "\n";
  }

  // member: center_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_x, out);
    out << "\n";
  }

  // member: center_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmorTarget & msg, bool use_flow_style = false)
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

}  // namespace cpp08_armor_detector

namespace rosidl_generator_traits
{

[[deprecated("use cpp08_armor_detector::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cpp08_armor_detector::msg::ArmorTarget & msg,
  std::ostream & out, size_t indentation = 0)
{
  cpp08_armor_detector::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cpp08_armor_detector::msg::to_yaml() instead")]]
inline std::string to_yaml(const cpp08_armor_detector::msg::ArmorTarget & msg)
{
  return cpp08_armor_detector::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cpp08_armor_detector::msg::ArmorTarget>()
{
  return "cpp08_armor_detector::msg::ArmorTarget";
}

template<>
inline const char * name<cpp08_armor_detector::msg::ArmorTarget>()
{
  return "cpp08_armor_detector/msg/ArmorTarget";
}

template<>
struct has_fixed_size<cpp08_armor_detector::msg::ArmorTarget>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<cpp08_armor_detector::msg::ArmorTarget>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<cpp08_armor_detector::msg::ArmorTarget>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__TRAITS_HPP_
