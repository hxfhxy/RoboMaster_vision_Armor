// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cpp08_armor_detector:msg/ArmorTarget.idl
// generated code does not contain a copyright notice

#ifndef CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__BUILDER_HPP_
#define CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cpp08_armor_detector/msg/detail/armor_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cpp08_armor_detector
{

namespace msg
{

namespace builder
{

class Init_ArmorTarget_center_y
{
public:
  explicit Init_ArmorTarget_center_y(::cpp08_armor_detector::msg::ArmorTarget & msg)
  : msg_(msg)
  {}
  ::cpp08_armor_detector::msg::ArmorTarget center_y(::cpp08_armor_detector::msg::ArmorTarget::_center_y_type arg)
  {
    msg_.center_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp08_armor_detector::msg::ArmorTarget msg_;
};

class Init_ArmorTarget_center_x
{
public:
  explicit Init_ArmorTarget_center_x(::cpp08_armor_detector::msg::ArmorTarget & msg)
  : msg_(msg)
  {}
  Init_ArmorTarget_center_y center_x(::cpp08_armor_detector::msg::ArmorTarget::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_ArmorTarget_center_y(msg_);
  }

private:
  ::cpp08_armor_detector::msg::ArmorTarget msg_;
};

class Init_ArmorTarget_filtered_yaw
{
public:
  explicit Init_ArmorTarget_filtered_yaw(::cpp08_armor_detector::msg::ArmorTarget & msg)
  : msg_(msg)
  {}
  Init_ArmorTarget_center_x filtered_yaw(::cpp08_armor_detector::msg::ArmorTarget::_filtered_yaw_type arg)
  {
    msg_.filtered_yaw = std::move(arg);
    return Init_ArmorTarget_center_x(msg_);
  }

private:
  ::cpp08_armor_detector::msg::ArmorTarget msg_;
};

class Init_ArmorTarget_distance
{
public:
  explicit Init_ArmorTarget_distance(::cpp08_armor_detector::msg::ArmorTarget & msg)
  : msg_(msg)
  {}
  Init_ArmorTarget_filtered_yaw distance(::cpp08_armor_detector::msg::ArmorTarget::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_ArmorTarget_filtered_yaw(msg_);
  }

private:
  ::cpp08_armor_detector::msg::ArmorTarget msg_;
};

class Init_ArmorTarget_pitch
{
public:
  explicit Init_ArmorTarget_pitch(::cpp08_armor_detector::msg::ArmorTarget & msg)
  : msg_(msg)
  {}
  Init_ArmorTarget_distance pitch(::cpp08_armor_detector::msg::ArmorTarget::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ArmorTarget_distance(msg_);
  }

private:
  ::cpp08_armor_detector::msg::ArmorTarget msg_;
};

class Init_ArmorTarget_yaw
{
public:
  explicit Init_ArmorTarget_yaw(::cpp08_armor_detector::msg::ArmorTarget & msg)
  : msg_(msg)
  {}
  Init_ArmorTarget_pitch yaw(::cpp08_armor_detector::msg::ArmorTarget::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_ArmorTarget_pitch(msg_);
  }

private:
  ::cpp08_armor_detector::msg::ArmorTarget msg_;
};

class Init_ArmorTarget_is_detected
{
public:
  explicit Init_ArmorTarget_is_detected(::cpp08_armor_detector::msg::ArmorTarget & msg)
  : msg_(msg)
  {}
  Init_ArmorTarget_yaw is_detected(::cpp08_armor_detector::msg::ArmorTarget::_is_detected_type arg)
  {
    msg_.is_detected = std::move(arg);
    return Init_ArmorTarget_yaw(msg_);
  }

private:
  ::cpp08_armor_detector::msg::ArmorTarget msg_;
};

class Init_ArmorTarget_header
{
public:
  Init_ArmorTarget_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmorTarget_is_detected header(::cpp08_armor_detector::msg::ArmorTarget::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ArmorTarget_is_detected(msg_);
  }

private:
  ::cpp08_armor_detector::msg::ArmorTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp08_armor_detector::msg::ArmorTarget>()
{
  return cpp08_armor_detector::msg::builder::Init_ArmorTarget_header();
}

}  // namespace cpp08_armor_detector

#endif  // CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__BUILDER_HPP_
