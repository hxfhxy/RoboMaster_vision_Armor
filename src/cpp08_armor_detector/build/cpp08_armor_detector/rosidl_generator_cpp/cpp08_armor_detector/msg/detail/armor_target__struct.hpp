// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cpp08_armor_detector:msg/ArmorTarget.idl
// generated code does not contain a copyright notice

#ifndef CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__STRUCT_HPP_
#define CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cpp08_armor_detector__msg__ArmorTarget __attribute__((deprecated))
#else
# define DEPRECATED__cpp08_armor_detector__msg__ArmorTarget __declspec(deprecated)
#endif

namespace cpp08_armor_detector
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArmorTarget_
{
  using Type = ArmorTarget_<ContainerAllocator>;

  explicit ArmorTarget_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_detected = false;
      this->yaw = 0.0f;
      this->pitch = 0.0f;
      this->distance = 0.0f;
      this->filtered_yaw = 0.0f;
      this->center_x = 0.0f;
      this->center_y = 0.0f;
    }
  }

  explicit ArmorTarget_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_detected = false;
      this->yaw = 0.0f;
      this->pitch = 0.0f;
      this->distance = 0.0f;
      this->filtered_yaw = 0.0f;
      this->center_x = 0.0f;
      this->center_y = 0.0f;
    }
  }

  // field types and members
  using _is_detected_type =
    bool;
  _is_detected_type is_detected;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _pitch_type =
    float;
  _pitch_type pitch;
  using _distance_type =
    float;
  _distance_type distance;
  using _filtered_yaw_type =
    float;
  _filtered_yaw_type filtered_yaw;
  using _center_x_type =
    float;
  _center_x_type center_x;
  using _center_y_type =
    float;
  _center_y_type center_y;

  // setters for named parameter idiom
  Type & set__is_detected(
    const bool & _arg)
  {
    this->is_detected = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__pitch(
    const float & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__filtered_yaw(
    const float & _arg)
  {
    this->filtered_yaw = _arg;
    return *this;
  }
  Type & set__center_x(
    const float & _arg)
  {
    this->center_x = _arg;
    return *this;
  }
  Type & set__center_y(
    const float & _arg)
  {
    this->center_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator> *;
  using ConstRawPtr =
    const cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cpp08_armor_detector__msg__ArmorTarget
    std::shared_ptr<cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cpp08_armor_detector__msg__ArmorTarget
    std::shared_ptr<cpp08_armor_detector::msg::ArmorTarget_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmorTarget_ & other) const
  {
    if (this->is_detected != other.is_detected) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->filtered_yaw != other.filtered_yaw) {
      return false;
    }
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->center_y != other.center_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmorTarget_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmorTarget_

// alias to use template instance with default allocator
using ArmorTarget =
  cpp08_armor_detector::msg::ArmorTarget_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cpp08_armor_detector

#endif  // CPP08_ARMOR_DETECTOR__MSG__DETAIL__ARMOR_TARGET__STRUCT_HPP_
