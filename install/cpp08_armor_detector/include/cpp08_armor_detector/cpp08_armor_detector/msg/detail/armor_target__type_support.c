// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cpp08_armor_detector:msg/ArmorTarget.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cpp08_armor_detector/msg/detail/armor_target__rosidl_typesupport_introspection_c.h"
#include "cpp08_armor_detector/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cpp08_armor_detector/msg/detail/armor_target__functions.h"
#include "cpp08_armor_detector/msg/detail/armor_target__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cpp08_armor_detector__msg__ArmorTarget__init(message_memory);
}

void cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_fini_function(void * message_memory)
{
  cpp08_armor_detector__msg__ArmorTarget__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp08_armor_detector__msg__ArmorTarget, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp08_armor_detector__msg__ArmorTarget, is_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp08_armor_detector__msg__ArmorTarget, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp08_armor_detector__msg__ArmorTarget, pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp08_armor_detector__msg__ArmorTarget, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "filtered_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp08_armor_detector__msg__ArmorTarget, filtered_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "center_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp08_armor_detector__msg__ArmorTarget, center_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "center_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp08_armor_detector__msg__ArmorTarget, center_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_members = {
  "cpp08_armor_detector__msg",  // message namespace
  "ArmorTarget",  // message name
  8,  // number of fields
  sizeof(cpp08_armor_detector__msg__ArmorTarget),
  cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_member_array,  // message members
  cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_init_function,  // function to initialize message memory (memory has to be allocated)
  cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_type_support_handle = {
  0,
  &cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cpp08_armor_detector
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cpp08_armor_detector, msg, ArmorTarget)() {
  cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_type_support_handle.typesupport_identifier) {
    cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cpp08_armor_detector__msg__ArmorTarget__rosidl_typesupport_introspection_c__ArmorTarget_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
