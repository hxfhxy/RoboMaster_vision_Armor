#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "cpp08_armor_detector__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__cpp08_armor_detector__msg__ArmorTarget() -> *const std::ffi::c_void;
}

#[link(name = "cpp08_armor_detector__rosidl_generator_c")]
extern "C" {
    fn cpp08_armor_detector__msg__ArmorTarget__init(msg: *mut ArmorTarget) -> bool;
    fn cpp08_armor_detector__msg__ArmorTarget__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ArmorTarget>, size: usize) -> bool;
    fn cpp08_armor_detector__msg__ArmorTarget__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ArmorTarget>);
    fn cpp08_armor_detector__msg__ArmorTarget__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ArmorTarget>, out_seq: *mut rosidl_runtime_rs::Sequence<ArmorTarget>) -> bool;
}

// Corresponds to cpp08_armor_detector__msg__ArmorTarget
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 装甲板识别结果消息

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmorTarget {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// 是否识别到装甲板
    pub is_detected: bool,

    /// 偏航角（度）
    pub yaw: f32,

    /// 俯仰角（度）
    pub pitch: f32,

    /// 距离（mm）
    pub distance: f32,

    /// 卡尔曼滤波后的偏航角
    pub filtered_yaw: f32,

    /// 装甲板中心像素X
    pub center_x: f32,

    /// 装甲板中心像素Y
    pub center_y: f32,

}



impl Default for ArmorTarget {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !cpp08_armor_detector__msg__ArmorTarget__init(&mut msg as *mut _) {
        panic!("Call to cpp08_armor_detector__msg__ArmorTarget__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ArmorTarget {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cpp08_armor_detector__msg__ArmorTarget__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cpp08_armor_detector__msg__ArmorTarget__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { cpp08_armor_detector__msg__ArmorTarget__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ArmorTarget {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ArmorTarget where Self: Sized {
  const TYPE_NAME: &'static str = "cpp08_armor_detector/msg/ArmorTarget";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__cpp08_armor_detector__msg__ArmorTarget() }
  }
}


