#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to cpp08_armor_detector__msg__ArmorTarget
/// 装甲板识别结果消息

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ArmorTarget {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ArmorTarget::default())
  }
}

impl rosidl_runtime_rs::Message for ArmorTarget {
  type RmwMsg = super::msg::rmw::ArmorTarget;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        is_detected: msg.is_detected,
        yaw: msg.yaw,
        pitch: msg.pitch,
        distance: msg.distance,
        filtered_yaw: msg.filtered_yaw,
        center_x: msg.center_x,
        center_y: msg.center_y,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      is_detected: msg.is_detected,
      yaw: msg.yaw,
      pitch: msg.pitch,
      distance: msg.distance,
      filtered_yaw: msg.filtered_yaw,
      center_x: msg.center_x,
      center_y: msg.center_y,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      is_detected: msg.is_detected,
      yaw: msg.yaw,
      pitch: msg.pitch,
      distance: msg.distance,
      filtered_yaw: msg.filtered_yaw,
      center_x: msg.center_x,
      center_y: msg.center_y,
    }
  }
}


