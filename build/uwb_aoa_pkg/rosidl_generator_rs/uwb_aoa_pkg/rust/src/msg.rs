#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to uwb_aoa_pkg__msg__LibAoaRobotMsg

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LibAoaRobotMsg {

    // This member is not documented.
    #[allow(missing_docs)]
    pub r: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub a: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rssi: [i8; 6],


    // This member is not documented.
    #[allow(missing_docs)]
    pub pos_confidence: u8,

}



impl Default for LibAoaRobotMsg {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::LibAoaRobotMsg::default())
  }
}

impl rosidl_runtime_rs::Message for LibAoaRobotMsg {
  type RmwMsg = super::msg::rmw::LibAoaRobotMsg;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        r: msg.r,
        a: msg.a,
        x: msg.x,
        y: msg.y,
        state: msg.state,
        rssi: msg.rssi,
        pos_confidence: msg.pos_confidence,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      r: msg.r,
      a: msg.a,
      x: msg.x,
      y: msg.y,
      state: msg.state,
        rssi: msg.rssi,
      pos_confidence: msg.pos_confidence,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      r: msg.r,
      a: msg.a,
      x: msg.x,
      y: msg.y,
      state: msg.state,
      rssi: msg.rssi,
      pos_confidence: msg.pos_confidence,
    }
  }
}


