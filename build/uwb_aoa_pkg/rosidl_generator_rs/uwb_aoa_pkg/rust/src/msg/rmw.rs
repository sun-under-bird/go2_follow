#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "uwb_aoa_pkg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__uwb_aoa_pkg__msg__LibAoaRobotMsg() -> *const std::ffi::c_void;
}

#[link(name = "uwb_aoa_pkg__rosidl_generator_c")]
extern "C" {
    fn uwb_aoa_pkg__msg__LibAoaRobotMsg__init(msg: *mut LibAoaRobotMsg) -> bool;
    fn uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LibAoaRobotMsg>, size: usize) -> bool;
    fn uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LibAoaRobotMsg>);
    fn uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LibAoaRobotMsg>, out_seq: *mut rosidl_runtime_rs::Sequence<LibAoaRobotMsg>) -> bool;
}

// Corresponds to uwb_aoa_pkg__msg__LibAoaRobotMsg
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !uwb_aoa_pkg__msg__LibAoaRobotMsg__init(&mut msg as *mut _) {
        panic!("Call to uwb_aoa_pkg__msg__LibAoaRobotMsg__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LibAoaRobotMsg {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { uwb_aoa_pkg__msg__LibAoaRobotMsg__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LibAoaRobotMsg {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LibAoaRobotMsg where Self: Sized {
  const TYPE_NAME: &'static str = "uwb_aoa_pkg/msg/LibAoaRobotMsg";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__uwb_aoa_pkg__msg__LibAoaRobotMsg() }
  }
}


