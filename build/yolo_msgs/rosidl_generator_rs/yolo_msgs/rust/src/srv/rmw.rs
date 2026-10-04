#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "yolo_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__yolo_msgs__srv__SetClasses_Request() -> *const std::ffi::c_void;
}

#[link(name = "yolo_msgs__rosidl_generator_c")]
extern "C" {
    fn yolo_msgs__srv__SetClasses_Request__init(msg: *mut SetClasses_Request) -> bool;
    fn yolo_msgs__srv__SetClasses_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetClasses_Request>, size: usize) -> bool;
    fn yolo_msgs__srv__SetClasses_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetClasses_Request>);
    fn yolo_msgs__srv__SetClasses_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetClasses_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetClasses_Request>) -> bool;
}

// Corresponds to yolo_msgs__srv__SetClasses_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetClasses_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub classes: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

}



impl Default for SetClasses_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !yolo_msgs__srv__SetClasses_Request__init(&mut msg as *mut _) {
        panic!("Call to yolo_msgs__srv__SetClasses_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetClasses_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo_msgs__srv__SetClasses_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo_msgs__srv__SetClasses_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo_msgs__srv__SetClasses_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetClasses_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetClasses_Request where Self: Sized {
  const TYPE_NAME: &'static str = "yolo_msgs/srv/SetClasses_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__yolo_msgs__srv__SetClasses_Request() }
  }
}


#[link(name = "yolo_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__yolo_msgs__srv__SetClasses_Response() -> *const std::ffi::c_void;
}

#[link(name = "yolo_msgs__rosidl_generator_c")]
extern "C" {
    fn yolo_msgs__srv__SetClasses_Response__init(msg: *mut SetClasses_Response) -> bool;
    fn yolo_msgs__srv__SetClasses_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetClasses_Response>, size: usize) -> bool;
    fn yolo_msgs__srv__SetClasses_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetClasses_Response>);
    fn yolo_msgs__srv__SetClasses_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetClasses_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetClasses_Response>) -> bool;
}

// Corresponds to yolo_msgs__srv__SetClasses_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetClasses_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetClasses_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !yolo_msgs__srv__SetClasses_Response__init(&mut msg as *mut _) {
        panic!("Call to yolo_msgs__srv__SetClasses_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetClasses_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo_msgs__srv__SetClasses_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo_msgs__srv__SetClasses_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { yolo_msgs__srv__SetClasses_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetClasses_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetClasses_Response where Self: Sized {
  const TYPE_NAME: &'static str = "yolo_msgs/srv/SetClasses_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__yolo_msgs__srv__SetClasses_Response() }
  }
}






#[link(name = "yolo_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__yolo_msgs__srv__SetClasses() -> *const std::ffi::c_void;
}

// Corresponds to yolo_msgs__srv__SetClasses
#[allow(missing_docs, non_camel_case_types)]
pub struct SetClasses;

impl rosidl_runtime_rs::Service for SetClasses {
    type Request = SetClasses_Request;
    type Response = SetClasses_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__yolo_msgs__srv__SetClasses() }
    }
}


