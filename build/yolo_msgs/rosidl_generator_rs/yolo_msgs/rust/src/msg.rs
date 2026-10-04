#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to yolo_msgs__msg__Point2D
/// 2D point in pixel coordinates

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Point2D {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,

}



impl Default for Point2D {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Point2D::default())
  }
}

impl rosidl_runtime_rs::Message for Point2D {
  type RmwMsg = super::msg::rmw::Point2D;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        x: msg.x,
        y: msg.y,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      x: msg.x,
      y: msg.y,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      x: msg.x,
      y: msg.y,
    }
  }
}


// Corresponds to yolo_msgs__msg__Vector2
/// 2D size in pixel

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Vector2 {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f64,

}



impl Default for Vector2 {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Vector2::default())
  }
}

impl rosidl_runtime_rs::Message for Vector2 {
  type RmwMsg = super::msg::rmw::Vector2;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        x: msg.x,
        y: msg.y,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      x: msg.x,
      y: msg.y,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      x: msg.x,
      y: msg.y,
    }
  }
}


// Corresponds to yolo_msgs__msg__Pose2D
/// 2D position in pixel coordinates

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Pose2D {

    // This member is not documented.
    #[allow(missing_docs)]
    pub position: super::msg::Point2D,


    // This member is not documented.
    #[allow(missing_docs)]
    pub theta: f64,

}



impl Default for Pose2D {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Pose2D::default())
  }
}

impl rosidl_runtime_rs::Message for Pose2D {
  type RmwMsg = super::msg::rmw::Pose2D;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: super::msg::Point2D::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        theta: msg.theta,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        position: super::msg::Point2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
      theta: msg.theta,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      position: super::msg::Point2D::from_rmw_message(msg.position),
      theta: msg.theta,
    }
  }
}


// Corresponds to yolo_msgs__msg__BoundingBox2D
/// 2D position and orientation of the bounding box center

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BoundingBox2D {

    // This member is not documented.
    #[allow(missing_docs)]
    pub center: super::msg::Pose2D,

    /// total size of the bounding box, in pixels, surrounding the object's center
    pub size: super::msg::Vector2,

}



impl Default for BoundingBox2D {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::BoundingBox2D::default())
  }
}

impl rosidl_runtime_rs::Message for BoundingBox2D {
  type RmwMsg = super::msg::rmw::BoundingBox2D;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        center: super::msg::Pose2D::into_rmw_message(std::borrow::Cow::Owned(msg.center)).into_owned(),
        size: super::msg::Vector2::into_rmw_message(std::borrow::Cow::Owned(msg.size)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        center: super::msg::Pose2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.center)).into_owned(),
        size: super::msg::Vector2::into_rmw_message(std::borrow::Cow::Borrowed(&msg.size)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      center: super::msg::Pose2D::from_rmw_message(msg.center),
      size: super::msg::Vector2::from_rmw_message(msg.size),
    }
  }
}


// Corresponds to yolo_msgs__msg__BoundingBox3D
/// 3D position and orientation of the bounding box center

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BoundingBox3D {

    // This member is not documented.
    #[allow(missing_docs)]
    pub center: geometry_msgs::msg::Pose,

    /// total size of the bounding box, in meters, surrounding the object's center
    pub size: geometry_msgs::msg::Vector3,

    /// frame reference
    pub frame_id: std::string::String,

}



impl Default for BoundingBox3D {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::BoundingBox3D::default())
  }
}

impl rosidl_runtime_rs::Message for BoundingBox3D {
  type RmwMsg = super::msg::rmw::BoundingBox3D;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        center: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.center)).into_owned(),
        size: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.size)).into_owned(),
        frame_id: msg.frame_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        center: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.center)).into_owned(),
        size: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.size)).into_owned(),
        frame_id: msg.frame_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      center: geometry_msgs::msg::Pose::from_rmw_message(msg.center),
      size: geometry_msgs::msg::Vector3::from_rmw_message(msg.size),
      frame_id: msg.frame_id.to_string(),
    }
  }
}


// Corresponds to yolo_msgs__msg__Mask
/// segmentation mask for one instance

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Mask {
    /// size of the original image
    pub height: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub width: i32,

    /// mask data represeted by the points of the border of the mask
    pub data: Vec<super::msg::Point2D>,

}



impl Default for Mask {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Mask::default())
  }
}

impl rosidl_runtime_rs::Message for Mask {
  type RmwMsg = super::msg::rmw::Mask;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        height: msg.height,
        width: msg.width,
        data: msg.data
          .into_iter()
          .map(|elem| super::msg::Point2D::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      height: msg.height,
      width: msg.width,
        data: msg.data
          .iter()
          .map(|elem| super::msg::Point2D::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      height: msg.height,
      width: msg.width,
      data: msg.data
          .into_iter()
          .map(super::msg::Point2D::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to yolo_msgs__msg__KeyPoint2D
/// one keypoint for human pose estimation

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyPoint2D {
    /// id of the keypoint
    pub id: i32,

    /// 2D point in pixels
    pub point: super::msg::Point2D,

    /// conf of the keypoint
    pub score: f64,

}



impl Default for KeyPoint2D {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::KeyPoint2D::default())
  }
}

impl rosidl_runtime_rs::Message for KeyPoint2D {
  type RmwMsg = super::msg::rmw::KeyPoint2D;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        point: super::msg::Point2D::into_rmw_message(std::borrow::Cow::Owned(msg.point)).into_owned(),
        score: msg.score,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
        point: super::msg::Point2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.point)).into_owned(),
      score: msg.score,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      point: super::msg::Point2D::from_rmw_message(msg.point),
      score: msg.score,
    }
  }
}


// Corresponds to yolo_msgs__msg__KeyPoint2DArray
/// represents all the keypoints for human pose estimation

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyPoint2DArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub data: Vec<super::msg::KeyPoint2D>,

}



impl Default for KeyPoint2DArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::KeyPoint2DArray::default())
  }
}

impl rosidl_runtime_rs::Message for KeyPoint2DArray {
  type RmwMsg = super::msg::rmw::KeyPoint2DArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data
          .into_iter()
          .map(|elem| super::msg::KeyPoint2D::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data
          .iter()
          .map(|elem| super::msg::KeyPoint2D::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      data: msg.data
          .into_iter()
          .map(super::msg::KeyPoint2D::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to yolo_msgs__msg__KeyPoint3D
/// one keypoint for human pose estimation

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyPoint3D {
    /// id of the keypoint
    pub id: i32,

    /// 3D point in meters
    pub point: geometry_msgs::msg::Point,

    /// conf of the keypoint
    pub score: f64,

}



impl Default for KeyPoint3D {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::KeyPoint3D::default())
  }
}

impl rosidl_runtime_rs::Message for KeyPoint3D {
  type RmwMsg = super::msg::rmw::KeyPoint3D;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.point)).into_owned(),
        score: msg.score,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
        point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.point)).into_owned(),
      score: msg.score,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      point: geometry_msgs::msg::Point::from_rmw_message(msg.point),
      score: msg.score,
    }
  }
}


// Corresponds to yolo_msgs__msg__KeyPoint3DArray
/// represents all the keypoints for human pose estimation in meters

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct KeyPoint3DArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub data: Vec<super::msg::KeyPoint3D>,

    /// frame reference
    pub frame_id: std::string::String,

}



impl Default for KeyPoint3DArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::KeyPoint3DArray::default())
  }
}

impl rosidl_runtime_rs::Message for KeyPoint3DArray {
  type RmwMsg = super::msg::rmw::KeyPoint3DArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data
          .into_iter()
          .map(|elem| super::msg::KeyPoint3D::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        frame_id: msg.frame_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data
          .iter()
          .map(|elem| super::msg::KeyPoint3D::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        frame_id: msg.frame_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      data: msg.data
          .into_iter()
          .map(super::msg::KeyPoint3D::from_rmw_message)
          .collect(),
      frame_id: msg.frame_id.to_string(),
    }
  }
}


// Corresponds to yolo_msgs__msg__Detection
/// defines a YOLO detection result

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Detection {
    /// class probability
    pub class_id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub class_name: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub score: f64,

    /// ID for tracking
    pub id: std::string::String,

    /// 2D bounding box surrounding the object in pixels
    pub bbox: super::msg::BoundingBox2D,

    /// 3D bounding box surrounding the object in meters
    pub bbox3d: super::msg::BoundingBox3D,

    /// segmentation mask of the detected object
    /// it is only the boundary of the segmented object
    pub mask: super::msg::Mask,

    /// keypoints for human pose estimation
    pub keypoints: super::msg::KeyPoint2DArray,

    /// keypoints for human pose estimation
    pub keypoints3d: super::msg::KeyPoint3DArray,

}



impl Default for Detection {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Detection::default())
  }
}

impl rosidl_runtime_rs::Message for Detection {
  type RmwMsg = super::msg::rmw::Detection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        class_id: msg.class_id,
        class_name: msg.class_name.as_str().into(),
        score: msg.score,
        id: msg.id.as_str().into(),
        bbox: super::msg::BoundingBox2D::into_rmw_message(std::borrow::Cow::Owned(msg.bbox)).into_owned(),
        bbox3d: super::msg::BoundingBox3D::into_rmw_message(std::borrow::Cow::Owned(msg.bbox3d)).into_owned(),
        mask: super::msg::Mask::into_rmw_message(std::borrow::Cow::Owned(msg.mask)).into_owned(),
        keypoints: super::msg::KeyPoint2DArray::into_rmw_message(std::borrow::Cow::Owned(msg.keypoints)).into_owned(),
        keypoints3d: super::msg::KeyPoint3DArray::into_rmw_message(std::borrow::Cow::Owned(msg.keypoints3d)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      class_id: msg.class_id,
        class_name: msg.class_name.as_str().into(),
      score: msg.score,
        id: msg.id.as_str().into(),
        bbox: super::msg::BoundingBox2D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bbox)).into_owned(),
        bbox3d: super::msg::BoundingBox3D::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bbox3d)).into_owned(),
        mask: super::msg::Mask::into_rmw_message(std::borrow::Cow::Borrowed(&msg.mask)).into_owned(),
        keypoints: super::msg::KeyPoint2DArray::into_rmw_message(std::borrow::Cow::Borrowed(&msg.keypoints)).into_owned(),
        keypoints3d: super::msg::KeyPoint3DArray::into_rmw_message(std::borrow::Cow::Borrowed(&msg.keypoints3d)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      class_id: msg.class_id,
      class_name: msg.class_name.to_string(),
      score: msg.score,
      id: msg.id.to_string(),
      bbox: super::msg::BoundingBox2D::from_rmw_message(msg.bbox),
      bbox3d: super::msg::BoundingBox3D::from_rmw_message(msg.bbox3d),
      mask: super::msg::Mask::from_rmw_message(msg.mask),
      keypoints: super::msg::KeyPoint2DArray::from_rmw_message(msg.keypoints),
      keypoints3d: super::msg::KeyPoint3DArray::from_rmw_message(msg.keypoints3d),
    }
  }
}


// Corresponds to yolo_msgs__msg__DetectionArray
/// represents all YOLO detections in one frame

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectionArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detections: Vec<super::msg::Detection>,

}



impl Default for DetectionArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DetectionArray::default())
  }
}

impl rosidl_runtime_rs::Message for DetectionArray {
  type RmwMsg = super::msg::rmw::DetectionArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detections: msg.detections
          .into_iter()
          .map(|elem| super::msg::Detection::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        detections: msg.detections
          .iter()
          .map(|elem| super::msg::Detection::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detections: msg.detections
          .into_iter()
          .map(super::msg::Detection::from_rmw_message)
          .collect(),
    }
  }
}


