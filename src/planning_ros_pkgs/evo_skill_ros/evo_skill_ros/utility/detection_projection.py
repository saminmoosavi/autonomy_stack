#!/usr/bin/env python3
"""Project YOLO 2D detections into a global frame via an organized PointCloud2.

The projection logic here was lifted verbatim from
``evo_skill_ros/nodes/tracker_with_yolo.py`` so that node and
``nodes/observation_logger.py`` compute identical map-frame positions -- the
logger's output is cross-checked against the tracker's ``<ns>/tracks``, which is
only meaningful if both run the same code.

``tracker_with_yolo.py`` still carries its own private copies; it is launched
4x by every experiment harness and feeds STL replanning, so it was left
untouched. Migrate it to this module (a pure-deletion diff) once the logger has
survived a full density_sweep run.

The functions are deliberately free functions taking an explicit ``logger`` and
a ``warned`` set, so they work outside a Node subclass.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2

# PointCloud2 helpers (ROS 2 Python)
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401 - registers geometry_msgs transforms with TF2


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def warn_once(logger, warned: set, key: str, msg: str) -> None:
    if key in warned:
        return
    warned.add(key)
    logger.warn(msg)


def info_once(logger, warned: set, key: str, msg: str) -> None:
    if key in warned:
        return
    warned.add(key)
    logger.info(msg)


def stamp_to_sec(stamp) -> float:
    """builtin_interfaces/Time -> float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# ---------- YOLO field extraction ----------
def extract_bbox_center_px(det) -> Optional[Tuple[int, int]]:
    """Integer (u, v) bbox centre for indexing an organized cloud.

    yolo_msgs Detection bbox formats can vary; try common patterns.
    """
    bbox = getattr(det, "bbox", None)
    if bbox is None:
        return None

    center = getattr(bbox, "center", None)
    if center is not None:
        pos = getattr(center, "position", None)
        if pos is not None and hasattr(pos, "x") and hasattr(pos, "y"):
            return int(round(pos.x)), int(round(pos.y))
        if hasattr(center, "x") and hasattr(center, "y"):
            return int(round(center.x)), int(round(center.y))

    # fallback: xmin/xmax/ymin/ymax
    if all(hasattr(bbox, a) for a in ["xmin", "xmax", "ymin", "ymax"]):
        cx = 0.5 * (float(bbox.xmin) + float(bbox.xmax))
        cy = 0.5 * (float(bbox.ymin) + float(bbox.ymax))
        return int(round(cx)), int(round(cy))

    # fallback: x_offset/y_offset/width/height
    if all(hasattr(bbox, a) for a in ["x_offset", "y_offset", "width", "height"]):
        cx = float(bbox.x_offset) + 0.5 * float(bbox.width)
        cy = float(bbox.y_offset) + 0.5 * float(bbox.height)
        return int(round(cx)), int(round(cy))

    return None


def extract_bbox_px(det) -> Optional[dict]:
    """Sub-pixel 2D bbox as a JSON-ready dict, for the observation record.

    yolo_msgs/BoundingBox2D.size is the TOTAL width/height in pixels (not half),
    so the corners are centre +/- size/2. Kept separate from
    extract_bbox_center_px, which rounds to int for cloud indexing -- rounding
    the logged box would discard information for no reason.
    """
    bbox = getattr(det, "bbox", None)
    if bbox is None:
        return None

    cx = cy = w = h = None

    center = getattr(bbox, "center", None)
    if center is not None:
        pos = getattr(center, "position", None)
        if pos is not None and hasattr(pos, "x") and hasattr(pos, "y"):
            cx, cy = float(pos.x), float(pos.y)
        elif hasattr(center, "x") and hasattr(center, "y"):
            cx, cy = float(center.x), float(center.y)
    size = getattr(bbox, "size", None)
    if size is not None and hasattr(size, "x") and hasattr(size, "y"):
        w, h = float(size.x), float(size.y)

    if cx is None and all(hasattr(bbox, a) for a in ["xmin", "xmax", "ymin", "ymax"]):
        cx = 0.5 * (float(bbox.xmin) + float(bbox.xmax))
        cy = 0.5 * (float(bbox.ymin) + float(bbox.ymax))
        w = abs(float(bbox.xmax) - float(bbox.xmin))
        h = abs(float(bbox.ymax) - float(bbox.ymin))

    if cx is None and all(hasattr(bbox, a) for a in ["x_offset", "y_offset", "width", "height"]):
        w, h = float(bbox.width), float(bbox.height)
        cx = float(bbox.x_offset) + 0.5 * w
        cy = float(bbox.y_offset) + 0.5 * h

    if cx is None or cy is None:
        return None
    if w is None or h is None:
        w = h = 0.0

    return {
        "cx": cx,
        "cy": cy,
        "w": w,
        "h": h,
        "xmin": cx - 0.5 * w,
        "ymin": cy - 0.5 * h,
        "xmax": cx + 0.5 * w,
        "ymax": cy + 0.5 * h,
    }


# ---------- PointCloud2 sampling ----------
def xyz_from_organized_cloud(
    cloud: PointCloud2,
    u: int,
    v: int,
    min_r: float,
    max_r: float,
    logger,
    warned: set,
):
    """Sample the camera-frame XYZ at pixel (u, v) of an organized cloud."""
    w = int(cloud.width)
    h = int(cloud.height)

    if h <= 1:
        warn_once(
            logger,
            warned,
            "unorganized_pointcloud",
            "PointCloud2 appears UNORGANIZED (height<=1). This method needs organized cloud.",
        )
        return None

    u = clamp(u, 0, w - 1)
    v = clamp(v, 0, h - 1)

    # Build numpy dtype from PointCloud2 fields (handles padding/offsets)
    try:
        dtype = pc2.dtype_from_fields(cloud.fields, point_step=cloud.point_step)
        arr = np.frombuffer(cloud.data, dtype=dtype)
    except Exception as e:
        logger.warn(f"Failed creating numpy view of PointCloud2: {e}")
        return None

    # Handle endianness if needed
    if cloud.is_bigendian:
        arr = arr.byteswap().newbyteorder()

    # Reshape to organized image: (height, width)
    try:
        arr = arr.reshape((h, w))
    except Exception as e:
        logger.warn(f"Failed reshaping PointCloud2 to (h,w)=({h},{w}): {e}")
        return None

    # Extract x,y,z (field names must exist)
    if "x" not in arr.dtype.names or "y" not in arr.dtype.names or "z" not in arr.dtype.names:
        warn_once(
            logger,
            warned,
            "missing_xyz_fields",
            f"PointCloud2 missing x/y/z fields. Has: {arr.dtype.names}",
        )
        return None

    x = float(arr["x"][v, u])
    y = float(arr["y"][v, u])
    z = float(arr["z"][v, u])

    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return None

    r = math.sqrt(x * x + y * y + z * z)
    if not (min_r <= r <= max_r):
        return None

    return x, y, z


# ---------- TF ----------
def transform_point_to_target(
    tf_buffer,
    xyz: Tuple[float, float, float],
    src_frame: str,
    stamp,
    target_frame: str,
    logger,
    warned: set,
    timeout_s: float = 0.05,
) -> Optional[Tuple[float, float, float]]:
    """Attempt TF transform src_frame -> target_frame at time stamp.

    If TF is missing, return None so non-global detections are not recorded.
    """
    if not target_frame:
        warn_once(
            logger, warned, "missing_target_frame",
            "No target frame configured; skipping tracked detections.",
        )
        return None
    if not src_frame:
        warn_once(
            logger, warned, "missing_cloud_frame",
            "PointCloud2 header.frame_id is empty; skipping tracked detections.",
        )
        return None
    if target_frame == src_frame:
        return xyz

    pt = PointStamped()
    pt.header.stamp = stamp
    pt.header.frame_id = src_frame
    pt.point.x, pt.point.y, pt.point.z = xyz

    try:
        pt_out = tf_buffer.transform(
            pt, target_frame, timeout=rclpy.duration.Duration(seconds=timeout_s)
        )
        return (pt_out.point.x, pt_out.point.y, pt_out.point.z)
    except TransformException as e:
        try:
            pt.header.stamp = Time().to_msg()
            pt_out = tf_buffer.transform(
                pt, target_frame, timeout=rclpy.duration.Duration(seconds=timeout_s)
            )
            warn_once(
                logger, warned, f"tf_latest_{src_frame}_to_{target_frame}",
                f"TF at detection time unavailable for {src_frame} -> {target_frame}; "
                f"using latest transform.",
            )
            return (pt_out.point.x, pt_out.point.y, pt_out.point.z)
        except TransformException:
            pass
        warn_once(
            logger, warned, f"tf_missing_{src_frame}_to_{target_frame}",
            f"TF missing {src_frame} -> {target_frame}: {e}. Skipping detection.",
        )
        return None


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
