#!/usr/bin/env python3
"""Record what the robot observed, and where, for plan/perception desync.

The PDDL plan assumes the world matches problem P: it expects object A at R3.
The robot drives to R3 and perception finds object B instead. Nothing in this
stack persists that fact -- ``eveo_plan_deploy.tracked_objects`` is a transient
obstacle set that expires after ``tracked_object_timeout_s`` (2 s), and
``tracker_with_yolo`` publishes only a bare Detection3D position, discarding the
2D bbox. This node fills that gap.

Two outputs:

* ``observations.jsonl`` -- one appended line per tick: timestamp, map-frame
  robot pose and region, and for each detected object its 2D pixel bbox, its
  map-frame position, and the region it snaps to.
* ``belief.json`` -- cumulative, atomically rewritten: per region, the objects
  the graph EXPECTS there versus the classes actually OBSERVED there, plus
  visit/dwell counts. ``expected_not_observed`` is only meaningful where
  ``visits > 0``, which is what separates "A is not at R3" from "we never
  looked at R3".

IMPORTANT -- this is a periodic SNAPSHOT, not a complete event log. Cameras
publish at ~15 Hz; at the default 1 s period 14 of 15 frames are never
examined. Do not read observations.jsonl as "every object ever seen"; the
belief map is what accumulates across ticks.

Positions come from the same projection code as ``<ns>/tracks``
(utility/detection_projection.py), so the two are directly comparable.

Standalone example (the /tf remaps are mandatory in a namespaced stack):

  ros2 run evo_skill_ros observation_logger --ros-args \\
    -r __ns:=/j100_0000 -r /tf:=tf -r /tf_static:=tf_static \\
    -p use_sim_time:=true -p namespace:=/j100_0000 -p cameras:=0,1,2,3 \\
    -p observations_file:=/tmp/obs.jsonl -p belief_file:=/tmp/belief.json
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from functools import partial
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import PointCloud2
from yolo_msgs.msg import DetectionArray

import tf2_ros
from tf2_ros import TransformException

from evo_skill_ros.utility.detection_projection import (
    extract_bbox_center_px,
    extract_bbox_px,
    quaternion_to_yaw,
    stamp_to_sec,
    transform_point_to_target,
    warn_once,
    xyz_from_organized_cloud,
)
from evo_skill_ros.utility.factory_graph import (
    expected_objects_by_region,
    load_graph,
    nearest_region,
    object_type_from_name,
)

DROP_REASONS = (
    "no_data",
    "desync",
    "no_bbox",
    "bad_depth",
    "no_tf",
    "class_filtered",
    "excluded",
    "low_score",
    "region_out_of_range",
)


def _default_config(name: str) -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("evo_skill_ros")) / "config" / name)
    except Exception:
        return ""


class ObservationLogger(Node):
    def __init__(self) -> None:
        super().__init__("observation_logger")
        self._warned: set = set()

        # -------- Parameters --------
        ns = self._str_param("namespace", "/j100_0000").rstrip("/")
        cameras_raw = self._str_param("cameras", "0")
        tracking_tpl = self._str_param("tracking_topic_template", "/yolo_{cam}/tracking")
        points_tpl = self._str_param("points_topic_template", "/sensors/camera_{cam}/points")

        graph_file = self._str_param("graph_file", _default_config("graph.json"))
        exclude_file = self._str_param("exclude_file", _default_config("exclude.json"))

        log_classes_raw = self._str_param("log_classes", "")
        self._min_score = self._float_param("min_score", 0.0)
        self._log_period_s = self._float_param("log_period_s", 1.0)

        obs_file = self._str_param(
            "observations_file",
            str(Path.home() / "autonomy_stack_ros_humble" / "observations.jsonl"),
        )
        belief_file = self._str_param(
            "belief_file", str(Path.home() / "autonomy_stack_ros_humble" / "belief.json")
        )

        # Object types kept OUT of the expected-vs-observed mismatch (they are
        # still fully logged in observations.jsonl and belief.observed).
        mismatch_ignore_raw = self._str_param("mismatch_ignore_types", "human")

        self._region_snap_max_m = self._float_param("region_snap_max_m", 6.0)
        self._visit_radius_m = self._float_param("visit_radius_m", 2.0)
        self._target_frame = self._str_param("target_frame", "map").strip() or "map"
        self._base_frame = self._str_param("base_frame", "base_link").strip() or "base_link"
        self._detection_timeout_s = self._float_param("detection_timeout_s", 1.0)
        self._sync_slop_s = self._float_param("sync_slop_s", 0.15)
        self._min_r = self._float_param("min_range_m", 0.1)
        self._max_r = self._float_param("max_range_m", 50.0)
        self._points_qos_depth = int(self._float_param("points_qos_depth", 1.0))

        self._cameras = [int(c) for c in cameras_raw.replace(" ", "").split(",") if c != ""]

        # Class allowlist: empty = log everything. yolo-world only ever emits the
        # classes handed to set_classes, so this is a second-stage filter; a
        # typo'd entry must not be able to silently empty the file.
        tokens = [t.strip().lower() for t in log_classes_raw.split(",") if t.strip()]
        self._class_allow = {t for t in tokens}
        self._class_allow_ids = {int(t) for t in tokens if t.lstrip("-").isdigit()}

        # Class denylist from exclude.json. Deny wins over allow.
        self._exclude_file = exclude_file
        self._class_exclude = self._load_exclude(exclude_file)

        self._mismatch_ignore = {
            t.strip().lower() for t in mismatch_ignore_raw.split(",") if t.strip()
        }

        # -------- Graph --------
        self._regions: Dict[str, Tuple[float, float]] = {}
        self._expected: Dict[str, Dict[str, list]] = {}
        self._graph_file = graph_file
        if graph_file:
            try:
                _, self._regions, objects = load_graph(graph_file)
                self._expected = expected_objects_by_region(
                    self._regions, objects, self._region_snap_max_m
                )
                self._prune_expected_for_exclusions()
                self.get_logger().info(
                    f"Loaded graph {graph_file}: {len(self._regions)} regions, "
                    f"{len(objects)} objects"
                )
            except Exception as exc:
                self.get_logger().error(
                    f"Could not load graph_file {graph_file}: {exc}. "
                    f"Regions will be null and belief.json will stay empty."
                )
        else:
            self.get_logger().error(
                "No graph_file resolved; regions will be null and belief.json will stay empty."
            )

        # -------- State --------
        self._det: Dict[int, Optional[Tuple[DetectionArray, float]]] = {c: None for c in self._cameras}
        self._cloud: Dict[int, Optional[Tuple[PointCloud2, float]]] = {c: None for c in self._cameras}
        self._tracking_topics: Dict[int, str] = {}
        self._seq = 0
        self._belief = self._init_belief()
        self._belief_dirty = True
        self._inside_region: Optional[str] = None
        self._last_tick_ros: Optional[float] = None
        self._file_closed = False
        self._obs_failed = False
        self._belief_failed = False

        # -------- Output files --------
        self._obs_path = Path(obs_file).expanduser()
        self._belief_path = Path(belief_file).expanduser()
        self._fh = None
        try:
            self._obs_path.parent.mkdir(parents=True, exist_ok=True)
            self._fh = open(self._obs_path, "a", encoding="utf-8")
        except OSError as exc:
            self._obs_failed = True
            self.get_logger().warn(f"Could not open observation log {self._obs_path}: {exc}")

        # -------- QoS --------
        qos_det = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        qos_cloud = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, self._points_qos_depth),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        # -------- TF --------
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # -------- Subs --------
        for cam in self._cameras:
            t_topic = tracking_tpl.format(cam=cam) if "{cam}" in tracking_tpl else tracking_tpl
            p_topic = ns + (points_tpl.format(cam=cam) if "{cam}" in points_tpl else points_tpl)
            self._tracking_topics[cam] = t_topic
            self.create_subscription(
                DetectionArray, t_topic, partial(self._det_cb, cam), qos_det
            )
            self.create_subscription(
                PointCloud2, p_topic, partial(self._cloud_cb, cam), qos_cloud
            )
            self.get_logger().info(f"  cam{cam}: {t_topic} + {p_topic}")

        self._timer = self.create_timer(self._log_period_s, self._on_tick)

        self.get_logger().info(
            f"Observation logger up.\n"
            f"  namespace:    {ns}\n"
            f"  cameras:      {self._cameras}\n"
            f"  classes:      {sorted(self._class_allow) or '<all>'}\n"
            f"  excluded:     {sorted(self._class_exclude) or '<none>'}\n"
            f"  no-mismatch:  {sorted(self._mismatch_ignore) or '<none>'}\n"
            f"  period:       {self._log_period_s} s (SIM seconds when use_sim_time is set)\n"
            f"  frame:        {self._target_frame} <- {self._base_frame}\n"
            f"  observations: {self._obs_path}\n"
            f"  belief:       {self._belief_path}"
        )

    # ---------- parameters ----------
    # Declared with dynamic_typing so YAML type inference cannot reject a valid
    # value: `-p cameras:=0` infers INTEGER while `-p cameras:=0,1` infers
    # STRING, and a statically-typed declaration rejects the former. Same trap
    # for `-p min_score:=0` (INTEGER) against a DOUBLE. We coerce here instead.
    _DYNAMIC = ParameterDescriptor(dynamic_typing=True)

    def _str_param(self, name: str, default: str) -> str:
        value = self.declare_parameter(name, default, self._DYNAMIC).value
        return "" if value is None else str(value)

    def _float_param(self, name: str, default: float) -> float:
        value = self.declare_parameter(name, default, self._DYNAMIC).value
        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Parameter '{name}'={value!r} is not a number; using {default}."
            )
            return default

    # ---------- exclusions ----------
    def _load_exclude(self, path: str) -> set:
        """Load the class denylist. The file is OPTIONAL; missing is not an error.

        Accepts {"classes": [...]} or a bare [...] array. A malformed file IS an
        error -- silently logging everything would misrepresent the artifacts.
        """
        if not path:
            return set()
        p = Path(path).expanduser()
        if not p.exists():
            self.get_logger().info(f"No exclude file at {p}; logging all detected classes.")
            return set()
        try:
            data = json.loads(p.read_text())
        except (OSError, ValueError) as exc:
            self.get_logger().error(
                f"Could not parse exclude_file {p}: {exc}. Excluding nothing."
            )
            return set()

        if isinstance(data, list):
            raw = data
        elif isinstance(data, dict):
            raw = data.get("classes", [])
        else:
            self.get_logger().error(
                f"exclude_file {p} must be an object or an array, got {type(data).__name__}. "
                f"Excluding nothing."
            )
            return set()
        if not isinstance(raw, list):
            self.get_logger().error(f"exclude_file {p}: 'classes' must be a list. Excluding nothing.")
            return set()
        return {str(c).strip().lower() for c in raw if str(c).strip()}

    def _prune_expected_for_exclusions(self) -> None:
        """Drop excluded types from every region's expected set.

        `expected` comes from graph.json, independent of perception. Excluding
        'shelf' while the graph still expects a shelf in R3 would make every
        visited region report expected_not_observed=['shelf'] -- a fabricated
        desync, which is exactly the signal this logger exists to make
        trustworthy.
        """
        if not self._class_exclude:
            return
        excluded_types = {object_type_from_name(c) for c in self._class_exclude}
        for by_type in self._expected.values():
            for t in excluded_types & set(by_type):
                del by_type[t]

    # ---------- belief ----------
    def _init_belief(self) -> Dict[str, dict]:
        belief = {}
        for name, coords in self._regions.items():
            belief[name] = {
                "coords": [float(coords[0]), float(coords[1])],
                "expected": self._expected.get(name, {}),
                "observed": {},
                "visits": 0,
                "dwell_s": 0.0,
                "last_visit_ros": None,
            }
        return belief

    # ---------- callbacks (cache only; all work happens in the tick) ----------
    def _det_cb(self, cam: int, msg: DetectionArray) -> None:
        self._det[cam] = (msg, stamp_to_sec(msg.header.stamp))

    def _cloud_cb(self, cam: int, msg: PointCloud2) -> None:
        self._cloud[cam] = (msg, stamp_to_sec(msg.header.stamp))

    # ---------- helpers ----------
    def _class_excluded(self, det) -> bool:
        if not self._class_exclude:
            return False
        return str(getattr(det, "class_name", "")).strip().lower() in self._class_exclude

    def _class_allowed(self, det) -> bool:
        if not self._class_allow:
            return True
        name = str(getattr(det, "class_name", "")).strip().lower()
        if name in self._class_allow:
            return True
        try:
            return int(getattr(det, "class_id", -1)) in self._class_allow_ids
        except Exception:
            return False

    def _snap_region(self, xy) -> Tuple[Optional[str], Optional[float]]:
        if not self._regions:
            return None, None
        region, distance = nearest_region(self._regions, xy)
        if distance > self._region_snap_max_m:
            return None, distance
        return region, distance

    def _lookup_robot_pose(self) -> Tuple[Optional[dict], Optional[str]]:
        # Time() = LATEST available, deliberately not the tick time: map->odom
        # from AMCL runs ~2 Hz and frequently would not cover the tick instant.
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame, self._base_frame, Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException as exc:
            warn_once(
                self.get_logger(), self._warned, "robot_tf",
                f"TF {self._target_frame} -> {self._base_frame} unavailable: {exc}. "
                f"Logging robot: null until it appears.",
            )
            return None, str(exc)

        t = tf.transform.translation
        q = tf.transform.rotation
        return (
            {
                "x": float(t.x), "y": float(t.y), "z": float(t.z),
                "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
                "yaw": quaternion_to_yaw(q),
                "tf_stamp_sec": stamp_to_sec(tf.header.stamp),
            },
            None,
        )

    # ---------- tick ----------
    def _on_tick(self) -> None:
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            # No /clock yet under use_sim_time. Returning here also prevents a
            # burst of ticks when the sim clock first jumps.
            return
        now_sec = now.nanoseconds / 1e9

        robot, robot_err = self._lookup_robot_pose()
        dropped = {reason: 0 for reason in DROP_REASONS}
        stale_camera = []
        objects = []

        robot_region = robot_region_dist = None
        if robot is not None:
            robot_region, robot_region_dist = self._snap_region((robot["x"], robot["y"]))
            self._update_visits(robot, now_sec)

        for cam in self._cameras:
            det_entry = self._det.get(cam)
            cloud_entry = self._cloud.get(cam)
            if det_entry is None or cloud_entry is None:
                dropped["no_data"] += 1
                continue
            det_msg, t_det = det_entry
            cloud_msg, t_cloud = cloud_entry

            # Stale caches are skipped, never cleared: a dead camera then shows
            # up on every line instead of looking like one that never published.
            if (now_sec - t_det) > self._detection_timeout_s or (
                now_sec - t_cloud
            ) > self._detection_timeout_s:
                stale_camera.append(cam)
                continue
            # The main correctness guard: never project a pixel from time A into
            # depth from time B.
            if abs(t_det - t_cloud) > self._sync_slop_s:
                dropped["desync"] += 1
                continue

            for det in det_msg.detections:
                # Deny wins over allow: a class in both lists is excluded.
                if self._class_excluded(det):
                    dropped["excluded"] += 1
                    continue
                if not self._class_allowed(det):
                    dropped["class_filtered"] += 1
                    continue
                if float(getattr(det, "score", 0.0)) < self._min_score:
                    dropped["low_score"] += 1
                    continue

                bbox = extract_bbox_px(det)
                uv = extract_bbox_center_px(det)
                if bbox is None or uv is None:
                    dropped["no_bbox"] += 1
                    continue

                xyz_cam = xyz_from_organized_cloud(
                    cloud_msg, uv[0], uv[1], self._min_r, self._max_r,
                    self.get_logger(), self._warned,
                )
                if xyz_cam is None:
                    dropped["bad_depth"] += 1
                    continue

                # Cloud frame + cloud stamp, matching tracker_with_yolo, so these
                # positions stay comparable with <ns>/tracks.
                xyz_map = transform_point_to_target(
                    self._tf_buffer, xyz_cam, cloud_msg.header.frame_id,
                    cloud_msg.header.stamp, self._target_frame,
                    self.get_logger(), self._warned,
                )
                if xyz_map is None:
                    dropped["no_tf"] += 1
                    continue

                region, region_dist = self._snap_region((xyz_map[0], xyz_map[1]))
                if region is None and region_dist is not None:
                    dropped["region_out_of_range"] += 1

                class_name = str(getattr(det, "class_name", ""))
                track_id = str(getattr(det, "id", ""))
                record = {
                    "camera": cam,
                    "tracking_topic": self._tracking_topics[cam],
                    "uid": f"{cam}:{track_id}",
                    "track_id": track_id,
                    "class_id": int(getattr(det, "class_id", -1)),
                    "class_name": class_name,
                    "object_type": object_type_from_name(class_name),
                    "score": float(getattr(det, "score", 0.0)),
                    "det_stamp_sec": t_det,
                    "cloud_stamp_sec": t_cloud,
                    "cloud_frame": cloud_msg.header.frame_id,
                    "bbox_px": bbox,
                    "position_map": {"x": xyz_map[0], "y": xyz_map[1], "z": xyz_map[2]},
                    "position_cam": {"x": xyz_cam[0], "y": xyz_cam[1], "z": xyz_cam[2]},
                    "range_m": math.sqrt(sum(v * v for v in xyz_cam)),
                    "region": region,
                    "region_dist": region_dist,
                }
                objects.append(record)
                self._fold_into_belief(record, now_sec)

        self._seq += 1
        self._write_observation(
            {
                "seq": self._seq,
                "wall_time": datetime.now(timezone.utc).isoformat(),
                "ros_time_sec": now_sec,
                "use_sim_time": bool(self.get_parameter("use_sim_time").value),
                "log_period_s": self._log_period_s,
                "frame": self._target_frame,
                "robot": robot,
                "robot_region": robot_region,
                "robot_region_dist": robot_region_dist,
                "robot_tf_error": robot_err,
                "n_objects": len(objects),
                "objects": objects,
                "dropped": dropped,
                "stale_camera": sorted(set(stale_camera)),
            }
        )
        self._write_belief(now_sec)
        self._last_tick_ros = now_sec

    def _update_visits(self, robot: dict, now_sec: float) -> None:
        """Edge-triggered visit counting + dwell accumulation."""
        inside = None
        if self._regions:
            region, distance = nearest_region(self._regions, (robot["x"], robot["y"]))
            if distance <= self._visit_radius_m:
                inside = region

        if inside is not None and inside != self._inside_region:
            self._belief[inside]["visits"] += 1
            self._belief_dirty = True
        if inside is not None:
            entry = self._belief[inside]
            if self._last_tick_ros is not None and self._inside_region == inside:
                entry["dwell_s"] += max(0.0, now_sec - self._last_tick_ros)
            entry["last_visit_ros"] = now_sec
            self._belief_dirty = True
        self._inside_region = inside

    def _fold_into_belief(self, record: dict, now_sec: float) -> None:
        region = record["region"]
        if region is None or region not in self._belief:
            return
        observed = self._belief[region]["observed"]
        key = record["class_name"] or record["object_type"]
        entry = observed.get(key)
        px, py = record["position_map"]["x"], record["position_map"]["y"]
        if entry is None:
            observed[key] = {
                "object_type": record["object_type"],
                "n": 1,
                "first_ros": now_sec,
                "last_ros": now_sec,
                "centroid": [px, py],
                "max_score": record["score"],
                "uids": [record["uid"]],
            }
        else:
            n = entry["n"] + 1
            entry["n"] = n
            entry["last_ros"] = now_sec
            # running mean, so a class seen 200x yields one stable coordinate
            entry["centroid"][0] += (px - entry["centroid"][0]) / n
            entry["centroid"][1] += (py - entry["centroid"][1]) / n
            entry["max_score"] = max(entry["max_score"], record["score"])
            if record["uid"] not in entry["uids"]:
                entry["uids"].append(record["uid"])
        self._belief_dirty = True

    # ---------- output ----------
    def _write_observation(self, record: dict) -> None:
        if self._fh is None or self._obs_failed:
            return
        try:
            self._fh.write(json.dumps(record, separators=(",", ":")) + "\n")
            self._fh.flush()
        except OSError as exc:
            if not self._obs_failed:
                self._obs_failed = True
                self.get_logger().warn(f"Could not write {self._obs_path}: {exc}")

    def _write_belief(self, now_sec: float) -> None:
        if not self._belief_dirty or self._belief_failed:
            return
        regions_out = {}
        for name, entry in self._belief.items():
            # Dynamic agents are excluded from BOTH sides of the comparison.
            # graph.json describes static map furniture and contains no people,
            # so every pedestrian would otherwise be structurally guaranteed to
            # report as observed_not_expected -- noise that swamps the real
            # "plan expected A here, perception saw B" signal.
            expected_types = set(entry["expected"]) - self._mismatch_ignore
            observed_types = {
                o["object_type"] for o in entry["observed"].values()
            } - self._mismatch_ignore
            # Only meaningful where we actually went and looked; computing it for
            # unvisited regions would manufacture false mismatches.
            missing = sorted(expected_types - observed_types) if entry["visits"] > 0 else []
            regions_out[name] = {
                **entry,
                "expected_not_observed": missing,
                "observed_not_expected": sorted(observed_types - expected_types),
            }
        payload = {
            "graph_file": self._graph_file,
            "exclude_file": self._exclude_file,
            # so a reader can tell "never seen" from "suppressed"
            "excluded_classes": sorted(self._class_exclude),
            # types present in `observed` but deliberately not counted as mismatches
            "mismatch_ignore_types": sorted(self._mismatch_ignore),
            "frame": self._target_frame,
            "updated_ros_sec": now_sec,
            "updated_wall": datetime.now(timezone.utc).isoformat(),
            "visit_radius_m": self._visit_radius_m,
            "region_snap_max_m": self._region_snap_max_m,
            "regions": regions_out,
        }
        # Atomic tmp+rename (scand_metrics idiom): a consumer polling this file
        # must never read a partial document.
        try:
            self._belief_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._belief_path.with_suffix(self._belief_path.suffix + ".tmp")
            tmp.write_text(json.dumps(payload, indent=2))
            tmp.rename(self._belief_path)
            self._belief_dirty = False
        except OSError as exc:
            if not self._belief_failed:
                self._belief_failed = True
                self.get_logger().warn(f"Could not write {self._belief_path}: {exc}")

    def close_file(self) -> None:
        if self._file_closed:
            return
        self._file_closed = True
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError:
                pass


def main() -> None:
    rclpy.init()
    node = ObservationLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # run_sim.sh cleanup() SIGTERMs its children; without catching
        # ExternalShutdownException that dumps a traceback into evo.log.
        pass
    finally:
        node.close_file()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
