#!/usr/bin/env python3
"""Lightweight run recorder: throttled camera JPEGs + a JSONL of planner state.

A full rosbag of the raw camera is ~25 MB/s; this keeps what a replay video
needs (front camera at a few Hz, AMCL pose, Nav2 path, PDDL waypoints, tracked
people, the executor's 1 Hz status blob) in a few hundred MB.

  python3 record_run.py --ns /j100_0000 --out results/recordings/<tag>
"""
import argparse
import json
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection3D


class Recorder(Node):
    def __init__(self, ns, out, fps, cameras):
        super().__init__("run_recorder")
        self.out, self.bridge = out, CvBridge()
        self.period = 1.0 / fps
        self.last_img = {}
        os.makedirs(out, exist_ok=True)
        self.log = open(os.path.join(out, "events.jsonl"), "a", buffering=1)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for cam in cameras:
            os.makedirs(os.path.join(out, f"cam{cam}"), exist_ok=True)
            self.create_subscription(
                Image, f"{ns}/sensors/camera_{cam}/color/image",
                lambda m, c=cam: self.on_image(c, m), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, f"{ns}/amcl_pose", self.on_pose, 10)
        self.create_subscription(NavPath, f"{ns}/plan", lambda m: self.on_path("nav2_plan", m), 10)
        # the executor publishes its waypoints both volatile and latched depending
        # on build; subscribe both ways, duplicates are harmless
        for q in (10, latched):
            self.create_subscription(NavPath, f"{ns}/ppddl_nav2_goals",
                                     lambda m: self.on_path("waypoints", m), q)
            self.create_subscription(NavPath, "ppddl_nav2_goals",
                                     lambda m: self.on_path("waypoints", m), q)
        self.create_subscription(Detection3D, f"{ns}/tracks", self.on_track, 50)
        self.create_subscription(String, f"{ns}/evoplan/status", self.on_status, 10)

    def stamp(self, header=None):
        d = {"wall": time.time()}
        if header is not None:
            d["sim"] = header.stamp.sec + header.stamp.nanosec * 1e-9
        return d

    def write(self, kind, d):
        d["kind"] = kind
        self.log.write(json.dumps(d) + "\n")

    def on_image(self, cam, msg):
        now = time.time()
        if now - self.last_img.get(cam, 0.0) < self.period:
            return
        self.last_img[cam] = now
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        name = f"cam{cam}/{now:.3f}.jpg"
        cv2.imwrite(os.path.join(self.out, name), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        d = self.stamp(msg.header); d["file"] = name
        self.write("image", d)

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        d = self.stamp(msg.header)
        d.update(x=p.x, y=p.y, qz=q.z, qw=q.w)
        self.write("pose", d)

    def on_path(self, kind, msg):
        d = self.stamp(msg.header)
        d["xy"] = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        self.write(kind, d)

    def on_track(self, msg):
        d = self.stamp(msg.header)
        c = msg.bbox.center.position
        d.update(id=msg.id, x=c.x, y=c.y, frame=msg.header.frame_id,
                 cls=(msg.results[0].hypothesis.class_id if msg.results else ""))
        self.write("track", d)

    def on_status(self, msg):
        d = self.stamp()
        try:
            d["status"] = json.loads(msg.data)
        except ValueError:
            d["raw"] = msg.data
        self.write("status", d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="/j100_0000")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=6.0)
    ap.add_argument("--cameras", default="0")
    a = ap.parse_args()
    rclpy.init()
    node = Recorder(a.ns.rstrip("/"), a.out, a.fps, [c for c in a.cameras.split(",") if c])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
