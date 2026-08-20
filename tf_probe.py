"""Reproduce AMCL's message-filter lookup exactly, and report why it fails.

AMCL does not ask for the LATEST transform (which is what tf2_echo shows, and which can
succeed while AMCL still fails). It asks for odom -> <scan frame> AT THE SCAN'S OWN
TIMESTAMP. This subscribes to the same scan topic and performs that same lookup.

Run it with the namespace remaps, or it will listen on the wrong TF topics:

  python3 -u ~/autonomy_stack_ros_humble/tf_probe.py \
      --ros-args -r /tf:=/j100_0612/tf -r /tf_static:=/j100_0612/tf_static

NOTE ON WARM-UP: /tf_static is latched, and a listener needs a moment to receive it. The
first scans after startup will always fail; that is the listener filling, not a fault.
The probe therefore waits for the tree before judging anything.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import tf2_ros

SCAN_TOPIC = "/j100_0612/sensors/lidar2d_0/scan"
WARMUP_S = 8.0        # how long to wait for the static tree to arrive
SAMPLES = 20          # scans to judge once warm


class Probe(Node):
    def __init__(self):
        super().__init__("tf_probe")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        # urg_node publishes best_effort; a reliable subscriber would receive nothing.
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, SCAN_TOPIC, self.cb, qos)
        self.started = time.monotonic()
        self.warm = False
        self.scans = 0
        self.judged = 0
        self.ok = 0
        self.failures = {}

    def tree_ready(self):
        try:
            self.buf.lookup_transform("odom", "lidar2d_0_laser", rclpy.time.Time())
            return True
        except Exception:
            return False

    def cb(self, msg):
        self.scans += 1
        elapsed = time.monotonic() - self.started

        if not self.warm:
            if self.tree_ready():
                self.warm = True
                print(f"tree complete after {elapsed:.1f}s "
                      f"({self.scans} scans seen) — now judging lookups at scan time")
            elif elapsed > WARMUP_S:
                print(f"TREE NEVER COMPLETED in {WARMUP_S:.0f}s: odom <- lidar2d_0_laser "
                      "does not resolve even at LATEST. This is a TF problem, not timing.")
                rclpy.shutdown()
            return

        try:
            self.buf.lookup_transform("odom", msg.header.frame_id, msg.header.stamp)
            self.ok += 1
        except Exception as exc:
            key = f"{type(exc).__name__}: {exc}"
            self.failures[key] = self.failures.get(key, 0) + 1
        self.judged += 1

        if self.judged >= SAMPLES:
            print(f"\nodom <- {msg.header.frame_id} at scan timestamp: "
                  f"{self.ok}/{self.judged} OK")
            for key, count in self.failures.items():
                print(f"  {count}x {key}")
            if self.ok == self.judged:
                print("VERDICT: AMCL's lookup succeeds. TF is not the problem.")
            else:
                print("VERDICT: this is what makes AMCL drop scans.")
            rclpy.shutdown()


rclpy.init()
node = Probe()
try:
    rclpy.spin(node)
except Exception:
    pass
if node.scans == 0:
    print(f"NO SCANS received on {SCAN_TOPIC} — the lidar is not publishing.")
