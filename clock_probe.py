"""Which side is off: the scan stamps, or the TF stream? Measured in one process,
so wall clock, scan stamp and TF stamp are all read against the same clock."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf2_ros

class P(Node):
    def __init__(self):
        super().__init__("clock_probe")
        self.buf = tf2_ros.Buffer(); self.l = tf2_ros.TransformListener(self.buf, self)
        q = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "/j100_0612/sensors/lidar2d_0/scan", self.scan, q)
        self.create_subscription(Odometry, "/j100_0612/platform/odom", self.odom, 10)
        self.last_scan = None; self.last_odom = None; self.n = 0
        self.create_timer(1.0, self.tick)
    def scan(self, m): self.last_scan = m.header.stamp.sec + m.header.stamp.nanosec/1e9
    def odom(self, m): self.last_odom = m.header.stamp.sec + m.header.stamp.nanosec/1e9
    def tick(self):
        now = time.time()
        try:
            t = self.buf.lookup_transform("odom", "base_link", rclpy.time.Time())
            tf_t = t.header.stamp.sec + t.header.stamp.nanosec/1e9
        except Exception:
            tf_t = None
        f = lambda v: f"{v-now:+.3f}" if v else "  n/a"
        print(f"wall={now:.3f} | scan {f(self.last_scan)} | odom_msg {f(self.last_odom)} "
              f"| tf odom->base_link {f(tf_t)}   (offsets in s, relative to wall clock)")
        self.n += 1
        if self.n >= 6: rclpy.shutdown()

rclpy.init(); rclpy.spin(P())
