#!/usr/bin/env python3
"""scand_metrics.py - runtime social-compliance metrics for the warehouse sim.

Monitors the running Gazebo + Nav2 + evo pipeline and prints metrics in the
style of Table 2 of scand_shield.pdf:

  * Hall proxemic tick-counts   intim (<0.45 m), pers (<1.2 m), social (<3.6 m)
  * P_int  = sum_t max(0, 1.2 - min_human_clearance(t))   (personal-space integral)
  * coll   = collision onsets (robot within --collision-radius of a human)  [proximity-based]
  * succ   = reached --goal without colliding (if --goal given)
  * SPL    = succ * L_opt / max(L_opt, L_travelled)        (if --goal given)
  * path length travelled, min human clearance

...plus the mined dynamics/TTC envelope (mined_constraints.md): for each of
speed, yaw_rate, accel, jerk, lat_accel, ttc, ped_ttc, ped_approach_rate it
reports the extreme observed value and the % of ticks the bound held.

All entity ground-truth poses come from Gazebo via a parameter_bridge on
/world/<world>/{pose,dynamic_pose}/info (started by this script), so proxemics
are measured against ground truth. Robot/human velocities are finite-differenced
from those poses. NOTE: gz <actor> people may not publish on these topics in all
Fortress builds; on startup the script logs the entities it sees so you can
confirm your humans are captured (static person *models* always are). Humans are
recognised by name (person/human/visitor/male/female/randy/walk/actor/pedestrian).

Usage (inside the container, with the sim already running):
  source /opt/ros/humble/setup.bash && source install/setup.bash
  export ROS_LOCALHOST_ONLY=1
  python3 scand_metrics.py --ns /j100_0000 --world warehouse [--goal X,Y] [--duration 120]
Press Ctrl-C to stop and print the summary (or it auto-stops after --duration).
"""

import argparse
import math
import os
import re
import signal
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def parse_actor_trajectories(sdf_path):
    """Parse looping <actor> walk trajectories from a world SDF.

    gz <actor> people publish no poses on any gz topic, but they follow these
    deterministic scripts -> we reconstruct their world position from sim time.
    Returns {actor_name: [(time, x, y), ...]} sorted by time.
    """
    try:
        txt = open(sdf_path).read()
    except OSError:
        return {}
    trajs = {}
    for m in re.finditer(r'<actor\s+name="([^"]+)"(.*?)</actor>', txt, re.S):
        name, body = m.group(1), m.group(2)
        wps = []
        for w in re.finditer(r"<time>\s*([\d.]+)\s*</time>\s*<pose>\s*([-\d.]+)\s+([-\d.]+)",
                             body):
            wps.append((float(w.group(1)), float(w.group(2)), float(w.group(3))))
        if len(wps) >= 2:
            wps.sort()
            trajs[name] = wps
    return trajs


def interp_traj(wps, t):
    """Linear-interpolate a looping waypoint trajectory at time t."""
    period = wps[-1][0]
    if period <= 0:
        return wps[0][1], wps[0][2]
    tt = t % period
    for i in range(len(wps) - 1):
        t0, x0, y0 = wps[i]
        t1, x1, y1 = wps[i + 1]
        if t0 <= tt <= t1:
            f = (tt - t0) / (t1 - t0) if t1 > t0 else 0.0
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
    return wps[-1][1], wps[-1][2]

HUMAN_KEYS = (
    "person", "human", "visitor", "female", "male", "randy", "walk",
    "pedestrian", "actor",
)
# names that are never humans even if they match a key
NON_HUMAN = ("warehouse", "ground_plane", "shelf", "table", "chair", "barrier")

# Hall proxemic zones (m)
ZONES = (("intim", 0.45), ("pers", 1.2), ("social", 3.6))
PERSONAL = 1.2  # P_int reference distance

# mined dynamics/TTC envelope: (name, op, threshold)
ENVELOPE = (
    ("speed", "<=", 2.603),
    ("yaw_rate", "<=", 3.293),
    ("accel", "<=", 3.213),
    ("jerk", "<=", 5.862),
    ("lat_accel", "<=", 2.228),
    ("ttc", ">=", 0.452),
    ("ped_ttc", ">=", 0.422),
    ("ped_approach_rate", "<=", 5.0),
)


class _Stop(Exception):
    """Raised from the timer callback to cleanly break rclpy.spin()."""


def is_human(name):
    n = name.lower()
    if any(k in n for k in NON_HUMAN):
        return False
    return any(k in n for k in HUMAN_KEYS)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScandMetrics(Node):
    def __init__(self, args):
        super().__init__("scand_metrics")
        self.args = args
        self.robot_name = args.robot or f"{args.ns.strip('/')}/robot"
        self.coll_r = args.collision_radius
        self.ahead_cos = args.ahead_cos
        self.goal = None
        if args.goal:
            gx, gy = (float(v) for v in args.goal.split(","))
            self.goal = (gx, gy)

        # latest world poses: name -> (x, y, yaw, t_s)
        self.poses = {}
        self.entities_seen = set()

        # robot motion state
        self._r_prev = None       # (x, y, yaw, t)
        self._r_speed_prev = None  # (speed, accel, t)
        self.robot_vel = (0.0, 0.0)
        self.robot_yaw = 0.0
        self.cur = {k: 0.0 for k, _, _ in ENVELOPE}
        self._h_prev = {}         # human name -> (x, y, t)

        # accumulators
        self.ticks = 0
        self.zone_counts = {z: 0 for z, _ in ZONES}
        self.p_int = 0.0
        self.collisions = 0
        self._in_collision = False
        self.path_len = 0.0
        self._path_prev = None
        self.min_human_clr = float("inf")
        self.reached_goal = False
        self.env_ok = {k: 0 for k, _, _ in ENVELOPE}     # ticks the bound held
        self.env_n = {k: 0 for k, _, _ in ENVELOPE}      # ticks the signal was defined
        self.env_ext = {}                                 # extreme observed value

        # Scripted walking actors (no gz pose) reconstructed from sim time.
        self.actor_traj = parse_actor_trajectories(args.actors_sdf) if args.actors_sdf else {}
        self.sim_time = 0.0
        if self.actor_traj:
            self.create_subscription(Clock, "/clock", self.clock_cb, 10)

        for topic in (f"/world/{args.world}/pose/info",
                      f"/world/{args.world}/dynamic_pose/info"):
            self.create_subscription(TFMessage, topic, self.tf_cb, 50)

        self.create_timer(1.0 / args.rate, self.tick)
        self.start_s = self._now()
        self._announced = False
        self.get_logger().info(
            f"scand_metrics: robot={self.robot_name}, world={args.world}, "
            f"rate={args.rate} Hz, collision_radius={self.coll_r} m"
        )

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def human_positions(self):
        """Combined human (name, x, y) list: gz-published people + scripted actors."""
        humans = [
            (name, ox, oy)
            for name, (ox, oy, _yaw, _t) in list(self.poses.items())
            if name != self.robot_name and is_human(name)
        ]
        for name, wps in self.actor_traj.items():
            x, y = interp_traj(wps, self.sim_time)
            humans.append((name, x, y))
        return humans

    def tf_cb(self, msg):
        t = self._now()
        for tr in msg.transforms:
            name = tr.child_frame_id
            self.poses[name] = (
                tr.transform.translation.x,
                tr.transform.translation.y,
                yaw_of(tr.transform.rotation),
                t,
            )
            self.entities_seen.add(name)

    def update_robot_motion(self, x, y, yaw, t):
        if self._r_prev is not None:
            px, py, pyaw, pt = self._r_prev
            dt = t - pt
            if 1e-3 < dt < 0.5:
                speed = math.hypot(x - px, y - py) / dt
                dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
                self.cur["speed"] = speed
                self.cur["yaw_rate"] = abs(dyaw / dt)
                self.cur["lat_accel"] = abs(speed * dyaw / dt)
                self.robot_yaw = yaw
                self.robot_vel = ((x - px) / dt, (y - py) / dt)
                if self._r_speed_prev is not None:
                    s0, a0, t0 = self._r_speed_prev
                    dts = t - t0
                    if 1e-3 < dts < 0.5:
                        accel = (speed - s0) / dts
                        self.cur["accel"] = abs(accel)
                        self.cur["jerk"] = abs((accel - a0) / dts)
                        self._r_speed_prev = (speed, accel, t)
                    else:
                        self._r_speed_prev = (speed, self.cur["accel"], t)
                else:
                    self._r_speed_prev = (speed, 0.0, t)
        self._r_prev = (x, y, yaw, t)

    def human_metrics(self, rx, ry, t):
        """Return (min_clearance, ttc, ped_approach_rate) over humans."""
        hx_dir, hy_dir = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        vrx, vry = self.robot_vel
        min_clr = float("inf")
        ttc = float("inf")
        approach = 0.0
        best_ahead_d = float("inf")
        for name, ox, oy in self.human_positions():
            d = math.hypot(ox - rx, oy - ry)
            if d < min_clr:
                min_clr = d
            # human velocity (finite diff)
            ovx = ovy = 0.0
            prev = self._h_prev.get(name)
            if prev is not None:
                px, py, pt = prev
                dts = t - pt
                if 1e-3 < dts < 1.0:
                    ovx, ovy = (ox - px) / dts, (oy - py) / dts
            self._h_prev[name] = (ox, oy, t)
            if d < 1e-3:
                continue
            ux, uy = (ox - rx) / d, (oy - ry) / d
            if (hx_dir * ux + hy_dir * uy) < self.ahead_cos:
                continue  # not ahead
            if d < best_ahead_d:
                best_ahead_d = d
                closing = (vrx - ovx) * ux + (vry - ovy) * uy
                approach = max(closing, 0.0)
                ttc = d / closing if closing > 1e-3 else float("inf")
        valid = set(self.poses) | set(self.actor_traj)
        for n in [k for k in self._h_prev if k not in valid]:
            self._h_prev.pop(n, None)
        return min_clr, ttc, approach

    def tick(self):
        if self.args.duration and (self._now() - self.start_s) >= self.args.duration:
            raise _Stop
        r = self.poses.get(self.robot_name)
        if r is None:
            return
        if not self._announced and len(self.entities_seen) > 1:
            self._announced = True
            gz_humans = sorted(n for n in self.entities_seen if is_human(n))
            actors = sorted(self.actor_traj)
            self.get_logger().info(
                f"Tracking {len(gz_humans) + len(actors)} human(s): "
                f"gz-pose={gz_humans or '[]'}; scripted-actors={actors or '[]'}"
            )
        rx, ry, ryaw, rt = r
        self.update_robot_motion(rx, ry, ryaw, rt)

        min_clr, ttc, approach = self.human_metrics(rx, ry, rt)
        self.cur["ttc"] = ttc
        self.cur["ped_ttc"] = ttc
        self.cur["ped_approach_rate"] = approach

        self.ticks += 1
        # path length
        if self._path_prev is not None:
            self.path_len += math.hypot(rx - self._path_prev[0], ry - self._path_prev[1])
        self._path_prev = (rx, ry)
        # proxemics
        if math.isfinite(min_clr):
            self.min_human_clr = min(self.min_human_clr, min_clr)
            for z, thr in ZONES:
                if min_clr < thr:
                    self.zone_counts[z] += 1
            self.p_int += max(0.0, PERSONAL - min_clr)
            if min_clr < self.coll_r:
                if not self._in_collision:
                    self.collisions += 1
                    self._in_collision = True
            else:
                self._in_collision = False
        # goal
        if self.goal is not None:
            if math.hypot(rx - self.goal[0], ry - self.goal[1]) <= self.args.goal_tol:
                self.reached_goal = True
        # envelope compliance
        for name, op, thr in ENVELOPE:
            val = self.cur[name]
            if not math.isfinite(val):
                # inf ttc = trivially compliant lower-bound; skip from extremes
                if op == ">=":
                    self.env_n[name] += 1
                    self.env_ok[name] += 1
                continue
            self.env_n[name] += 1
            ok = (val <= thr) if op == "<=" else (val >= thr)
            if ok:
                self.env_ok[name] += 1
            if op == "<=":
                self.env_ext[name] = max(self.env_ext.get(name, -math.inf), val)
            else:
                self.env_ext[name] = min(self.env_ext.get(name, math.inf), val)

    def print_summary(self):
        dur = self._now() - self.start_s
        n = max(self.ticks, 1)

        def pct(c):
            return 100.0 * c / n

        print("\n" + "=" * 70)
        print(f"SCAND-shield runtime metrics  (robot={self.robot_name})")
        print(f"  duration={dur:.1f}s   ticks={self.ticks}   path_length={self.path_len:.2f} m")
        print("-" * 70)
        print("Table-2 style (per-run):")
        print(f"  coll (proximity onsets, < {self.coll_r:.2f} m of a human) : {self.collisions}")
        if self.goal is not None:
            spl = ""
            if self.reached_goal:
                l_opt = math.hypot(self.goal[0] - (self._path_prev[0] if self._path_prev else 0),
                                   self.goal[1] - (self._path_prev[1] if self._path_prev else 0))
                # L_opt unknown without a planner; report path length only
                spl = "n/a (needs optimal-path length)"
            print(f"  succ (reached goal, no collision)                : "
                  f"{'yes' if (self.reached_goal and self.collisions == 0) else 'no'}")
            print(f"  SPL                                              : {spl or 'n/a'}")
            print(f"  ct% (collision-terminated)                       : "
                  f"{100.0 if self.collisions and not self.reached_goal else 0.0:.1f}")
        print(f"  intim  ticks (< 0.45 m)  : {self.zone_counts['intim']:6d}  ({pct(self.zone_counts['intim']):.1f}%)")
        print(f"  pers   ticks (< 1.20 m)  : {self.zone_counts['pers']:6d}  ({pct(self.zone_counts['pers']):.1f}%)")
        print(f"  social ticks (< 3.60 m)  : {self.zone_counts['social']:6d}  ({pct(self.zone_counts['social']):.1f}%)")
        print(f"  P_int = sum max(0, 1.2 - clr) : {self.p_int:.1f}")
        mc = self.min_human_clr
        print(f"  min human clearance      : {('%.2f m' % mc) if math.isfinite(mc) else 'n/a (no humans seen)'}")
        print("-" * 70)
        print("Mined dynamics/TTC envelope (mined_constraints.md): extreme | %ticks within bound")
        for name, op, thr in ENVELOPE:
            ext = self.env_ext.get(name)
            extt = f"{ext:.3f}" if ext is not None and math.isfinite(ext) else "n/a"
            comp = 100.0 * self.env_ok[name] / max(self.env_n[name], 1)
            kind = "max" if op == "<=" else "min"
            print(f"  {name:18s} {kind}={extt:>8s}  bound {op} {thr:<6.3f}  compliant {comp:5.1f}%")
        print("=" * 70 + "\n")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--ns", default="/j100_0000")
    ap.add_argument("--robot", default=None, help="gz model name (default <ns>/robot)")
    ap.add_argument("--rate", type=float, default=10.0, help="metrics eval rate (Hz)")
    ap.add_argument("--duration", type=float, default=0.0, help="auto-stop after N s (0 = until Ctrl-C)")
    ap.add_argument("--actors-sdf", default="",
                    help="world .sdf to read scripted <actor> walk trajectories from "
                         "(includes the gz-invisible walking people in proxemics)")
    ap.add_argument("--goal", default=None, help="goal as 'X,Y' (world frame) for succ/ct%")
    ap.add_argument("--goal-tol", type=float, default=0.75)
    ap.add_argument("--collision-radius", type=float, default=0.35,
                    help="robot-human distance counted as a collision")
    ap.add_argument("--ahead-cos", type=float, default=0.5,
                    help="cos of forward half-cone for TTC (0.5 = 60 deg)")
    ap.add_argument("--no-bridge", action="store_true",
                    help="do not start the gz->ROS pose bridge (assume it already runs)")
    args = ap.parse_args()

    bridge = None
    if not args.no_bridge:
        cmd = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"/world/{args.world}/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            f"/world/{args.world}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        ]
        env = dict(os.environ)
        # gz-transport must reach the gz server; on multi-NIC hosts it needs to
        # be pinned to loopback (same fix the rest of the pipeline uses).
        env.setdefault("IGN_IP", "127.0.0.1")
        bridge = subprocess.Popen(cmd, env=env,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rclpy.init(args=[])
    node = ScandMetrics(args)

    def _raise(_sig, _frm):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _raise)
    signal.signal(signal.SIGTERM, _raise)

    try:
        rclpy.spin(node)
    except (_Stop, KeyboardInterrupt):
        pass
    except Exception as exc:  # surface unexpected errors but still print what we have
        print(f"scand_metrics error: {exc}", flush=True)
    node.print_summary()
    if rclpy.ok():
        rclpy.shutdown()
    if bridge is not None:
        bridge.terminate()


if __name__ == "__main__":
    main()
