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
import json
import math
import os
import pathlib
import re
import signal
import subprocess
import sys

import rclpy
import rclpy.utilities
from rclpy.node import Node
from nav_msgs.msg import Path as NavPath
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from evoplan_bridge.envelope import ENVELOPE


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

# Hall proxemic zones (m)
ZONES = (("intim", 0.45), ("pers", 1.2), ("social", 3.6))
PERSONAL = 1.2  # P_int reference distance

# The mined dynamics/TTC envelope now lives in evoplan_bridge.envelope,
# imported above and shared with the online Phi_mob shield.


class _Stop(Exception):
    """Raised from the timer callback to cleanly break rclpy.spin()."""


def is_human(name):
    n = name.lower()
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
        self.goal_src = "none"
        if args.goal:
            gx, gy = (float(v) for v in args.goal.split(","))
            self.goal = (gx, gy)
            self.goal_src = "--goal arg"
        # The goal extracted from the plan: evo publishes the plan's waypoints as
        # a Path; its last pose is the mission's final destination. Subscribing
        # keeps the metric goal aligned with whatever plan evo is executing.
        self._goal_topic = args.goal_topic
        self.stop_on_success = str(args.stop_on_success).strip().lower() in ("1", "true", "yes")
        # A collision ends the trial as a failure (see tick()).
        self.stop_on_collision = str(getattr(args, "stop_on_collision", "true")
                                     ).strip().lower() in ("1", "true", "yes")

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
        self._settled_since = None   # sim-time the robot settled at the goal
        self.path_len = 0.0
        self._path_prev = None
        self.min_human_clr = float("inf")
        self.reached_goal = False
        self.mission_plan_complete = True  # until the planner says otherwise
        # planned waypoint route (from goal-topic Path) for path-progress
        self.path_poly = []          # [(x,y), ...] plan waypoints
        self.path_total = 0.0        # total planned distance, summed across replans
        self.path_leg_total = 0.0    # length of the CURRENT route only
        self.route_completed_m = 0.0 # distance banked from routes a replan replaced
        self.max_progress_arc = 0.0  # furthest arc-length along the route reached
        self.env_ok = {k: 0 for k, _, _ in ENVELOPE}     # ticks the bound held
        self.env_n = {k: 0 for k, _, _ in ENVELOPE}      # ticks the signal was defined
        self.env_ext = {}                                 # extreme observed value

        # Per-tick ground-truth trace (optional). The live metrics collapse every
        # human into the single scalar min_clr, so a specific person can never be
        # excluded after the fact; this keeps the per-human breakdown so the
        # proxemic metrics can be recomputed offline (filter_trace_metrics.py).
        self._last_humans = []      # [(name, x, y, dist), ...] for the current tick
        self.trace_fh = None
        if getattr(args, "trace_out", ""):
            try:
                pathlib.Path(args.trace_out).parent.mkdir(parents=True, exist_ok=True)
                self.trace_fh = open(args.trace_out, "w", buffering=1)  # line-buffered
                self.get_logger().info(f"Per-tick trace -> {args.trace_out}")
            except OSError as exc:
                self.get_logger().warn(f"Could not open trace file {args.trace_out}: {exc}")

        # Scripted walking actors (no gz pose) reconstructed from sim time.
        self.actor_traj = parse_actor_trajectories(args.actors_sdf) if args.actors_sdf else {}
        self.sim_time = 0.0
        # Latest /evoplan/status blob, folded into the results JSON.
        self.evoplan_status = None
        if self.actor_traj:
            self.create_subscription(Clock, "/clock", self.clock_cb, 10)

        if getattr(args, "evoplan_status_topic", None):
            from std_msgs.msg import String as _String
            self.create_subscription(_String, args.evoplan_status_topic,
                                     self.evoplan_status_cb, 10)

        for topic in (f"/world/{args.world}/pose/info",
                      f"/world/{args.world}/dynamic_pose/info"):
            self.create_subscription(TFMessage, topic, self.tf_cb, 50)

        if self._goal_topic:
            self.create_subscription(NavPath, self._goal_topic, self.goal_cb, 10)

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

    def goal_cb(self, msg):
        # last waypoint of the plan's published path = mission goal
        if msg.poses:
            # store the full waypoint polyline for path-progress
            poly = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
            if poly != self.path_poly:
                # An online replan swaps the route mid-mission. Bank the distance
                # already achieved on the route being replaced -- resetting it
                # threw away everything driven before the replan and measured
                # progress only against the (typically much shorter) new route.
                self.route_completed_m += self.max_progress_arc
                # The published route lists waypoint DESTINATIONS only, so the
                # leg from where the robot actually is to the first waypoint is
                # absent. For a single-waypoint replan -- which is what you get
                # whenever the robot is one hop from the goal -- that made the
                # polyline zero-length and Route% undefined. Anchor it at the
                # robot's current pose.
                r = self.poses.get(self.robot_name)
                anchored = ([(r[0], r[1])] if r else []) + poly
                self.path_poly = poly
                self.path_leg_total = sum(
                    math.hypot(anchored[i + 1][0] - anchored[i][0],
                               anchored[i + 1][1] - anchored[i][1])
                    for i in range(len(anchored) - 1)
                )
                self.path_total = self.route_completed_m + self.path_leg_total
                self.max_progress_arc = 0.0
            p = msg.poses[-1].pose.position
            new_goal = (p.x, p.y)
            if self.goal != new_goal or self.goal_src != "plan":
                self.get_logger().info(
                    f"Goal from plan ({self._goal_topic}): "
                    f"({new_goal[0]:.2f}, {new_goal[1]:.2f})"
                )
            self.goal = new_goal
            self.goal_src = "plan"

    def _proj_arc(self, rx, ry):
        """Arc-length along the planned polyline of the point nearest to (rx,ry)."""
        if len(self.path_poly) < 2:
            return 0.0
        best_d = float("inf")
        best_arc = 0.0
        cum = 0.0
        for i in range(len(self.path_poly) - 1):
            ax, ay = self.path_poly[i]
            bx, by = self.path_poly[i + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 <= 1e-9 else max(0.0, min(1.0, ((rx - ax) * dx + (ry - ay) * dy) / seg2))
            cx, cy = ax + t * dx, ay + t * dy
            d = math.hypot(rx - cx, ry - cy)
            seglen = math.sqrt(seg2)
            if d < best_d:
                best_d = d
                best_arc = cum + t * seglen
            cum += seglen
        return best_arc

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

    def evoplan_status_cb(self, msg):
        try:
            self.evoplan_status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

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
        self._last_humans = []
        for name, ox, oy in self.human_positions():
            d = math.hypot(ox - rx, oy - ry)
            self._last_humans.append((name, ox, oy, d))
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

        # Per-tick ground-truth trace: written BEFORE the accumulators fold min_clr
        # into scalars, so the offline recompute sees exactly what this tick saw.
        if self.trace_fh is not None:
            try:
                self.trace_fh.write(json.dumps({
                    "tick": self.ticks,
                    "t": round(self.sim_time, 3),
                    "robot": {
                        "x": round(rx, 4), "y": round(ry, 4), "yaw": round(ryaw, 4),
                        "speed": round(math.hypot(*self.robot_vel), 4),
                    },
                    "humans": [
                        {"name": n, "x": round(hx, 4), "y": round(hy, 4), "d": round(hd, 4)}
                        for n, hx, hy, hd in self._last_humans
                    ],
                }) + "\n")
            except (OSError, ValueError):
                pass

        self.ticks += 1
        # path length
        if self._path_prev is not None:
            self.path_len += math.hypot(rx - self._path_prev[0], ry - self._path_prev[1])
        # path progress: furthest point reached along the planned waypoint route
        if self.path_total > 0.0:
            arc = self._proj_arc(rx, ry)
            if arc > self.max_progress_arc:
                self.max_progress_arc = arc
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
                    # A collision is a terminal failure, not a metric to keep
                    # accumulating: driving on afterwards logs proxemic and
                    # envelope statistics for a run whose outcome is already
                    # decided, and inflates duration_s and path_length_m with
                    # post-failure motion. Stop here and report succ=no.
                    if self.stop_on_collision:
                        self.get_logger().error(
                            f"COLLISION: human clearance {min_clr:.3f} m < "
                            f"{self.coll_r:.3f} m -- ending the trial as a failure"
                        )
                        raise _Stop
            else:
                self._in_collision = False
        # goal
        if self.goal is not None:
            at_goal = math.hypot(rx - self.goal[0], ry - self.goal[1]) <= self.args.goal_tol
            if at_goal:
                self.reached_goal = True
            # "until success" mode: stop & report once the robot has reached the
            # goal AND settled there. Requiring it to be stationary matters:
            # the robot passes within goal_tol while still decelerating, and
            # stopping then truncates duration_s before the mission has actually
            # ended. Settling is judged over a short window so a momentary dip
            # in speed mid-approach does not count.
            # Collision runs stop too (succ still reports 'no' for them):
            # otherwise one proximity onset in a peopled env disarms the early
            # stop, the run idles out the full --duration at the goal, and
            # duration_s records the backstop instead of the actual completion.
            # Position alone is not mission completion: a plan that loops back
            # through its target region (factory_mission_01 revisits R8 at step
            # 2 of 28) would score a two-action run as a success. When the
            # planner reports mission state, require it -- and require it to
            # have seen a plan at all, so a silent planner cannot pass by
            # omission.
            plan_done = True
            if self.evoplan_status is not None and self.evoplan_status.get("regions_required"):
                plan_done = bool(self.evoplan_status.get("mission_complete"))
            self.mission_plan_complete = plan_done

            # The planner has nothing left to drive. Stop rather than idle out
            # the trial timeout: a replan narrowed to (visited <target>) can
            # finish its plan while leaving the mission incomplete, and then
            # neither the positional test nor mission_complete will ever fire.
            # succ stays False in that case -- reported honestly, not hidden.
            if (self.evoplan_status or {}).get("plan_exhausted") and not plan_done:
                self.get_logger().warn(
                    "Planner reports its plan is exhausted but the mission is "
                    "incomplete -- ending the trial as a failure "
                    f"(regions never visited: {(self.evoplan_status or {}).get('regions_remaining')})"
                )
                raise _Stop
            if at_goal and plan_done and self.cur["speed"] <= self.args.goal_settle_speed:
                if self._settled_since is None:
                    self._settled_since = rt
                elif (rt - self._settled_since) >= self.args.goal_settle_s:
                    self.get_logger().info(
                        f"Goal reached and settled: within {self.args.goal_tol:.2f} m "
                        f"and below {self.args.goal_settle_speed:.2f} m/s for "
                        f"{self.args.goal_settle_s:.1f} s -- ending the trial."
                    )
                    if self.stop_on_success:
                        raise _Stop
            else:
                self._settled_since = None
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

        # compute once so both text output and JSON can use the same values
        succ = None
        prog = None
        if self.goal is not None:
            succ = bool(self.reached_goal and self.collisions == 0
                        and getattr(self, "mission_plan_complete", True))
            if self.path_total > 0.0:
                # Progress spans every route the mission used: distance banked
                # from routes replaced by a replan, plus the arc achieved on the
                # current one.
                prog = 100.0 if self.reached_goal else min(
                    100.0, 100.0 * (self.route_completed_m + self.max_progress_arc)
                    / self.path_total)
        mc = self.min_human_clr

        print("\n" + "=" * 70)
        print(f"SCAND-shield runtime metrics  (robot={self.robot_name})")
        print(f"  duration={dur:.1f}s   ticks={self.ticks}   path_length={self.path_len:.2f} m")
        print("-" * 70)
        print("Table-2 style (per-run):")
        print(f"  coll (proximity onsets, < {self.coll_r:.2f} m of a human) : {self.collisions}")
        if self.goal is not None:
            print(f"  goal ({self.goal_src})            : "
                  f"({self.goal[0]:.2f}, {self.goal[1]:.2f})")
            print(f"  succ (reached goal, no collision)                : "
                  f"{'yes' if succ else 'no'}")
            if prog is not None:
                print(f"  path progress (route completed)                  : "
                      f"{prog:.1f}%  ({self.max_progress_arc:.1f}/{self.path_total:.1f} m)")
            else:
                print(f"  path progress (route completed)                  : n/a (no plan path)")
            print(f"  SPL                                              : n/a (needs optimal-path length)")
            print(f"  ct% (collision-terminated)                       : "
                  f"{100.0 if self.collisions and not self.reached_goal else 0.0:.1f}")
        else:
            print("  goal/succ/SPL/ct%        : n/a (no --goal or --goal-topic)")
        print(f"  intim  ticks (< 0.45 m)  : {self.zone_counts['intim']:6d}  ({pct(self.zone_counts['intim']):.1f}%)")
        print(f"  pers   ticks (< 1.20 m)  : {self.zone_counts['pers']:6d}  ({pct(self.zone_counts['pers']):.1f}%)")
        print(f"  social ticks (< 3.60 m)  : {self.zone_counts['social']:6d}  ({pct(self.zone_counts['social']):.1f}%)")
        print(f"  P_int = sum max(0, 1.2 - clr) : {self.p_int:.1f}")
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

        if getattr(self.args, "json_out", None):
            envelope = {}
            for name, op, thr in ENVELOPE:
                ext = self.env_ext.get(name)
                comp = 100.0 * self.env_ok[name] / max(self.env_n[name], 1)
                envelope[name] = {
                    "extreme": round(ext, 4) if (ext is not None and math.isfinite(ext)) else None,
                    "bound": op,
                    "threshold": thr,
                    "compliant_pct": round(comp, 2),
                }
            payload = {
                "node": "scand_metrics",
                "robot": self.robot_name,
                "world": self.args.world,
                "duration_s": round(dur, 3),
                "ticks": self.ticks,
                "path_length_m": round(self.path_len, 3),
                "collisions": self.collisions,
                "success": succ,
                "path_progress_pct": round(prog, 2) if prog is not None else None,
                "path_arc_m": round(self.max_progress_arc, 3),
                "path_total_m": round(self.path_total, 3),
                "route_completed_m": round(self.route_completed_m, 3),
                "path_leg_total_m": round(self.path_leg_total, 3),
                "intim_pct": round(pct(self.zone_counts["intim"]), 3),
                "pers_pct": round(pct(self.zone_counts["pers"]), 3),
                "social_pct": round(pct(self.zone_counts["social"]), 3),
                "p_int": round(self.p_int, 3),
                "min_human_clearance_m": round(mc, 4) if math.isfinite(mc) else None,
                "envelope": envelope,
            }
            # Online EvoPlan accounting. Deliberation is NOT excluded from
            # duration_s on purpose: the Gazebo clock keeps running while the
            # robot holds, so a slow planner costs mission time exactly as it
            # would on hardware. moving_time_s separates "the planner was slow"
            # from "the robot drove badly".
            status = self.evoplan_status or {}
            held = float(status.get("held_s") or 0.0)
            payload.update({
                "symbolic_replans": int(status.get("symbolic_replans") or 0),
                "deliberation_time_s": round(float(status.get("deliberation_s_wall") or 0.0), 3),
                "held_time_s": round(held, 3),
                "moving_time_s": round(max(0.0, dur - held), 3),
                "shield_vetoes": int(status.get("shield_vetoes") or 0),
                "mission_complete": bool(status.get("mission_complete")) if status else None,
                "plan_exhausted": bool(status.get("plan_exhausted")) if status else None,
                "regions_required": status.get("regions_required"),
                "regions_visited": status.get("regions_visited"),
                "regions_remaining": status.get("regions_remaining"),
                "evoplan": status or None,
            })
            out = pathlib.Path(self.args.json_out)
            out.parent.mkdir(parents=True, exist_ok=True)
            tmp = out.with_suffix(".tmp")
            tmp.write_text(json.dumps(payload, indent=2))
            tmp.rename(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--evoplan-status-topic", default=None,
                    help="std_msgs/String JSON from evo_plan_deploy; folded into --json-out "
                         "so replan/deliberation columns land in the results table")
    ap.add_argument("--stop-on-collision", default="true",
                    help="end the trial immediately on a collision and report succ=no "
                         "(default true; set false to keep driving and accumulate metrics)")
    ap.add_argument("--world", default="warehouse")
    ap.add_argument("--ns", default="/j100_0000")
    ap.add_argument("--robot", default=None, help="gz model name (default <ns>/robot)")
    ap.add_argument("--rate", type=float, default=10.0, help="metrics eval rate (Hz)")
    ap.add_argument("--duration", type=float, default=0.0, help="auto-stop after N s (0 = until Ctrl-C)")
    ap.add_argument("--actors-sdf", default="",
                    help="world .sdf to read scripted <actor> walk trajectories from "
                         "(includes the gz-invisible walking people in proxemics)")
    ap.add_argument("--goal", default=None, help="static goal 'X,Y' (world frame) for succ/ct%%")
    ap.add_argument("--goal-topic", default="",
                    help="nav_msgs/Path of the plan's waypoints (e.g. /ppddl_nav2_goals); "
                         "the last pose is used as the goal, aligning succ/ct with the plan")
    ap.add_argument("--goal-tol", type=float, default=0.5,
                    help="distance to the final checkpoint counting as arrived (m)")
    ap.add_argument("--goal-settle-speed", type=float, default=0.05,
                    help="speed below which the robot counts as stopped (m/s)")
    ap.add_argument("--goal-settle-s", type=float, default=2.0,
                    help="seconds it must stay stopped at the goal before ending the trial")
    ap.add_argument("--stop-on-success", default="false",
                    help="true/false: stop and print the summary once the goal is reached "
                         "with no collision (paired with --duration as a backstop)")
    ap.add_argument("--collision-radius", type=float, default=0.3,
                    help="robot-human distance counted as a collision")
    ap.add_argument("--ahead-cos", type=float, default=0.5,
                    help="cos of forward half-cone for TTC (0.5 = 60 deg)")
    ap.add_argument("--no-bridge", action="store_true",
                    help="do not start the gz->ROS pose bridge (assume it already runs)")
    ap.add_argument("--json-out", default="",
                    help="write metrics summary as JSON to this path (written atomically at exit)")
    ap.add_argument("--trace-out", default="",
                    help="append one JSON line per tick to this path: sim time, robot "
                         "pose/speed, and every human's ground-truth position + distance. "
                         "Enables offline recomputation of the proxemic metrics (e.g. "
                         "excluding a scripted actor that walked into a stopped robot); "
                         "see filter_trace_metrics.py")
    # strip --ros-args and everything after so argparse doesn't see ROS remapping flags
    # (needed when launched as a ROS2 node via ros2 run / launch)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv[1:]))

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

    rclpy.init()
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
    if node.trace_fh is not None:
        try:
            node.trace_fh.close()
        except OSError:
            pass
    if rclpy.ok():
        rclpy.shutdown()
    if bridge is not None:
        bridge.terminate()


if __name__ == "__main__":
    main()
