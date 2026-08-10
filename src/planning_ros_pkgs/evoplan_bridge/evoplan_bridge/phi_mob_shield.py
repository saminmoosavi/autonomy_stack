#!/usr/bin/env python3
"""Online bounded-horizon robustness monitor for Phi_mob.

Phi_mob is structurally ``AND_i G(sig_i op_i thr_i)`` -- a plain conjunction of
always-bounds, with no nesting and no eventually. That shape makes the whole
monitor a sliding-window minimum:

    rho = min over t in window, over conjuncts i, of margin_i(sig_i(t))

so it needs a deque per signal and nothing else. There is no formula parser here
on purpose. The ``stl_formula_evolve`` half of the merge has a real STL engine,
but its predicates are axis-aligned XY rectangles over ``[T, 2]`` position
traces (``scripts/infer_and_repair_stl.py:build_scenario_from_record`` hard-
raises on anything else), and not one of Phi_mob's conjuncts is a rectangle in
XY -- they are bounds on speed, yaw rate, jerk, TTC. Reusing it would mean
extending the predicate language, the AST, and the PNF pass inside a submodule
this repo does not own, then putting Drake/stlpy on a 10 Hz callback. The cost
of not reusing it is that Phi_mob cannot be *evolved* here -- acceptable,
because it is already mined and frozen in ``mined_constraints.md``.

Signals come from perception, never from Gazebo ground truth. ``scand_metrics``
gets to see through walls because it is an offline scorer; a shield that did the
same would not transfer to hardware, and the disagreement between the two is
itself a useful measurement.

Pure stdlib and free of ROS types so it can be unit-tested directly.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from .envelope import ENVELOPE, VETO_SIGNALS, conjunct_margin

__all__ = ["ShieldResult", "PhiMobShield"]

#: A track must be within this cosine of the robot's heading to count as "ahead"
#: for TTC purposes. 0.5 == a +/-60 degree frontal cone, matching the
#: ``ahead_cos`` default in ``scand_metrics.py``.
AHEAD_COS = 0.5

#: Reject finite-difference steps outside this range. Below it, sensor jitter
#: divided by a tiny dt manufactures enormous fake accelerations; above it, the
#: two samples are too far apart to describe the same motion. Same guard, same
#: bounds as ``scand_metrics.update_robot_motion``.
MIN_DT = 1e-3
MAX_DT = 0.5

#: Human tracks go stale faster than odometry, so they get a looser bound.
MAX_TRACK_DT = 1.0


@dataclass
class ShieldResult:
    """Outcome of one robustness evaluation."""

    #: Minimum normalized margin over the window and all conjuncts. Positive is
    #: satisfied. ``inf`` means no conjunct has enough data to judge yet.
    robustness: float = math.inf
    #: Name of the conjunct attaining that minimum, or ``None``.
    worst_conjunct: str | None = None
    #: Per-conjunct minimum margin over the window, for logging which clause fired.
    margins: dict = field(default_factory=dict)
    #: Latest raw signal values, unnormalized, for the replan request.
    signals: dict = field(default_factory=dict)

    @property
    def satisfied(self) -> bool:
        return self.robustness >= 0.0


class PhiMobShield:
    """Sliding-window Phi_mob robustness from odometry and human tracks.

    Usage per control tick::

        shield.update_odom(x, y, yaw, t)
        shield.update_tracks([(name, hx, hy), ...], t)
        result = shield.evaluate(t)

    ``t`` is seconds and must be *sim* time -- the window length is in sim
    seconds, and mixing in wall time silently rescales the horizon whenever
    Gazebo is not running at 1.0x real time.
    """

    def __init__(self, horizon_s: float = 3.0, envelope=ENVELOPE,
                 veto_signals=VETO_SIGNALS):
        self.horizon_s = float(horizon_s)
        self.envelope = tuple(envelope)
        # Which conjuncts may trigger a veto. Everything in `envelope` is still
        # measured and reported; only these can escalate.
        self.veto_signals = frozenset(veto_signals)
        # (timestamp, value) per signal, trimmed to the horizon on each append.
        self._window: dict[str, deque] = {
            name: deque() for name, _, _ in self.envelope
        }
        self._cur: dict[str, float] = {name: math.inf for name, _, _ in self.envelope}
        # Bounds are upper limits, so an unmeasured signal must start at 0.0 --
        # inf would read as an infinite violation.
        for name, op, _ in self.envelope:
            if op == "<=":
                self._cur[name] = 0.0

        self._prev_pose: tuple[float, float, float, float] | None = None
        self._prev_speed: tuple[float, float, float] | None = None  # (speed, accel, t)
        self._prev_tracks: dict[str, tuple[float, float, float]] = {}
        self._robot_yaw = 0.0
        self._robot_vel = (0.0, 0.0)

    # ------------------------------------------------------------------
    # signal ingestion
    # ------------------------------------------------------------------
    def update_odom(self, x: float, y: float, yaw: float, t: float) -> None:
        """Derive speed, yaw rate, lateral accel, accel and jerk from a pose.

        Finite differences, ported from ``scand_metrics.update_robot_motion``
        so the shield and the offline scorer compute the same quantities from
        different sources.
        """
        prev = self._prev_pose
        self._prev_pose = (x, y, yaw, t)
        if prev is None:
            return
        px, py, pyaw, pt = prev
        dt = t - pt
        if not (MIN_DT < dt < MAX_DT):
            return

        speed = math.hypot(x - px, y - py) / dt
        # Shortest signed angular difference, so wrapping past +/-pi does not
        # register as a huge yaw rate.
        dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
        self._set("speed", speed, t)
        self._set("yaw_rate", abs(dyaw / dt), t)
        self._set("lat_accel", abs(speed * dyaw / dt), t)
        self._robot_yaw = yaw
        self._robot_vel = ((x - px) / dt, (y - py) / dt)

        prev_speed = self._prev_speed
        if prev_speed is None:
            self._prev_speed = (speed, 0.0, t)
            return
        s0, a0, t0 = prev_speed
        dts = t - t0
        if not (MIN_DT < dts < MAX_DT):
            self._prev_speed = (speed, a0, t)
            return
        accel = (speed - s0) / dts
        self._set("accel", abs(accel), t)
        self._set("jerk", abs((accel - a0) / dts), t)
        self._prev_speed = (speed, accel, t)

    def update_tracks(self, tracks, t: float) -> None:
        """Derive TTC and approach rate from human tracks.

        ``tracks`` is an iterable of ``(name, x, y)`` in the same frame as the
        odometry poses. Only tracks inside the frontal cone contribute: a person
        walking away behind the robot is not a collision risk, and counting them
        would make the shield veto constantly in a crowd.

        Ported from ``scand_metrics.human_metrics``, which assigns the same
        value to both ``ttc`` and ``ped_ttc``; that duplication is preserved
        here so the two conjuncts stay comparable between shield and scorer.
        """
        ttc = math.inf
        approach = 0.0
        best_ahead_d = math.inf
        hx_dir, hy_dir = math.cos(self._robot_yaw), math.sin(self._robot_yaw)
        vrx, vry = self._robot_vel
        rx, ry = (self._prev_pose[0], self._prev_pose[1]) if self._prev_pose else (0.0, 0.0)

        seen = set()
        for name, ox, oy in tracks:
            seen.add(name)
            ovx = ovy = 0.0
            prev = self._prev_tracks.get(name)
            if prev is not None:
                pxx, pyy, pt = prev
                dts = t - pt
                if MIN_DT < dts < MAX_TRACK_DT:
                    ovx, ovy = (ox - pxx) / dts, (oy - pyy) / dts
            self._prev_tracks[name] = (ox, oy, t)

            d = math.hypot(ox - rx, oy - ry)
            if d < MIN_DT:
                continue
            ux, uy = (ox - rx) / d, (oy - ry) / d
            if (hx_dir * ux + hy_dir * uy) < AHEAD_COS:
                continue  # behind or beside the robot
            if d < best_ahead_d:
                best_ahead_d = d
                # Component of relative velocity along the line of sight.
                closing = (vrx - ovx) * ux + (vry - ovy) * uy
                approach = max(closing, 0.0)
                ttc = d / closing if closing > MIN_DT else math.inf

        for stale in [k for k in self._prev_tracks if k not in seen]:
            self._prev_tracks.pop(stale, None)

        self._set("ttc", ttc, t)
        self._set("ped_ttc", ttc, t)
        self._set("ped_approach_rate", approach, t)

    # ------------------------------------------------------------------
    # evaluation
    # ------------------------------------------------------------------
    def evaluate(self, t: float) -> ShieldResult:
        """Minimum normalized margin over the horizon ending at ``t``.

        ``margins`` covers every conjunct, so the full envelope stays visible in
        the logs. ``robustness`` and ``worst_conjunct`` are computed over the
        veto subset only -- the signals that describe a social hazard rather
        than ride quality. See ``envelope.VETO_SIGNALS``.
        """
        result = ShieldResult(signals=dict(self._cur))
        worst = math.inf
        worst_name = None
        for name, op, thr in self.envelope:
            window = self._window[name]
            self._trim(window, t)
            if not window:
                continue
            margin = min(conjunct_margin(v, op, thr) for _, v in window)
            result.margins[name] = margin
            if name in self.veto_signals and margin < worst:
                worst = margin
                worst_name = name
        result.robustness = worst
        result.worst_conjunct = worst_name
        return result

    def reset(self) -> None:
        """Drop all history. Use after a plan swap so margins accrued under the
        old plan cannot immediately re-trigger under the new one."""
        for window in self._window.values():
            window.clear()
        self._prev_pose = None
        self._prev_speed = None
        self._prev_tracks.clear()

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------
    def _set(self, name: str, value: float, t: float) -> None:
        self._cur[name] = value
        window = self._window[name]
        window.append((t, value))
        self._trim(window, t)

    def _trim(self, window: deque, t: float) -> None:
        cutoff = t - self.horizon_s
        while window and window[0][0] < cutoff:
            window.popleft()
