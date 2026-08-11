#!/usr/bin/env python3
"""Where to stand, which way to look, and where to cut the route.

The arithmetic behind `inspect-object`, kept out of the ROS node for the reason
``observation_memory`` and ``symbolic_replan`` are: it is decidable without a
simulator, and a rule about where the robot points is worth being able to check
in a millisecond rather than a twenty-minute trial.

Pure stdlib, no ROS.
"""

from __future__ import annotations

import math

__all__ = ["inspection_pose", "segment_end", "yaw_error"]


def inspection_pose(region_xy, object_xy, standoff_m: float = 0.0):
    """``(stand_xy, yaw)`` for inspecting ``object_xy`` from ``region_xy``.

    The robot goes to the REGION, not to the object. Region centroids are
    waypoints Nav2 is known to be able to reach; a pose computed close to an
    object can land inside it, behind it, or in an aisle too narrow to turn in.
    ``standoff_m`` slides the pose along the centroid->object line to that
    distance from the object, for missions where the centroid is too far away
    to see anything -- and it is clamped at the region centroid, so a stand-off
    larger than the separation never pushes the robot PAST the thing it is
    inspecting or, worse, onto it.

    The yaw always points at the object, which is the half that cannot come
    from PDDL: the problem says which region the object is in and nothing about
    where in the region it sits.

    ``object_xy`` of ``None`` means the runtime never localised it -- an LLM
    replan can name an object this executor never discovered. The answer is the
    bare region with no heading, so the caller can still drive there and dwell
    rather than skipping the inspection outright.
    """
    rx, ry = float(region_xy[0]), float(region_xy[1])
    if object_xy is None:
        return (rx, ry), None
    ox, oy = float(object_xy[0]), float(object_xy[1])
    span = math.hypot(ox - rx, oy - ry)
    if span <= 1e-6:
        # Standing on the object's own position: any heading is as good as
        # another, and there is no line to slide along.
        return (rx, ry), 0.0
    yaw = math.atan2(oy - ry, ox - rx)
    stand = (rx, ry)
    if standoff_m > 0.0 and span > standoff_m:
        travel = span - standoff_m
        stand = (rx + (ox - rx) / span * travel, ry + (oy - ry) / span * travel)
    return stand, yaw


def segment_end(tasks) -> int | None:
    """Index of the first inspection in ``tasks``, or None if there is none.

    Nav2's ``FollowWaypoints`` drives a list straight through. Its only pause,
    the ``WaitAtWaypoint`` plugin, is a global parameter that would stop the
    robot for the same duration at EVERY waypoint -- so a route with
    inspections has to be cut into goals that each end at one, with the dwell
    happening between goals where the executor is in control.

    None means "drive the whole thing in one goal", which is every route of a
    mission with no inspections: the behaviour that predates all of this.
    """
    for idx, task in enumerate(tasks or ()):
        if (task or {}).get("kind") == "inspect":
            return idx
    return None


def yaw_error(desired: float, current: float) -> float:
    """Signed shortest rotation from ``current`` to ``desired``, in radians.

    Via atan2 of the difference rather than subtraction: a naive difference
    across the +/-pi discontinuity reports nearly a full turn where a few
    degrees would do, and the robot obediently spins the long way round in
    front of the object it is supposed to be looking at.
    """
    return math.atan2(math.sin(desired - current), math.cos(desired - current))
