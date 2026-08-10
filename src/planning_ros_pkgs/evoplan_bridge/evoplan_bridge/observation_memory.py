#!/usr/bin/env python3
"""Search the robot's observation memory: what was seen, where.

``observation_logger.py`` writes one JSONL record per log period, each listing the
objects perceived that tick with ``class_name``, ``score``, ``position_map``,
``region`` and ``region_dist``. That file has always been a diagnostic artefact
with no consumer. This module makes it queryable, so an inspection that finds
nothing can ask "where did I actually see this?" and the replan can be told.

Two questions, both answered here:

* **presence** -- is object X at region R *now*? (:func:`observed_in_region`)
* **recall**   -- where has object X ever been seen? (:func:`locate_object`)

Evidence thresholds are the point, not decoration. A live run produced spurious
``kite``, ``snowboard``, ``surfboard`` and ``teddy_bear`` labels, so a single
frame proves nothing. Requiring several detections above a score floor makes a
false negative (harmless: triggers a replan) far more likely than a false
positive (harmful: passes an inspection that should have failed).

Pure stdlib, no ROS, so it is unit-testable directly.
"""

from __future__ import annotations

import json
import math
from collections import defaultdict

__all__ = [
    "load_observations",
    "iter_detections",
    "observed_in_region",
    "locate_object",
    "position_xy",
    "summarize_for_planner",
    "DEFAULT_MIN_SCORE",
    "DEFAULT_MIN_HITS",
]

#: Detection confidence below which a sighting is ignored.
DEFAULT_MIN_SCORE = 0.5
#: Detections of a class in a region needed before it counts as really there.
DEFAULT_MIN_HITS = 3


def load_observations(path):
    """Read an observations JSONL file. Malformed lines are skipped.

    The logger writes continuously and may be mid-write when read, so a
    truncated final line is expected rather than exceptional.
    """
    records = []
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    except OSError:
        return []
    return records


def iter_detections(records, since_ros_sec=None, until_ros_sec=None):
    """Yield ``(record, object)`` pairs, optionally windowed by sim time.

    The window is what makes a *presence* check meaningful: "did I see it while
    I was standing there", not "did I ever see it".
    """
    for record in records:
        stamp = record.get("ros_time_sec")
        if since_ros_sec is not None and (stamp is None or stamp < since_ros_sec):
            continue
        if until_ros_sec is not None and (stamp is None or stamp > until_ros_sec):
            continue
        for obj in record.get("objects") or []:
            yield record, obj


def position_xy(obj):
    """(x, y) of a detection in the map frame, or None if it has no position.

    ``observation_logger.py:490`` writes ``position_map`` as a MAPPING --
    ``{"x": .., "y": .., "z": ..}`` -- and every observations.jsonl on disk uses
    that shape. Sequences are accepted too because hand-written fixtures and
    older logs use ``[x, y]``, and silently disagreeing with either format is
    how this went wrong the first time:

      * ``locate_object`` subscripted ``position_map[0]`` unguarded, so a real
        (dict) log raised ``KeyError: 0`` and killed the executor mid-run, at
        the exact moment the tour ended and the object approach began;
      * ``_region_of`` guarded with ``isinstance(xy, (list, tuple))``, so the
        same dict silently failed the check and fell back to the logged
        ``region`` label -- which meant the snap_max_m cutoff never ran on any
        real log, and a detection 200 m from every region could still be
        attributed to one.

    One accessor for both call sites, so the two can no longer disagree about
    what a position is.
    """
    if obj is None:
        return None
    pos = obj.get("position_map") if hasattr(obj, "get") else None
    if pos is None:
        return None
    if isinstance(pos, dict):
        x, y = pos.get("x"), pos.get("y")
    elif isinstance(pos, (list, tuple)) and len(pos) >= 2:
        x, y = pos[0], pos[1]
    else:
        return None
    try:
        return float(x), float(y)
    except (TypeError, ValueError):
        return None      # a null or non-numeric coordinate is "no position"


def _region_of(obj, regions=None, snap_max_m=6.0):
    """Region for a detection, preferring geometry over the logged label.

    ``observation_logger`` records ``region: null`` whenever TF was unavailable,
    even though ``position_map`` is present and perfectly usable. When a region
    table is supplied, recompute from the map position; regions are points
    rather than polygons, so anything beyond ``snap_max_m`` is left unattributed
    instead of being snapped to a far-away centroid.
    """
    if regions:
        xy = position_xy(obj)
        if xy is not None:
            best, best_d = None, float("inf")
            for name, coords in regions.items():
                d = math.dist(xy, (coords[0], coords[1]))
                if d < best_d:
                    best, best_d = name, d
            if best is not None and best_d <= snap_max_m:
                return best.lower()
            return None
    region = obj.get("region")
    return region.lower() if isinstance(region, str) else None


def observed_in_region(records, class_name, region, since_ros_sec=None,
                       until_ros_sec=None, min_score=DEFAULT_MIN_SCORE,
                       regions=None):
    """Detections of ``class_name`` attributed to ``region`` in the window.

    Returns the matching detection dicts, so the caller can report *how much*
    evidence there was rather than only whether the threshold was met.
    """
    class_name = (class_name or "").lower()
    region = (region or "").lower()
    hits = []
    for _, obj in iter_detections(records, since_ros_sec, until_ros_sec):
        if (obj.get("class_name") or "").lower() != class_name:
            continue
        if float(obj.get("score") or 0.0) < min_score:
            continue
        if _region_of(obj, regions) != region:
            continue
        hits.append(obj)
    return hits


def locate_object(records, class_name, min_score=DEFAULT_MIN_SCORE,
                  min_hits=DEFAULT_MIN_HITS, regions=None):
    """Where has ``class_name`` been seen? Strongest evidence first.

    Returns ``[{region, hits, mean_score, xy}, ...]`` for regions meeting
    ``min_hits``. Empty means "never convincingly seen" -- which is a useful
    answer in itself, and distinct from "seen somewhere else".
    """
    class_name = (class_name or "").lower()
    by_region = defaultdict(list)
    for _, obj in iter_detections(records):
        if (obj.get("class_name") or "").lower() != class_name:
            continue
        if float(obj.get("score") or 0.0) < min_score:
            continue
        region = _region_of(obj, regions)
        if region:
            by_region[region].append(obj)

    out = []
    for region, objs in by_region.items():
        if len(objs) < min_hits:
            continue
        # position_xy tolerates both the dict shape the logger writes and the
        # [x, y] sequence fixtures use; detections with no usable position are
        # dropped from the centroid rather than crashing it.
        xys = [xy for xy in (position_xy(o) for o in objs) if xy is not None]
        out.append({
            "region": region,
            "hits": len(objs),
            "mean_score": round(sum(float(o.get("score") or 0.0) for o in objs) / len(objs), 3),
            "xy": [round(sum(p[0] for p in xys) / len(xys), 2),
                   round(sum(p[1] for p in xys) / len(xys), 2)] if xys else None,
        })
    # Most-seen first; the planner should act on the strongest evidence.
    out.sort(key=lambda r: (-r["hits"], -r["mean_score"]))
    return out


def summarize_for_planner(records, classes=None, min_score=DEFAULT_MIN_SCORE,
                          min_hits=DEFAULT_MIN_HITS, regions=None,
                          exclude_types=("human",)):
    """Compact ``{class_name: [locations]}`` map for the replan request.

    This is what crosses to the host service, so it must stay small: a mission
    can accumulate thousands of detections, and the whole log would swamp both
    the PDDL problem and the LLM prompt. People are excluded by default -- they
    move, so "where a person was" is not a fact worth planning against.
    """
    seen = set()
    for _, obj in iter_detections(records):
        if (obj.get("object_type") or "") in exclude_types:
            continue
        name = (obj.get("class_name") or "").lower()
        if name:
            seen.add(name)
    if classes:
        seen &= {c.lower() for c in classes}

    summary = {}
    for name in sorted(seen):
        found = locate_object(records, name, min_score, min_hits, regions)
        if found:
            summary[name] = found
    return summary
