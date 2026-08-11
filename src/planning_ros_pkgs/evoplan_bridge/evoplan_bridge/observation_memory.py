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
    "cluster_detections",
    "position_xy",
    "summarize_for_planner",
    "DEFAULT_MIN_SCORE",
    "DEFAULT_MIN_HITS",
    "DEFAULT_LINK_RADIUS_M",
]

#: Detection confidence below which a sighting is ignored.
DEFAULT_MIN_SCORE = 0.5
#: Detections of a class in a region needed before it counts as really there.
DEFAULT_MIN_HITS = 3
#: How far apart two detections of the same class can be and still be believed
#: to be the same physical object.
#:
#: Was 1.5 m, sized for the jitter of repeated YOLO+depth fixes on one
#: stationary object. A live run showed that estimate to be far too tight: one
#: traffic cone produced two clusters 2.53 m apart -- a 6-hit group at
#: (0.43, -12.32) alongside the real 63-hit group at (2.76, -11.33) -- so the
#: robot minted a phantom `traffic_cone_3`, drove to it and held a 5 s
#: inspection of empty floor. Depth projection at these ranges scatters
#: detections much further than the sensor's own noise suggests.
#:
#: 5.0 m absorbs that scatter. The cost is explicit and worth stating: two
#: genuinely distinct objects of the same class closer than 5 m are now ONE
#: object, inspected once. In this factory the regions are 7-8 m apart and the
#: searchable objects sit one per region, so nothing real is lost -- but a world
#: that places two cones side by side needs this lowered, and the mission would
#: silently under-count until it was.
DEFAULT_LINK_RADIUS_M = 5.0


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


def cluster_detections(records, classes=None, min_score=DEFAULT_MIN_SCORE,
                       min_hits=DEFAULT_MIN_HITS,
                       link_radius_m=DEFAULT_LINK_RADIUS_M, regions=None,
                       snap_max_m=6.0):
    """Group detections into ONE ENTRY PER PHYSICAL OBJECT, strongest first.

    :func:`locate_object` answers "which region holds a cone", which is all a
    mission needs when it is hunting a single named object. A find-and-inspect
    mission needs the question this function answers instead: *how many* cones
    are there and where is each one. Two cones five metres apart in the same
    region are one entry to ``locate_object`` -- with a centroid in the empty
    space between them, which is worse than useless as a thing to drive to and
    face -- and two entries here.

    Clustering is single-pass and greedy: detections are walked in log order and
    each joins the first cluster whose running centroid is within
    ``link_radius_m``, or starts a new one. Not k-means, and deliberately not --
    the number of clusters is the unknown being solved for, and the log order is
    a genuine signal (consecutive frames of the same object arrive together), so
    a greedy pass is both cheaper and more stable across the repeated calls this
    is subjected to: it runs on a timer while the robot drives, and a clustering
    that reshuffles identities between polls would rename objects mid-mission.

    Returns ``[{class_name, xy, region, hits, mean_score}, ...]``. Clusters below
    ``min_hits`` are dropped, exactly as ``locate_object`` drops thin evidence --
    a live run produced spurious ``kite`` and ``teddy_bear`` labels, and here a
    false positive costs a wasted drive plus a five-second stare at nothing.
    Clusters whose centroid is further than ``snap_max_m`` from every region are
    dropped too: with no region to name there is no ``(object-at ...)`` to
    assert and nothing the planner could do with it.
    """
    wanted = {c.lower() for c in classes} if classes else None
    clusters = []          # [{class_name, xs, ys, scores}]
    for _, obj in iter_detections(records):
        name = (obj.get("class_name") or "").lower()
        if not name or (wanted is not None and name not in wanted):
            continue
        score = float(obj.get("score") or 0.0)
        if score < min_score:
            continue
        xy = position_xy(obj)
        if xy is None:
            continue      # no position, so nothing to cluster it by
        for cluster in clusters:
            if cluster["class_name"] != name:
                continue
            cx = sum(cluster["xs"]) / len(cluster["xs"])
            cy = sum(cluster["ys"]) / len(cluster["ys"])
            if math.dist(xy, (cx, cy)) <= link_radius_m:
                cluster["xs"].append(xy[0])
                cluster["ys"].append(xy[1])
                cluster["scores"].append(score)
                break
        else:
            clusters.append({"class_name": name, "xs": [xy[0]], "ys": [xy[1]],
                             "scores": [score]})

    out = []
    for cluster in clusters:
        hits = len(cluster["xs"])
        if hits < min_hits:
            continue
        cx = sum(cluster["xs"]) / hits
        cy = sum(cluster["ys"]) / hits
        region, distance = _nearest_region((cx, cy), regions)
        if region is None or distance > snap_max_m:
            continue
        out.append({
            "class_name": cluster["class_name"],
            "xy": [round(cx, 2), round(cy, 2)],
            "region": region,
            "hits": hits,
            "mean_score": round(sum(cluster["scores"]) / hits, 3),
        })
    out.sort(key=lambda r: (r["class_name"], -r["hits"], -r["mean_score"]))
    return out


def _nearest_region(xy, regions):
    """``(name, distance)`` of the closest region centroid, or ``(None, inf)``."""
    best, best_d = None, float("inf")
    for name, coords in (regions or {}).items():
        d = math.dist(xy, (coords[0], coords[1]))
        if d < best_d:
            best, best_d = name.lower(), d
    return best, best_d


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
