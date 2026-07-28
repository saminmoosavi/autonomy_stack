#!/usr/bin/env python3
"""Factory graph helpers: regions, objects, and the shared type vocabulary.

Copied out of ``evo_skill_ros/nodes/eveo_plan_deploy.py`` so
``nodes/observation_logger.py`` can attribute observations to graph regions
without importing (and thereby instantiating anything from) the planner node.
``eveo_plan_deploy.py`` keeps its own private copies for now; migrating it to
this module is a separate, revertable, pure-deletion change.

``object_type_from_name`` is the load-bearing piece: it normalises BOTH graph
object names and YOLO class names into one vocabulary, which is the only space
in which "expected vs observed" is comparable -- the graph says
``short_shelf_2`` while YOLO says ``shelf``.
"""

from __future__ import annotations

import json
import math
import re
from typing import Dict, Optional, Tuple

Coords = Tuple[float, float]


def load_graph(path) -> Tuple[dict, Dict[str, Coords], Dict[str, Coords]]:
    """Load graph.json -> (world, regions, objects), names lowercased."""
    with open(path, "r") as stream:
        world = json.load(stream)
    regions = {
        region["name"].lower(): tuple(region["coords"])
        for region in world.get("regions", [])
    }
    objects = {
        obj["name"].lower(): tuple(obj["coords"])
        for obj in world.get("objects", [])
    }
    return world, regions, objects


def nearest_region(
    regions: Dict[str, Coords], xy: Coords
) -> Tuple[Optional[str], float]:
    """Closest region centroid to xy.

    NOTE: regions in graph.json are single points, not polygons, so this returns
    the closest one UNCONDITIONALLY -- a point 40 m from everything still snaps.
    Callers must apply their own distance cutoff (see region_snap_max_m in
    observation_logger) before treating the result as membership.
    """
    x, y = xy
    best_region = None
    best_distance = float("inf")
    for name, (rx, ry) in regions.items():
        distance = math.hypot(x - rx, y - ry)
        if distance < best_distance:
            best_region = name
            best_distance = distance
    return best_region, best_distance


def object_type_from_name(name: str) -> str:
    """Normalise a graph object name OR a YOLO class name to a common type."""
    normalized = str(name).lower()
    if normalized.startswith(("person", "human")):
        return "human"
    if normalized.startswith("chair"):
        return "chair"
    if "shelf" in normalized:
        return "shelf"
    if normalized.startswith("column"):
        return "column"
    if normalized.startswith("table"):
        return "table"
    if normalized.startswith("frisbe"):
        return "frisbe"
    return "obstacle"


def normalize_track_label(value) -> str:
    normalized = str(value).strip().lower()
    normalized = re.sub(r"[^a-z0-9_]+", "_", normalized)
    return normalized.strip("_")


def expected_objects_by_region(
    regions: Dict[str, Coords],
    objects: Dict[str, Coords],
    snap_max_m: float,
) -> Dict[str, Dict[str, list]]:
    """Snap every graph object to its nearest region, grouped by object type.

    Returns {region_name: {object_type: [object_name, ...]}}. Objects farther
    than snap_max_m from every region centroid are dropped rather than
    attributed to a distant region.
    """
    expected: Dict[str, Dict[str, list]] = {name: {} for name in regions}
    for obj_name, obj_xy in objects.items():
        region, distance = nearest_region(regions, obj_xy)
        if region is None or distance > snap_max_m:
            continue
        obj_type = object_type_from_name(obj_name)
        expected[region].setdefault(obj_type, []).append(obj_name)
    for by_type in expected.values():
        for names in by_type.values():
            names.sort()
    return expected
