#!/usr/bin/env python3
"""Stable identities for objects nobody knew were there.

A find-and-inspect mission is open-world: the problem file cannot declare
``cone_1 cone_2 cone_3`` because how many cones exist is the thing the robot was
sent to find out. Objects therefore have to be *minted* at runtime, from
perception, and that creates a requirement classical planning takes for granted
and this pipeline has to earn -- **an object must keep the same name for the
whole mission**.

Everything downstream depends on it:

* the PDDL problem declares ``traffic_cone_2 - target`` and asserts
  ``(object-at traffic_cone_2 r3)``; a rename between replans reads to the
  planner as a *new, uninspected* object, and the robot re-inspects one it has
  already done while the real newcomer goes unvisited;
* ``(inspected-object traffic_cone_2)`` is how the executor records work
  already performed, and a name is the only handle it has on that fact.

:func:`~evoplan_bridge.observation_memory.cluster_detections` re-clusters the
whole log from scratch on every poll, and it must -- a later detection can merge
two clusters or sharpen a centroid. So identity cannot live in the clustering.
It lives here: clusters are matched to already-known objects by class and
proximity, and only an unmatched cluster mints a name.

Pure stdlib, no ROS, so the identity rules are unit-testable without a simulator.
"""

from __future__ import annotations

import math
import re

__all__ = ["InspectionTarget", "ObjectRegistry", "DEFAULT_MERGE_RADIUS_M"]

#: How far a cluster centroid may move between polls and still be judged the
#: same object. Tracks the clustering link radius: the centroid of a growing
#: cluster drifts as detections accumulate (a cone first seen from one side
#: settles toward its true position as the robot drives past), and that drift
#: must not look like a second cone. A merge radius SMALLER than the link radius
#: would mint duplicates for drift the clustering had already accepted.
#:
#: Raising this alone would not have prevented the phantom cone that motivated
#: the change -- see DEFAULT_LINK_RADIUS_M. `update` lets each known object
#: absorb at most one cluster per call, so two clusters present in the SAME poll
#: always produce two objects however generous this radius is. The split has to
#: be prevented in the clustering; this value only keeps identity stable
#: afterwards.
DEFAULT_MERGE_RADIUS_M = 5.0


def _slug(text: str) -> str:
    """Detector class -> PDDL-legal symbol stem ("traffic cone" -> traffic_cone).

    PDDL names cannot contain spaces, and the detector classes routinely do.
    Leading digits are prefixed rather than stripped, because ``2_person`` is
    not a legal symbol and dropping the digit would collide two classes.
    """
    slug = re.sub(r"[^a-z0-9_]+", "_", (text or "").lower()).strip("_")
    if not slug:
        return "object"
    return slug if slug[0].isalpha() else f"obj_{slug}"


class InspectionTarget:
    """One physical object the mission has discovered.

    ``inspected`` is the mission's memory of work done. It is set by the
    executor when the dwell completes, never by perception -- seeing an object
    again is not inspecting it.
    """

    __slots__ = ("name", "class_name", "xy", "region", "hits", "mean_score",
                 "inspected", "inspected_at", "first_seen_at")

    def __init__(self, name, class_name, xy, region, hits=0, mean_score=0.0,
                 first_seen_at=None):
        self.name = name
        self.class_name = class_name
        self.xy = (float(xy[0]), float(xy[1]))
        self.region = region
        self.hits = int(hits)
        self.mean_score = float(mean_score)
        self.inspected = False
        self.inspected_at = None
        self.first_seen_at = first_seen_at

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "class": self.class_name,
            "xy": [round(self.xy[0], 2), round(self.xy[1], 2)],
            "region": self.region,
            "hits": self.hits,
            "mean_score": self.mean_score,
            "inspected": self.inspected,
            "inspected_at": self.inspected_at,
        }

    def as_instance(self) -> dict:
        """What ``build_runtime_problem(inspect_objects=...)`` consumes.

        ``inspected`` is not decoration. The problem's goal carries an
        ``(inspected-object ?t)`` conjunct for every object, so unless ``:init``
        also records the ones already done, a sound planner must plan to do them
        AGAIN -- it has been handed a goal it is told is unmet. That is not an
        LLM failing: Fast Downward, given the same problem, produced the same
        redundant re-inspection. This field is what lets the splice state the
        truth the executor already knows.
        """
        return {"name": self.name, "region": self.region,
                "class": self.class_name, "inspected": self.inspected}

    def __repr__(self) -> str:  # pragma: no cover - diagnostics only
        state = "inspected" if self.inspected else "pending"
        return (f"InspectionTarget({self.name} @ {self.region} "
                f"{self.xy} {self.hits} hits, {state})")


class ObjectRegistry:
    """The mission's list of what is out there, kept stable across polls."""

    def __init__(self, merge_radius_m: float = DEFAULT_MERGE_RADIUS_M):
        self.merge_radius_m = float(merge_radius_m)
        self._targets: dict[str, InspectionTarget] = {}
        self._minted: dict[str, int] = {}

    # ------------------------------------------------------------------
    def update(self, clusters, now=None) -> list[InspectionTarget]:
        """Fold a fresh clustering in. Returns only the objects newly minted.

        Callers use the return value to decide whether anything CHANGED -- a
        poll that discovers nothing new must not trigger a replan, and the
        clustering itself offers no way to tell, since it returns every object
        every time.

        Matching is nearest-first within a class, and each existing object can
        absorb at most one cluster per call. Without that constraint two cones
        1.5 m apart would both match the nearer registry entry, the second would
        overwrite the first's position, and the further cone would never get a
        name of its own.
        """
        new = []
        claimed = set()
        for cluster in sorted(clusters or (), key=lambda c: -int(c.get("hits") or 0)):
            class_name = (cluster.get("class_name") or "").lower()
            xy = cluster.get("xy")
            region = (cluster.get("region") or "").lower()
            if not class_name or not region or not xy:
                continue
            match = self._nearest(class_name, xy, claimed)
            if match is None:
                match = self._mint(class_name, xy, region, now)
                new.append(match)
            claimed.add(match.name)
            # Position and region always track the newest evidence; `inspected`
            # never does. An object seen again after the dwell is still done.
            match.xy = (float(xy[0]), float(xy[1]))
            match.region = region
            match.hits = int(cluster.get("hits") or match.hits)
            match.mean_score = float(cluster.get("mean_score") or match.mean_score)
        return new

    def _nearest(self, class_name, xy, claimed):
        best, best_d = None, float("inf")
        for target in self._targets.values():
            if target.class_name != class_name or target.name in claimed:
                continue
            d = math.dist((float(xy[0]), float(xy[1])), target.xy)
            if d < best_d:
                best, best_d = target, d
        return best if best is not None and best_d <= self.merge_radius_m else None

    def _mint(self, class_name, xy, region, now):
        index = self._minted.get(class_name, 0) + 1
        self._minted[class_name] = index
        name = f"{_slug(class_name)}_{index}"
        target = InspectionTarget(name, class_name, xy, region,
                                  first_seen_at=now)
        self._targets[name] = target
        return target

    # ------------------------------------------------------------------
    def get(self, name) -> InspectionTarget | None:
        return self._targets.get(str(name or "").lower())

    def mark_inspected(self, name, at=None) -> bool:
        """Record that the dwell completed. False if the name is unknown."""
        target = self.get(name)
        if target is None:
            return False
        target.inspected = True
        target.inspected_at = at
        return True

    def all(self) -> list[InspectionTarget]:
        return sorted(self._targets.values(), key=lambda t: t.name)

    def pending(self) -> list[InspectionTarget]:
        """Discovered but not yet inspected. Empty means the mission is done."""
        return [t for t in self.all() if not t.inspected]

    def instances(self, pending_only: bool = False) -> list[dict]:
        """``inspect_objects`` payload for the replan request.

        ``pending_only=False`` by default, and that is the load-bearing choice:
        the problem must keep declaring objects already inspected, because
        dropping one would delete its ``(object-at ...)`` and leave the planner
        unable to explain the ``(inspected-object ...)`` the mission is carrying.
        """
        return [t.as_instance() for t in self.all()
                if not (pending_only and t.inspected)]

    def summary(self) -> dict:
        """Compact status blob for JSON events and the ROS status topic."""
        targets = self.all()
        return {
            "total": len(targets),
            "inspected": sum(1 for t in targets if t.inspected),
            "pending": [t.name for t in targets if not t.inspected],
            "by_class": {
                class_name: sum(1 for t in targets if t.class_name == class_name)
                for class_name in sorted({t.class_name for t in targets})
            },
            "objects": [t.as_dict() for t in targets],
        }

    def __len__(self) -> int:
        return len(self._targets)
