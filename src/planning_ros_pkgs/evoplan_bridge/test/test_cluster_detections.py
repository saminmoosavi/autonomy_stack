"""Counting objects, not regions.

``locate_object`` answers "which region holds a cone" and is enough for a
mission hunting one named object. An open-world find-and-inspect mission needs
the count and the position of each instance, and the difference is not
cosmetic: two cones five metres apart in one region are ONE ``locate_object``
entry whose centroid sits in the empty space between them -- a position that is
useless both to drive to and to face.

Fixtures come from ``obs_fixtures`` for the reason that module exists: the
logger writes ``position_map`` as a mapping, hand-written ``[x, y]`` fixtures
once made twelve tests pass against a shape the system never produces.
"""

import sys
from pathlib import Path

import pytest
from obs_fixtures import detection, record

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "src" / "planning_ros_pkgs" / "evoplan_bridge"))

from evoplan_bridge.observation_memory import (  # noqa: E402
    DEFAULT_LINK_RADIUS_M,
    cluster_detections,
    locate_object,
)

#: Region centroids, spaced so a detection near one is nowhere near another.
REGIONS = {"r1": (0.0, 0.0), "r2": (20.0, 0.0), "r3": (40.0, 0.0)}


def seen(xy, cls="traffic cone", times=4, score=0.8, start=0.0):
    """``times`` records of one detection at ``xy``, one per second."""
    return [record(start + i, [detection(cls, xy, score=score)])
            for i in range(times)]


class TestInstances:
    def test_two_objects_in_one_region_are_two_entries(self):
        """Both 3 m from r1's centroid, 6 m from each other -- beyond the 5 m
        link radius, so two objects sharing one region stay two objects."""
        records = seen((-3.0, 0.0)) + seen((3.0, 0.0), start=10)
        found = cluster_detections(records, ["traffic cone"], regions=REGIONS)
        assert len(found) == 2
        assert {tuple(f["xy"]) for f in found} == {(-3.0, 0.0), (3.0, 0.0)}

    def test_locate_object_collapses_what_this_separates(self):
        """The motivating comparison, pinned so the distinction cannot be lost."""
        records = seen((-3.0, 0.0)) + seen((3.0, 0.0), start=10)
        assert len(locate_object(records, "traffic cone", regions=REGIONS)) == 1
        assert len(cluster_detections(records, ["traffic cone"], regions=REGIONS)) == 2

    def test_repeated_frames_of_one_object_are_one_entry(self):
        records = seen((1.0, 1.0), times=12)
        found = cluster_detections(records, ["traffic cone"], regions=REGIONS)
        assert len(found) == 1
        assert found[0]["hits"] == 12

    def test_jitter_within_the_link_radius_does_not_split(self):
        records = ([record(i, [detection("traffic cone", (1.0 + 0.2 * i, 1.0))])
                    for i in range(5)])
        found = cluster_detections(records, ["traffic cone"], regions=REGIONS,
                                   link_radius_m=1.5)
        assert len(found) == 1

    def test_each_instance_carries_its_own_region(self):
        records = seen((0.5, 0.0)) + seen((20.4, 0.0), start=10)
        found = cluster_detections(records, ["traffic cone"], regions=REGIONS)
        assert {f["region"] for f in found} == {"r1", "r2"}


class TestEvidence:
    def test_thin_evidence_is_dropped(self):
        """A single spurious frame must not become an object to drive to."""
        records = seen((1.0, 1.0), times=2)
        assert cluster_detections(records, ["traffic cone"], regions=REGIONS,
                                  min_hits=3) == []

    def test_low_scores_are_dropped(self):
        records = seen((1.0, 1.0), score=0.2)
        assert cluster_detections(records, ["traffic cone"], regions=REGIONS,
                                  min_score=0.5) == []

    def test_only_the_requested_classes_count(self):
        records = seen((1.0, 1.0)) + seen((2.0, 2.0), cls="chair", start=10)
        found = cluster_detections(records, ["traffic cone"], regions=REGIONS)
        assert [f["class_name"] for f in found] == ["traffic cone"]

    def test_every_class_when_none_requested(self):
        records = seen((1.0, 1.0)) + seen((5.0, 2.0), cls="chair", start=10)
        found = cluster_detections(records, regions=REGIONS)
        assert {f["class_name"] for f in found} == {"traffic cone", "chair"}

    def test_detections_far_from_every_region_are_dropped(self):
        """With no region to name there is no (object-at ...) to assert."""
        records = seen((500.0, 500.0))
        assert cluster_detections(records, ["traffic cone"], regions=REGIONS,
                                  snap_max_m=6.0) == []

    def test_detections_without_a_position_are_dropped(self):
        records = [record(i, [{"class_name": "traffic cone", "score": 0.9,
                               "position_map": None, "region": "r1"}])
                   for i in range(5)]
        assert cluster_detections(records, ["traffic cone"], regions=REGIONS) == []

    def test_no_records_is_no_objects(self):
        assert cluster_detections([], ["traffic cone"], regions=REGIONS) == []


class TestTheLinkRadius:
    """The phantom cone, pinned to the geometry that actually produced it.

    Run 1 of batch2 (`factory_survey_02_p0_evoplan_only_i1_20260811_151344`)
    split ONE traffic cone into two objects: a 63-hit cluster at (2.76, -11.33)
    and a 6-hit cluster 2.53 m away at (0.43, -12.32). The robot minted
    `traffic_cone_3` for the second, drove to it, and held a 5 s inspection of
    empty floor. Depth projection scatters detections much further than the
    1.5 m the radius was originally sized for.
    """

    #: The observed split, translated to sit near r1.
    REAL = (0.0, 0.0)
    GHOST = (-2.33, -0.99)      # 2.53 m from REAL, as measured in the run

    def test_the_observed_split_is_one_object_at_the_default(self):
        records = seen(self.REAL, times=63) + seen(self.GHOST, times=6, start=200)
        found = cluster_detections(records, ["traffic cone"], regions=REGIONS)
        assert len(found) == 1, "the phantom cone is back"
        assert found[0]["hits"] == 69

    def test_the_old_radius_is_what_split_it(self):
        """Documents the knob, and why the default moved off 1.5 m."""
        records = seen(self.REAL, times=63) + seen(self.GHOST, times=6, start=200)
        found = cluster_detections(records, ["traffic cone"], regions=REGIONS,
                                   link_radius_m=1.5)
        assert len(found) == 2

    def test_the_default_is_five_metres(self):
        """A bare number here so a silent edit to the constant fails loudly --
        it decides whether two objects are one, which nothing downstream can
        second-guess."""
        assert DEFAULT_LINK_RADIUS_M == 5.0


class TestStability:
    @pytest.mark.parametrize("extra", [0, 3, 9])
    def test_more_evidence_does_not_move_an_object_to_another_cluster(self, extra):
        """The registry matches on position, so growing evidence must refine a
        cluster rather than reshuffle which detections belong to it."""
        records = seen((-3.0, 0.0), times=4 + extra) + seen((3.0, 0.0), start=50)
        found = cluster_detections(records, ["traffic cone"], regions=REGIONS)
        assert len(found) == 2
        near = min(found, key=lambda f: f["xy"][0])
        assert near["xy"] == [-3.0, 0.0]

    def test_output_order_is_deterministic(self):
        records = seen((3.0, 0.0)) + seen((-3.0, 0.0), times=9, start=20)
        first = cluster_detections(records, ["traffic cone"], regions=REGIONS)
        second = cluster_detections(records, ["traffic cone"], regions=REGIONS)
        assert first == second
        # Strongest evidence first, as locate_object does.
        assert first[0]["hits"] >= first[1]["hits"]
