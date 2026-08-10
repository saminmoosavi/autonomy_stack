"""Two-phase find-an-object missions: tour, then approach what was seen.

The mission the user asked for is "visit all regions, log objects and their
locations, then go to the <object>". The object's region is not known when the
mission starts -- that is the whole point -- so it cannot be written into the
PDDL goal up front. The executor therefore runs a normal region tour, and when
that plan finishes it queries the observation log and retargets.

These exercise the decision logic against the real ``observation_memory`` search
over synthetic JSONL, with no ROS, no sim and no LLM. What they protect:

* the tour must not be able to declare victory before the approach happens;
* evidence thresholds must actually gate retargeting, because a false positive
  sends the robot to the wrong region and calls it success;
* a never-seen object must end the mission cleanly rather than hang.
"""

import pytest

from evoplan_bridge.observation_memory import load_observations, locate_object

import obs_fixtures

REGIONS = {
    "r1": (-12.29, -12.29), "r5": (1.44, 2.74), "r8": (3.52, 13.01),
    "r10": (1.89, 22.13), "r14": (-12.94, 9.89),
}


def write_log(tmp_path, detections):
    """detections: list of (class_name, score, x, y). One record each.

    Records are built by obs_fixtures so they carry the logger's real
    position_map MAPPING. This function used to inline `"position_map": [x, y]`,
    a list -- a shape no live run produces -- so every test below passed while
    locate_object died with KeyError: 0 against an actual observations.jsonl.
    """
    return obs_fixtures.write_jsonl(
        tmp_path / "observations.jsonl",
        [obs_fixtures.record(i, [obs_fixtures.detection(cls, (x, y), score=score)])
         for i, (cls, score, x, y) in enumerate(detections)],
    )


class FindPhase:
    """Mirrors the executor's search -> approach -> done transition."""

    def __init__(self, obs_log, cls, min_hits=3, min_score=0.5):
        self.obs_log, self.cls = obs_log, cls
        self.min_hits, self.min_score = min_hits, min_score
        self.phase = "search" if cls else "disabled"
        self.target_region = "r14"       # the tour's endpoint
        self.region = None
        self.mission_complete = False
        self.tour_regions = set(REGIONS)
        self.visited = set()

    def begin_approach(self):
        self.phase = "not_found"
        records = load_observations(self.obs_log)
        if not records:
            return False
        found = locate_object(records, self.cls, min_score=self.min_score,
                              min_hits=self.min_hits, regions=REGIONS)
        if not found or found[0]["region"] not in REGIONS:
            return False
        self.region = found[0]["region"]
        self.target_region = self.region
        self.phase = "approach"
        return True

    def update_completion(self, at_target):
        if self.phase == "search":
            return
        if self.phase == "approach":
            if not at_target:
                return
            self.phase = "done"
        if self.tour_regions - self.visited or not at_target:
            return
        self.mission_complete = True


class TestTourCannotFinishTheMission:
    def test_completing_the_tour_does_not_complete_the_mission(self, tmp_path):
        """The tour ending at r14 must not latch success -- the approach is the
        mission. Without this gate the object is never looked up."""
        log = write_log(tmp_path, [("bookshelf", 0.9, 1.5, 2.8)] * 3)
        f = FindPhase(log, "bookshelf")
        f.visited = set(REGIONS)
        f.update_completion(at_target=True)      # parked at r14, tour done
        assert not f.mission_complete
        assert f.phase == "search"

    def test_mission_completes_only_after_reaching_the_object(self, tmp_path):
        log = write_log(tmp_path, [("bookshelf", 0.9, 1.5, 2.8)] * 3)
        f = FindPhase(log, "bookshelf")
        f.visited = set(REGIONS)
        assert f.begin_approach()
        assert f.target_region == "r5"
        f.update_completion(at_target=False)     # still driving to r5
        assert not f.mission_complete
        f.update_completion(at_target=True)
        assert f.phase == "done" and f.mission_complete


class TestRetargeting:
    def test_retargets_to_the_region_that_held_the_object(self, tmp_path):
        """The bookshelf sits at (3.04, 2.74); r5 is (1.44, 2.74)."""
        log = write_log(tmp_path, [("bookshelf", 0.8, 3.04, 2.74)] * 4)
        f = FindPhase(log, "bookshelf")
        assert f.begin_approach()
        assert f.region == "r5"

    def test_strongest_evidence_wins(self, tmp_path):
        """Same class seen in two regions -- go where it was seen most."""
        log = write_log(
            tmp_path,
            [("chair", 0.9, 1.5, 2.8)] * 3 + [("chair", 0.9, 3.6, 13.0)] * 9)
        f = FindPhase(log, "chair")
        assert f.begin_approach()
        assert f.region == "r8", "should prefer the 9-hit region over the 3-hit one"

    def test_target_is_left_alone_when_nothing_is_found(self, tmp_path):
        log = write_log(tmp_path, [("chair", 0.9, 1.5, 2.8)] * 5)
        f = FindPhase(log, "bookshelf")
        assert not f.begin_approach()
        assert f.target_region == "r14", "must not retarget on no evidence"


class TestEvidenceGating:
    def test_below_min_hits_does_not_retarget(self, tmp_path):
        """Two sightings are not enough at min_hits=3. A false positive sends
        the robot to the wrong region and reports success."""
        log = write_log(tmp_path, [("bookshelf", 0.9, 1.5, 2.8)] * 2)
        assert not FindPhase(log, "bookshelf", min_hits=3).begin_approach()

    def test_below_min_score_does_not_retarget(self, tmp_path):
        """A live run produced phantom detections sitting at the 0.5 floor."""
        log = write_log(tmp_path, [("bookshelf", 0.3, 1.5, 2.8)] * 10)
        assert not FindPhase(log, "bookshelf", min_score=0.5).begin_approach()

    def test_evidence_just_over_threshold_retargets(self, tmp_path):
        """Guards the boundary itself, so the two tests above straddle it."""
        log = write_log(tmp_path, [("bookshelf", 0.51, 1.5, 2.8)] * 3)
        assert FindPhase(log, "bookshelf", min_hits=3, min_score=0.5).begin_approach()


class TestDegradesCleanly:
    def test_missing_log_is_not_an_exception(self, tmp_path):
        f = FindPhase(str(tmp_path / "nope.jsonl"), "bookshelf")
        assert not f.begin_approach()
        assert f.phase == "not_found"

    def test_empty_class_disables_the_feature(self, tmp_path):
        """The default. Every existing mission must behave exactly as before."""
        f = FindPhase(write_log(tmp_path, []), "")
        assert f.phase == "disabled"
        f.visited = set(REGIONS)
        f.update_completion(at_target=True)
        assert f.mission_complete, "a normal tour must still complete"

    def test_detection_far_from_every_region_is_unattributed(self, tmp_path):
        """position_map beyond the 6 m snap radius must not be forced onto the
        nearest centroid -- that would invent a location."""
        log = write_log(tmp_path, [("bookshelf", 0.9, 200.0, 200.0)] * 5)
        assert not FindPhase(log, "bookshelf").begin_approach()
