"""Perception's answer must reach the problem file while the robot is driving.

The mission problem asserts ``(location-unknown cone)`` and a goal naming the
cone, which has no achiever -- the robot drives that plan's executable prefix,
the survey. The one fact that turns the unsolvable problem into a solvable one
is the object's region, and YOLO produces it partway through the survey.

``/observation`` is where that fact lands. It rewrites the problem around the
sighting and puts it on disk at the moment of detection, rather than the
approach only discovering it once the survey has run out.
"""

import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "pipeline" / "evoplan_bridge_host"))

import server as server_mod  # noqa: E402
from problem_builder import MissionLibrary  # noqa: E402

TAG = "factory_tour_03_p0_evoplan_20260810_210000"


@pytest.fixture
def svc(tmp_path, monkeypatch):
    """A service with real missions but artifacts redirected into tmp_path."""
    monkeypatch.setattr(server_mod, "REPO", tmp_path)
    s = object.__new__(server_mod.ReplanService)
    s.lock = __import__("threading").Lock()
    s.observations = {}
    s.missions = MissionLibrary(REPO / "evolve_stl_pddl" / "jackal" / "in",
                                extra_dirs=[REPO / "pipeline" / "missions"])
    return s


def sighting(**over):
    base = {"tag": TAG, "mission_id": "factory_tour_03", "robot": "jackal_1",
            "class": "traffic cone", "region": "r1", "current_region": "r2",
            "visited_regions": ["r1", "r2"],
            "evidence": {"hits": 64, "mean_score": 0.82}}
    base.update(over)
    return base


def written(tmp_path):
    return tmp_path / "results" / "single_trials" / f"{TAG}_problem_object_found.pddl"


def code(text):
    import re
    return re.sub(r";[^\n]*", "", text)


class TestTheProblemIsRewrittenOnSighting:
    def test_reports_accepted(self, svc, tmp_path):
        out = svc.observe(sighting())
        assert out["ok"] and out["region"] == "r1" and out["changed"]

    def test_problem_file_appears_at_detection_time(self, svc, tmp_path):
        svc.observe(sighting())
        assert written(tmp_path).is_file()

    def test_the_unknown_becomes_a_fact(self, svc, tmp_path):
        svc.observe(sighting())
        body = code(written(tmp_path).read_text())
        init = body.split("(:goal")[0]
        assert "(object-at cone r1)" in init
        assert "location-unknown" not in init, (
            "left in place, it is approach's negative precondition and the "
            "problem stays unsolvable with the cone in plain sight")

    def test_the_goal_becomes_reachable(self, svc, tmp_path):
        """The whole point: an unsolvable problem becomes a solvable one."""
        svc.observe(sighting())
        goal = code(written(tmp_path).read_text()).split("(:goal")[1]
        assert "(reached jackal_1 cone)" in goal

    def test_class_is_mapped_onto_the_declared_pddl_symbol(self, svc, tmp_path):
        """Perception says "traffic cone"; the mission declares `cone`."""
        svc.observe(sighting(**{"class": "traffic cone"}))
        assert "(object-at cone r1)" in code(written(tmp_path).read_text())


class TestItDoesNotDisturbTheDrive:
    def test_a_repeat_sighting_is_a_no_op(self, svc, tmp_path):
        """The executor polls on a timer; a steady detection stream must not
        rewrite the file every two seconds."""
        svc.observe(sighting())
        stamp = written(tmp_path).stat().st_mtime_ns
        out = svc.observe(sighting())
        assert out["changed"] is False
        assert written(tmp_path).stat().st_mtime_ns == stamp

    def test_a_moved_object_rewrites(self, svc, tmp_path):
        svc.observe(sighting(region="r1"))
        out = svc.observe(sighting(region="r2"))
        assert out["changed"] is True
        assert "(object-at cone r2)" in code(written(tmp_path).read_text())

    def test_no_planning_happens(self, svc, tmp_path):
        """A sighting must not cut the survey short -- the mission is 'survey,
        THEN approach'. observe() records and returns; escalation stays with
        the executor."""
        assert not hasattr(svc, "jobs") or not getattr(svc, "jobs", None)
        svc.observe(sighting())
        assert not getattr(svc, "jobs", None)

    def test_an_unwritable_results_dir_still_records(self, svc, tmp_path, monkeypatch):
        """Losing the artifact must not lose the sighting."""
        def boom(*a, **k):
            raise OSError("read-only file system")
        monkeypatch.setattr(Path, "write_text", boom)
        out = svc.observe(sighting())
        assert out["ok"] and out["problem_file"] is None
        assert svc.observations[(TAG, "traffic cone")]["region"] == "r1"


class TestBadInput:
    @pytest.mark.parametrize("bad", [{"region": ""}, {"class": ""}, {"region": None}])
    def test_incomplete_reports_are_rejected(self, svc, bad):
        out = svc.observe(sighting(**bad))
        assert out["ok"] is False

    def test_a_hostile_tag_cannot_steer_the_write(self, svc, tmp_path):
        svc.observe(sighting(tag="../../../etc/passwd"))
        files = list((tmp_path / "results" / "single_trials").glob("*.pddl"))
        assert files and all(f.parent.name == "single_trials" for f in files)


class TestTheReplanRemembersIt:
    """A replan for a trial whose object has already been seen must be built
    against the real position even if the request itself omits it."""

    def test_remembered_when_the_request_carries_nothing(self, svc):
        svc.observe(sighting())
        got = svc._remembered_sighting({"tag": TAG, "mission_id": "factory_tour_03"})
        assert got == [{"class": "traffic cone", "region": "r1"}]

    def test_every_object_is_remembered_not_just_the_latest(self, svc):
        """A mission may hunt several. Returning only the newest would make the
        second discovery erase the first from the next replan's problem."""
        svc.observe(sighting(mission_id="factory_tour_02"))
        svc.observe(sighting(mission_id="factory_tour_02",
                             **{"class": "skateboard", "region": "r3"}))
        got = svc._remembered_sighting({"tag": TAG, "mission_id": "factory_tour_02"})
        assert got == [{"class": "traffic cone", "region": "r1"},
                       {"class": "skateboard", "region": "r3"}]

    def test_another_trial_does_not_inherit_it(self, svc):
        svc.observe(sighting())
        assert svc._remembered_sighting({"tag": "some_other_trial"}) is None

    def test_nothing_seen_yet(self, svc):
        assert svc._remembered_sighting({"tag": TAG}) is None


class TestTheGeneratedProblemIsTheReplanInput:
    """The artifact must be load-bearing, not a parallel record.

    /observation writes the problem naming the object's position while the
    robot is still surveying. The replan that follows is planned FROM that
    file -- if the service instead rebuilt an equivalent problem from the
    authored mission, the file would be a diagnostic that merely happened to
    match, and editing it would change nothing.
    """

    def base(self, svc, tag=TAG, mission="factory_tour_03"):
        text, source = svc._base_problem({"tag": tag, "mission_id": mission}, mission)
        return code(text), source

    def test_before_any_sighting_it_is_the_authored_mission(self, svc):
        text, source = self.base(svc)
        assert source == "mission:factory_tour_03"
        assert "(location-unknown cone)" in text

    def test_after_a_sighting_it_is_the_generated_file(self, svc, tmp_path):
        svc.observe(sighting())
        text, source = self.base(svc)
        assert source == str(written(tmp_path))
        assert "(object-at cone r1)" in text
        assert "location-unknown" not in text.split("(:goal")[0]

    def test_it_is_re_read_from_disk_every_time(self, svc, tmp_path):
        """Load-bearing means editable: a change to the file must reach the
        next replan, not be masked by an in-memory copy."""
        svc.observe(sighting())
        f = written(tmp_path)
        f.write_text(f.read_text().replace("(object-at cone r1)", "(object-at cone r2)"))
        assert "(object-at cone r2)" in self.base(svc)[0]

    def test_a_deleted_file_falls_back_rather_than_failing(self, svc, tmp_path):
        """The robot is holding station; a stale-but-valid problem beats none."""
        svc.observe(sighting())
        written(tmp_path).unlink()
        text, source = self.base(svc)
        assert source == "mission:factory_tour_03"
        assert "(location-unknown cone)" in text

    def test_a_corrupt_file_falls_back(self, svc, tmp_path):
        svc.observe(sighting())
        written(tmp_path).write_text("(define (problem broken)")
        assert self.base(svc)[1] == "mission:factory_tour_03"

    def test_another_trial_is_unaffected(self, svc):
        svc.observe(sighting())
        assert self.base(svc, tag="different_trial")[1] == "mission:factory_tour_03"

    def test_replanning_on_it_does_not_duplicate_the_position(self, svc, tmp_path):
        """The base already carries (object-at cone r1). Splicing it again --
        or splicing a moved position alongside the old one -- would hand the
        planner two contradictory facts about where the object is."""
        from problem_builder import build_runtime_problem
        svc.observe(sighting())
        base, _ = svc._base_problem({"tag": TAG, "mission_id": "factory_tour_03"},
                                    "factory_tour_03")
        out = code(build_runtime_problem(
            base, robot="jackal_1", current_region="r5", blocked_regions=[],
            visited_regions=["r1", "r2", "r5"], target_region="r1",
            preserve_goal=False, found_object={"class": "traffic cone", "region": "r1"}))
        assert out.count("(object-at cone") == 1

    def test_a_moved_object_replaces_rather_than_accumulates(self, svc, tmp_path):
        from problem_builder import build_runtime_problem
        svc.observe(sighting(region="r1"))
        base, _ = svc._base_problem({"tag": TAG, "mission_id": "factory_tour_03"},
                                    "factory_tour_03")
        out = code(build_runtime_problem(
            base, robot="jackal_1", current_region="r5", blocked_regions=[],
            visited_regions=["r1", "r2", "r5"], target_region="r2",
            preserve_goal=False, found_object={"class": "traffic cone", "region": "r2"}))
        assert out.count("(object-at cone") == 1
        assert "(object-at cone r2)" in out


class TestTwoObjectMission:
    """factory_tour_02 hunts the cone AND the skateboard, found minutes apart.

    All sightings for a trial write the same problem file, so the second
    discovery must rebuild around BOTH -- rebuilding around the latest alone
    drops the first object's (object-at ...) and shrinks the goal back to one
    object, which reads as success right up until the robot skips half the
    mission.
    """

    def two(self, svc):
        svc.observe(sighting(mission_id="factory_tour_02", region="r1",
                             **{"class": "traffic cone"}))
        return svc.observe(sighting(mission_id="factory_tour_02", region="r3",
                                    **{"class": "skateboard"}))

    def test_both_positions_survive_the_second_sighting(self, svc, tmp_path):
        self.two(svc)
        init = code(written(tmp_path).read_text()).split("(:goal")[0]
        assert "(object-at cone r1)" in init
        assert "(object-at skateboard r3)" in init

    def test_both_unknowns_are_retracted(self, svc, tmp_path):
        self.two(svc)
        assert "location-unknown" not in code(written(tmp_path).read_text()).split("(:goal")[0]

    def test_the_goal_keeps_both_conjuncts(self, svc, tmp_path):
        self.two(svc)
        goal = code(written(tmp_path).read_text()).split("(:goal")[1]
        assert "(reached jackal_1 cone)" in goal
        assert "(reached jackal_1 skateboard)" in goal

    def test_after_only_the_first_the_other_is_still_unknown(self, svc, tmp_path):
        """Partial knowledge must be representable: one object located, one
        still being searched for."""
        svc.observe(sighting(mission_id="factory_tour_02", region="r1",
                             **{"class": "traffic cone"}))
        body = code(written(tmp_path).read_text())
        init, goal = body.split("(:goal")[0], body.split("(:goal")[1]
        assert "(object-at cone r1)" in init
        assert "(location-unknown skateboard)" in init
        assert "(reached jackal_1 cone)" in goal
        assert "skateboard" not in goal, (
            "an object with no known position cannot be a goal conjunct yet")

    def test_the_response_names_everything_known(self, svc):
        assert self.two(svc)["known_objects"] == ["traffic cone", "skateboard"]

    def test_a_moved_object_does_not_disturb_the_other(self, svc, tmp_path):
        self.two(svc)
        svc.observe(sighting(mission_id="factory_tour_02", region="r4",
                             **{"class": "skateboard"}))
        init = code(written(tmp_path).read_text()).split("(:goal")[0]
        assert "(object-at cone r1)" in init
        assert "(object-at skateboard r4)" in init
        assert init.count("(object-at skateboard") == 1

    def test_the_classes_resolve_to_distinct_pddl_symbols(self, svc, tmp_path):
        """With two targets declared the sole-target fallback is off, so this
        is real name resolution: 'traffic cone' by last word, 'skateboard'
        outright."""
        self.two(svc)
        init = code(written(tmp_path).read_text()).split("(:goal")[0]
        assert init.count("(object-at ") == 2
