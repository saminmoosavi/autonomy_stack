"""Index 0 and indices 1..N are written by different processes; they must agree.

The executor (in the container) archives the plan the robot starts on as
``<tag>_replan_0``; the host replan service archives everything after it. They
share no state, no counter and no filesystem view beyond a bind-mount, so the
only thing keeping the sequence coherent is that both call the same three
functions in ``evoplan_bridge.replan_artifacts``.

These tests exercise the seam. ``test_replan_artifacts.py`` already covers the
service's behaviour end to end; what is new here is that a ``_replan_0`` on
disk must slot in ahead of the service's numbering without either side
knowing about the other.
"""

import json
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "pipeline" / "evoplan_bridge_host"))

from evoplan_bridge.replan_artifacts import (  # noqa: E402
    next_index, safe_tag, write_artifacts)
import server as server_mod  # noqa: E402
from planners import PlanResult  # noqa: E402

TAG = "factory_tour_03_p0_evoplan_20260810_194032"


class FakeJob:
    def __init__(self, payload, result, problem_text="(define (problem p))"):
        self.id = "r-deadbeef"
        self.payload = payload
        self.result = result
        self.problem_text = problem_text


@pytest.fixture
def out(tmp_path, monkeypatch):
    monkeypatch.setattr(server_mod, "REPO", tmp_path)
    d = tmp_path / "results" / "single_trials"
    d.mkdir(parents=True)
    return d


def service_replan(out_dir, **over):
    """One archived replan, as the running service would write it."""
    payload = {"tag": TAG, "mission_id": "factory_tour_03",
               "current_region": "r5", "goal": {"target_region": "r1"},
               "reason": {"trigger": "nav2_goal_aborted"}}
    payload.update(over)
    svc = object.__new__(server_mod.ReplanService)
    server_mod.ReplanService._save_replan_artifacts(
        svc, FakeJob(payload, PlanResult(status="ok", plan=["(move jackal_1 r5 r1)"])))


def executor_replan_0(out_dir, tag=TAG, plan=("(move jackal_1 r5 r1)",)):
    """What archive_initial_plan writes, without standing up a ROS node."""
    return write_artifacts(out_dir, safe_tag(tag), next_index(out_dir, safe_tag(tag)),
                           "(define (problem factory_tour_03))", list(plan),
                           {"source": "plan_file", "authored": True,
                            "trigger": "initial_plan"})


def indices(out_dir, tag=TAG):
    return sorted(int(m.group(1)) for p in out_dir.glob("*.pddl")
                  if (m := re.fullmatch(rf"{tag}_replan_(\d+)\.pddl", p.name)))


class TestTheSequenceIsContiguousAcrossProcesses:
    def test_executor_takes_index_zero_on_an_empty_directory(self, out):
        assert executor_replan_0(out).name.endswith("_replan_0")

    def test_service_starts_at_one_when_zero_exists(self, out):
        """The service is never told index 0 happened; it reads it off disk."""
        executor_replan_0(out)
        service_replan(out)
        assert indices(out) == [0, 1]

    def test_service_still_starts_at_one_without_a_zero(self, out):
        """A mission that never archived an initial plan must not start at 0 --
        every existing trial's numbering has to keep meaning what it meant."""
        service_replan(out)
        assert indices(out) == [1]

    def test_a_full_trial_numbers_0_through_n(self, out):
        executor_replan_0(out)
        for _ in range(3):
            service_replan(out)
        assert indices(out) == [0, 1, 2, 3]

    def test_two_trials_do_not_interleave(self, out):
        executor_replan_0(out)
        executor_replan_0(out, tag="factory_tour_03_p0_evoplan_OTHER")
        service_replan(out)
        assert indices(out) == [0, 1]
        assert indices(out, "factory_tour_03_p0_evoplan_OTHER") == [0]


class TestReplanZeroContent:
    def test_all_three_files_exist(self, out):
        stem = executor_replan_0(out)
        for suffix in (".pddl", ".plan", ".json"):
            assert stem.with_suffix(suffix).is_file()

    def test_plan_matches_the_loaded_plan(self, out):
        stem = executor_replan_0(out, plan=("(move jackal_1 r5 r1)",
                                            "(move jackal_1 r1 r2)"))
        assert stem.with_suffix(".plan").read_text() == (
            "(move jackal_1 r5 r1)\n(move jackal_1 r1 r2)\n")

    def test_marked_authored_so_the_pddl_is_not_mistaken_for_a_runtime_problem(self, out):
        """No problem is built for the initial plan; the .pddl is the mission
        file as authored, with no (visited ...)/(blocked ...) spliced in."""
        meta = json.loads(executor_replan_0(out).with_suffix(".json").read_text())
        assert meta["authored"] is True
        assert meta["source"] == "plan_file"

    def test_a_missing_mission_file_still_archives_the_plan(self, out):
        """find_mission_problem returns None for an unknown mission. The plan
        is the part that cannot be recovered from anywhere else."""
        stem = write_artifacts(out, TAG, 0, None, ["(move jackal_1 r5 r1)"],
                               {"source": "plan_file", "problem_file": None})
        assert stem.with_suffix(".pddl").read_text() == ""
        assert stem.with_suffix(".plan").read_text().strip()


class TestSafeTag:
    @pytest.mark.parametrize("evil", ["../../../etc/passwd", "/absolute/path",
                                      "a/b/c", "..", "", "   "])
    def test_cannot_escape_the_directory(self, evil):
        tag = safe_tag(evil, fallback="factory_tour_03_r-1")
        assert "/" not in tag and ".." not in tag and tag

    def test_ordinary_tags_are_untouched(self):
        assert safe_tag(TAG) == TAG

    def test_falls_back_rather_than_returning_empty(self):
        assert safe_tag("", fallback="factory_tour_03_r-1") == "factory_tour_03_r-1"

    def test_fallback_is_sanitised_too(self):
        assert "/" not in safe_tag("", fallback="../../evil")


class TestNextIndex:
    def test_empty_directory_is_zero(self, tmp_path):
        assert next_index(tmp_path, TAG) == 0

    def test_gaps_do_not_reuse_an_index(self, tmp_path):
        """Numbering is max+1, not count: a deleted middle artifact must not
        cause the next write to overwrite a later one."""
        (tmp_path / f"{TAG}_replan_0.pddl").write_text("")
        (tmp_path / f"{TAG}_replan_7.pddl").write_text("")
        assert next_index(tmp_path, TAG) == 8

    def test_minimum_reserves_index_zero_for_the_executor(self, tmp_path):
        """What the service passes. Without it, a trial whose executor never
        archived an initial plan would number its first replan 0 and every
        pre-existing trial's numbering would mean something different."""
        assert next_index(tmp_path, TAG, minimum=1) == 1

    def test_minimum_does_not_cap_the_sequence(self, tmp_path):
        (tmp_path / f"{TAG}_replan_4.pddl").write_text("")
        assert next_index(tmp_path, TAG, minimum=1) == 5

    def test_other_tags_are_ignored(self, tmp_path):
        (tmp_path / "some_other_trial_replan_9.pddl").write_text("")
        assert next_index(tmp_path, TAG) == 0

    def test_similar_names_do_not_count(self, tmp_path):
        for name in (f"{TAG}_replan_x.pddl", f"{TAG}_replan_1.plan",
                     f"{TAG}_replan_1_extra.pddl"):
            (tmp_path / name).write_text("")
        assert next_index(tmp_path, TAG) == 0
