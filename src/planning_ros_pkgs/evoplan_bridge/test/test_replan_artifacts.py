"""Every replan must leave its problem on disk -- especially a failed one.

A replan that fails is exactly when the problem is worth reading, and nothing
used to persist it. An `(at jackal_1 r6)` naming a region the problem never
declared aborted Fast Downward's translator and killed a whole replan; it was
diagnosable only because that run happened to be in fd_first mode and fell
through to EvoPlan, which incidentally embedded the problem in its /tmp
workspace. An fd_only run leaves nothing.

These drive ReplanService._save_replan_artifacts directly with hand-built jobs:
the archiving contract is "given a finished job, write these files", and
standing up an HTTP service, Fast Downward and an LLM to assert it would test
everything except that.
"""

import json
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "pipeline" / "evoplan_bridge_host"))

import server as server_mod  # noqa: E402
from planners import PlanResult  # noqa: E402


TAG = "factory_tour_03_p0_evoplan_20260810_175325"


class FakeJob:
    """Shape of server.Job that _save_replan_artifacts actually reads."""

    def __init__(self, payload, result, problem_text="(define (problem p))"):
        self.id = "r-deadbeef"
        self.payload = payload
        self.result = result
        self.problem_text = problem_text


def payload(**over):
    base = {
        "tag": TAG, "mission_id": "factory_tour_03", "current_region": "r5",
        "goal": {"target_region": "r1"}, "blocked_regions": [],
        "preserve_goal": False, "planner_mode": "evoplan",
        "reason": {"trigger": "object_found"},
    }
    base.update(over)
    return base


@pytest.fixture
def svc(tmp_path, monkeypatch):
    """A service whose artifacts land in tmp_path/results/single_trials."""
    monkeypatch.setattr(server_mod, "REPO", tmp_path)
    s = object.__new__(server_mod.ReplanService)     # no __init__: no FD, no LLM
    return s


def out_dir(tmp_path):
    return tmp_path / "results" / "single_trials"


def save(svc, job):
    server_mod.ReplanService._save_replan_artifacts(svc, job)


class TestArtifactsAreWritten:
    def test_success_writes_all_three(self, svc, tmp_path):
        job = FakeJob(payload(), PlanResult(
            status="ok", plan=["(move jackal_1 r5 r1)"], planner="evoplan",
            valid=True, elapsed_s=36.374))
        save(svc, job)
        d = out_dir(tmp_path)
        assert (d / f"{TAG}_replan_1.pddl").is_file()
        assert (d / f"{TAG}_replan_1.plan").read_text() == "(move jackal_1 r5 r1)\n"
        meta = json.loads((d / f"{TAG}_replan_1.json").read_text())
        assert meta["status"] == "ok" and meta["planner"] == "evoplan"
        assert meta["job_id"] == "r-deadbeef", "must tie back to /tmp/evoplan_r-<id>"

    def test_failure_still_archives_the_problem(self, svc, tmp_path):
        """The regression that matters: a failed replan left no trace at all."""
        job = FakeJob(
            payload(current_region="r6"),
            PlanResult(status="failed", plan=[], planner="fd", valid=False,
                       error="translate exit 31"),
            problem_text="(define (problem factory_tour_03)) ;; (at jackal_1 r6)")
        save(svc, job)
        d = out_dir(tmp_path)
        assert "r6" in (d / f"{TAG}_replan_1.pddl").read_text()
        assert (d / f"{TAG}_replan_1.plan").read_text() == "", "no plan, but the file exists"
        meta = json.loads((d / f"{TAG}_replan_1.json").read_text())
        assert meta["status"] == "failed"
        assert meta["current_region"] == "r6", "the input that caused it"

    def test_numbering_increments_within_a_trial(self, svc, tmp_path):
        for _ in range(3):
            save(svc, FakeJob(payload(), PlanResult(status="ok", plan=[])))
        names = sorted(p.name for p in out_dir(tmp_path).glob("*.pddl"))
        assert names == [f"{TAG}_replan_{i}.pddl" for i in (1, 2, 3)]

    def test_numbering_resumes_from_disk(self, svc, tmp_path):
        """The service outlives trials (KEEP_SERVICE=1); a restart must not
        overwrite replan_1 with the next request."""
        d = out_dir(tmp_path)
        d.mkdir(parents=True)
        (d / f"{TAG}_replan_7.pddl").write_text("old")
        save(svc, FakeJob(payload(), PlanResult(status="ok", plan=[])))
        assert (d / f"{TAG}_replan_8.pddl").is_file()
        assert (d / f"{TAG}_replan_7.pddl").read_text() == "old"

    def test_two_trials_do_not_collide(self, svc, tmp_path):
        save(svc, FakeJob(payload(), PlanResult(status="ok", plan=[])))
        save(svc, FakeJob(payload(tag="factory_tour_03_p0_evoplan_OTHER"),
                          PlanResult(status="ok", plan=[])))
        assert (out_dir(tmp_path) / f"{TAG}_replan_1.pddl").is_file()
        assert (out_dir(tmp_path) / "factory_tour_03_p0_evoplan_OTHER_replan_1.pddl").is_file()


class TestDegradesSafely:
    def test_missing_tag_falls_back_to_mission_and_job_id(self, svc, tmp_path):
        """No tag is not a reason to discard the only record of a replan."""
        save(svc, FakeJob(payload(tag=""), PlanResult(status="ok", plan=[])))
        assert (out_dir(tmp_path) / "factory_tour_03_r-deadbeef_replan_1.pddl").is_file()

    @pytest.mark.parametrize("evil", [
        "../../../etc/passwd", "/absolute/path", "a/b/c", "..",
    ])
    def test_tag_cannot_escape_the_results_directory(self, svc, tmp_path, evil):
        """The tag arrives over HTTP; it must not be able to steer a write."""
        save(svc, FakeJob(payload(tag=evil), PlanResult(status="ok", plan=[])))
        written = list(out_dir(tmp_path).glob("*.pddl"))
        assert written, "should still archive under a sanitised name"
        for p in written:
            assert p.parent == out_dir(tmp_path)
            assert "/" not in p.name and ".." not in p.name.replace("_replan_", "")

    def test_a_write_failure_never_propagates(self, svc, tmp_path, monkeypatch):
        """Losing a diagnostic must not fail a replan the robot is waiting on."""
        def boom(*a, **k):
            raise OSError("disk full")
        monkeypatch.setattr(Path, "write_text", boom)
        save(svc, FakeJob(payload(), PlanResult(status="ok", plan=[])))  # no raise

    def test_none_problem_text_is_tolerated(self, svc, tmp_path):
        """_plan can raise before building a problem; archive the rest anyway."""
        job = FakeJob(payload(), PlanResult(status="failed", error="boom"),
                      problem_text=None)
        save(svc, job)
        assert (out_dir(tmp_path) / f"{TAG}_replan_1.pddl").read_text() == ""
        assert json.loads((out_dir(tmp_path) / f"{TAG}_replan_1.json").read_text())["error"] == "boom"


class TestEvoplanOnlySkipsFastDownward:
    """`evoplan_only` exists because the FD pre-check rejects the case the
    prefix is for: a goal naming an unlocated object is unsolvable, and FD
    saying so correctly is not a useful answer. Everywhere else that check
    stays -- handing an impossible problem to an LLM burns the whole budget.
    """

    def _service(self, monkeypatch, calls):
        monkeypatch.setattr(server_mod, "run_fast_downward",
                            lambda *a, **k: calls.append(a) or PlanResult(status="unsolvable"))
        monkeypatch.setattr(server_mod.ReplanService, "_run_evoplan",
                            lambda self, job, d, p, *a, **k: PlanResult(
                                status="ok", plan=["(move jackal_1 r5 r1)"], planner="evoplan"))
        s = object.__new__(server_mod.ReplanService)
        s.missions = server_mod.MissionLibrary(
            Path(server_mod.REPO) / "evolve_stl_pddl" / "jackal" / "in",
            extra_dirs=[Path(server_mod.REPO) / "pipeline" / "missions"])
        s.fd_path = Path("/nonexistent/fast-downward.py")
        s.args = SimpleNamespace(fd_timeout_s=10.0)
        s.mock_plan = None
        # _plan consults observations reported mid-drive via /observation.
        s.lock = threading.Lock()
        s.observations = {}
        return s

    def job(self, mode):
        return FakeJob({"mission_id": "factory_tour_03", "robot": "jackal_1",
                        "planner_mode": mode, "deadline_s": 60.0,
                        "goal": {"target_region": "r5"}, "preserve_goal": True,
                        "blocked_regions": [], "executed_plan": []},
                       PlanResult(status="running"))

    def test_evoplan_only_never_calls_fast_downward(self, monkeypatch):
        calls = []
        svc = self._service(monkeypatch, calls)
        result = server_mod.ReplanService._plan(svc, self.job("evoplan_only"))
        assert calls == [], "fast downward was consulted"
        assert result.status == "ok" and result.planner == "evoplan"

    def test_other_modes_still_call_it(self, monkeypatch):
        """The guard must not be lost for the modes that depend on it."""
        calls = []
        svc = self._service(monkeypatch, calls)
        server_mod.ReplanService._plan(svc, self.job("evoplan"))
        assert calls, "the unsolvability pre-check was skipped"

    def test_the_problem_is_still_archived(self, monkeypatch):
        """No FD result means no fallback plan; the problem is the only record."""
        job = self.job("evoplan_only")
        svc = self._service(monkeypatch, [])
        server_mod.ReplanService._plan(svc, job)
        assert job.problem_text and "location-unknown cone" in job.problem_text
