"""The narrowed goal must be reachable, not already true.

``build_runtime_problem`` narrows a mission goal to the region the executor is
being sent to. It used to narrow to ``(visited R)``. ``visited`` is monotone --
``move`` adds it, nothing retracts it -- and the find-object approach runs
*after* a coverage tour has entered every region, so the narrowed goal was
already in ``:init``. Fast Downward reported "Solution found, cost = 0" with
zero actions, ``apply_symbolic_replan`` rejected the empty plan, and the robot
never drove to the object it had just located.

These tests are written against the real ``factory_tour_*`` problems and, when
Fast Downward is present, assert on the plan FD actually returns -- the empty
plan was a planner-level fact, so checking the goal string alone would not have
caught it.
"""

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "pipeline" / "evoplan_bridge_host"))

from problem_builder import (  # noqa: E402
    EXECUTABLE_GOAL_PREDICATES,
    build_runtime_problem,
    goal_is_executable,
)

TOUR_02 = REPO / "pipeline" / "missions" / "factory_tour_02.pddl"
TOUR_01 = REPO / "pipeline" / "missions" / "factory_tour_01.pddl"
DOMAIN = REPO / "evolve_stl_pddl" / "jackal" / "in" / "factory_jackal_domain.pddl"
FD = REPO / "fast_downward" / "fast-downward.py"


def goal_of(problem_text):
    return re.search(r"\(:goal(.*?)\n  \)", problem_text, re.S).group(1)


def init_of(problem_text):
    return problem_text.split("(:goal")[0]


def approach_problem(mission, current, target, visited):
    """The problem the service builds for a post-tour object approach."""
    return build_runtime_problem(
        mission.read_text(), robot="jackal_1", current_region=current,
        blocked_regions=[], visited_regions=visited,
        target_region=target, preserve_goal=False,
    )


def fd_plan(problem_text):
    """Actions Fast Downward returns, or None when FD is unavailable."""
    if not FD.is_file():
        return None
    with tempfile.TemporaryDirectory() as tmp:
        prob = Path(tmp) / "p.pddl"
        prob.write_text(problem_text)
        plan = Path(tmp) / "p.plan"
        proc = subprocess.run(
            [sys.executable, str(FD), "--plan-file", str(plan),
             str(DOMAIN), str(prob), "--search", "astar(lmcut())"],
            cwd=tmp, capture_output=True, text=True, timeout=180,
        )
        assert "Solution found" in proc.stdout, proc.stdout[-2000:]
        if not plan.is_file():
            return []
        return [ln.strip() for ln in plan.read_text().splitlines()
                if ln.strip() and not ln.startswith(";")]


class TestNarrowedGoalIsNotAlreadySatisfied:
    """The regression proper: tour done, object located elsewhere."""

    def test_goal_is_at_not_visited(self):
        text = approach_problem(TOUR_02, current="r5", target="r1",
                                visited=["r1", "r2", "r3", "r4", "r5"])
        assert "(at jackal_1 r1)" in goal_of(text)
        assert "visited" not in goal_of(text), (
            "a (visited R) goal is already true after the tour entered R"
        )

    def test_visited_facts_are_still_asserted(self):
        """The fix must not work by hiding what the robot really did."""
        text = approach_problem(TOUR_02, current="r5", target="r1",
                                visited=["r1", "r2", "r3", "r4", "r5"])
        assert "(visited r1)" in init_of(text)

    @pytest.mark.skipif(not FD.is_file(), reason="fast_downward not present")
    def test_planner_returns_a_real_approach(self):
        """cost = 0 was the bug; the robot must actually be told to drive."""
        text = approach_problem(TOUR_02, current="r5", target="r1",
                                visited=["r1", "r2", "r3", "r4", "r5"])
        plan = fd_plan(text)
        assert plan, "empty plan -- apply_symbolic_replan rejects this"
        assert plan == ["(move jackal_1 r5 r1)"]

    @pytest.mark.skipif(not FD.is_file(), reason="fast_downward not present")
    def test_full_14_region_tour_then_approach(self):
        """tour_01: every region visited, object back at r5, robot at r14."""
        plan = fd_plan(approach_problem(
            TOUR_01, current="r14", target="r5",
            visited=[f"r{i}" for i in range(1, 15)]))
        assert plan, "empty plan"
        assert plan[-1] == "(move jackal_1 r7 r5)"
        assert all(a.startswith("(move ") for a in plan), (
            "the executor can only drive move actions"
        )


class TestCoverageToursAreStillPreserved:
    """Narrowing must not eat a tour whose goal is genuinely all-(visited)."""

    def test_tour_goal_is_executable(self):
        assert goal_is_executable(TOUR_02.read_text())

    def test_preserve_goal_none_keeps_every_region(self):
        text = build_runtime_problem(
            TOUR_02.read_text(), robot="jackal_1", current_region="r5",
            blocked_regions=[], visited_regions=["r1"],
            target_region="r1", preserve_goal=None,
        )
        goal = goal_of(text)
        for region in ("r1", "r2", "r3", "r4", "r5"):
            assert f"(visited {region})" in goal, "the tour was narrowed away"

    def test_delivery_goal_is_not_executable_and_gets_narrowed(self):
        delivery = REPO / "evolve_stl_pddl/jackal/in/factory_mission_08.pddl"
        assert not goal_is_executable(delivery.read_text())
        text = build_runtime_problem(
            delivery.read_text(), robot="jackal_1", current_region="r5",
            blocked_regions=[], visited_regions=[], target_region="r7",
            preserve_goal=None,
        )
        assert "(at jackal_1 r7)" in goal_of(text)


class TestPredicateSet:
    def test_move_achieves_every_executable_predicate(self):
        """Guards the constant against a predicate move cannot deliver."""
        effect = re.search(r":action move.*?:effect(.*?)\n  \)",
                           DOMAIN.read_text(), re.S).group(1)
        for pred in EXECUTABLE_GOAL_PREDICATES:
            assert re.search(rf"\({pred}\s", effect), (
                f"'{pred}' is listed executable but move does not assert it"
            )
