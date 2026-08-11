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

from evoplan_bridge import symbolic_replan  # noqa: E402
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


class TestUndeclaredStartRegion:
    """An undeclared (at robot R) aborts FD's TRANSLATOR -- no search, no plan.

    Live failure: a drifted odom-frame pose reported current_region=r6 while
    the robot sat at r5. tour_02 declares only R1-R5, so the problem carried
    `(at jackal_1 r6)`, FD aborted with translate exit 31, fd_first fell
    through to the LLM, and the 45 s deadline expired with the cone already
    located and the robot never sent to it.
    """

    def test_undeclared_start_is_dropped(self):
        text = approach_problem(TOUR_02, current="r6", target="r1",
                                visited=["r1", "r2", "r3", "r4", "r5"])
        # PDDL is case-insensitive and the authored problem uses `R5`, so
        # compare case-folded rather than assuming the writer's casing.
        init = init_of(text).lower()
        assert "(at jackal_1 r6)" not in init, "undeclared start must not be spliced"
        assert "(at jackal_1 r5)" in init, "should keep the problem's authored start"

    def test_declared_start_is_still_applied(self):
        text = approach_problem(TOUR_02, current="r3", target="r1",
                                visited=["r1", "r2", "r3"])
        assert "(at jackal_1 r3)" in init_of(text)

    @pytest.mark.skipif(not FD.is_file(), reason="fast_downward not present")
    def test_planner_still_solves_with_an_undeclared_start(self):
        """The point of dropping it: degrade to a solvable problem, not abort."""
        plan = fd_plan(approach_problem(
            TOUR_02, current="r6", target="r1",
            visited=["r1", "r2", "r3", "r4", "r5"]))
        assert plan, "translator aborted -- the whole replan fails"
        assert plan[-1].endswith("r1)")


class TestPredicateSet:
    """Guards the constant against a predicate the robot cannot actually deliver.

    "Executable" means: a plan achieving this goal drives the robot to where
    the goal wants it. Three ways to qualify. Most predicates qualify directly
    -- `move` asserts them, and `move` was long the only action with an
    executor. Some qualify because the actions that assert them are pure
    bookkeeping that filter_executable_actions drops, so the goal is still
    discharged by the move chain and nothing is silently skipped. The last
    group qualifies most strongly of all: a non-move action the executor
    genuinely performs.

    A predicate satisfying none of the three must NOT be listed: goals are
    narrowed when they are deemed non-executable, and wrongly calling one
    executable means a plan ending at a dropoff the robot cannot perform is
    accepted as reaching the mission endpoint.
    """

    #: Asserted only by actions with no executor, which are dropped before
    #: driving. `approach` requires (at ?r ?l) for the object's own region, so
    #: reaching it still means the robot drove there.
    BOOKKEEPING_ONLY = {"reached"}

    #: Asserted by a non-move action the executor DOES perform, mapped to the
    #: action that performs it. plan_to_nav2_goals lowers `inspect-object` to a
    #: goal pose facing the object plus a stationary dwell, so a plan achieving
    #: (inspected-object ?t) has the robot drive there AND hold a view of it.
    DRIVEN_BY_ACTION = {"inspected-object": "inspect-object"}

    def test_move_achieves_every_directly_executable_predicate(self):
        effect = re.search(r":action move.*?:effect(.*?)\n  \)",
                           DOMAIN.read_text(), re.S).group(1)
        direct = (EXECUTABLE_GOAL_PREDICATES - self.BOOKKEEPING_ONLY
                  - set(self.DRIVEN_BY_ACTION))
        for pred in direct:
            assert re.search(rf"\({pred}\s", effect), (
                f"'{pred}' is listed executable but move does not assert it"
            )

    @pytest.mark.parametrize("pred,action", sorted(DRIVEN_BY_ACTION.items()))
    def test_driven_predicates_are_asserted_by_an_executed_action(self, pred, action):
        """The exemption holds only while the executor still lowers the action.

        Two halves, and both matter: the domain must have `action` assert
        `pred`, and symbolic_replan must still count `action` as executable. If
        the executor ever stops driving it, the predicate becomes bookkeeping
        that no longer drives the robot anywhere, and listing it executable
        would mean a goal built from it is accepted while nothing happens.
        """
        domain = DOMAIN.read_text()
        body = re.search(rf":action {re.escape(action)}(.*?)(?=\n  \(:action|\n\)$)",
                         domain, re.S)
        assert body, f"the domain has no ({action} ...)"
        assert re.search(rf":effect.*?\({pred}\s", body.group(1), re.S), (
            f"({pred} ...) is not asserted by {action}"
        )
        assert action.startswith(symbolic_replan.EXECUTABLE_ACTION_PREFIXES), (
            f"{action} is no longer lowered by the executor, so ({pred} ...) "
            f"cannot be treated as executable"
        )

    @pytest.mark.parametrize("pred", sorted(BOOKKEEPING_ONLY))
    def test_bookkeeping_predicates_are_dropped_not_driven(self, pred):
        """If something the executor DOES drive ever starts asserting these,
        the exemption is no longer sound and this should fail."""
        domain = DOMAIN.read_text()
        asserting = [m.group(1) for m in
                     re.finditer(r":action ([a-z0-9-]+)(.*?)(?=\n  \(:action|\n\)$)",
                                 domain, re.S)
                     if re.search(rf":effect.*?\({pred}\s", m.group(2), re.S)]
        assert asserting, f"nothing asserts ({pred} ...); remove it from the set"
        assert not any(a.startswith("move") for a in asserting), (
            f"({pred} ...) is asserted by {asserting}, which the executor DRIVES"
        )
