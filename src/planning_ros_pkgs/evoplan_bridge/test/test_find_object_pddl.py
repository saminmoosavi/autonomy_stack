"""The object being searched for must exist in the PDDL, not just in Python.

Before this, the find-object mission was invisible to the planner. Phase 1 was
a region-coverage tour, and phase 2 narrowed the goal to ``(at jackal_1 r1)``
-- a region name computed by ``begin_object_approach`` from observation memory
and handed over as a bare string. Nothing in the problem said an object
existed, which region held it, or why r1 rather than r4. The LLM reads that
problem; it was being asked to solve a navigation puzzle with the motivating
fact removed.

The model these tests pin down:

* the mission declares ``cone - target`` and asserts ``(location-unknown cone)``;
* ``approach`` requires ``(not (location-unknown ?t))``, so declaring the cone
  cannot make phase 1 solvable by shortcut -- the tour still has to happen;
* on discovery the replan retracts the unknown flag, asserts
  ``(object-at cone <region>)`` and sets the goal to ``(reached jackal_1 cone)``,
  and the PLANNER derives the region.

They run Fast Downward wherever the claim is about what a planner does, since
every bug this replaces (cost-0 plans, translator aborts, unsolvable goals) was
a planner-level fact that inspecting the goal string would have missed.
"""

import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "pipeline" / "evoplan_bridge_host"))

from pddl_splice import remove_init_fact, set_goal, set_robot_location  # noqa: E402
from plan_prefix import executable_prefix  # noqa: E402
from problem_builder import (  # noqa: E402
    build_runtime_problem,
    resolve_target_object,
    targets_in_problem,
)
from evoplan_bridge.symbolic_replan import (  # noqa: E402
    filter_executable_actions,
    plan_reaches_target,
)

TOUR_03 = REPO / "pipeline" / "missions" / "factory_tour_03.pddl"
TOUR_02 = REPO / "pipeline" / "missions" / "factory_tour_02.pddl"
TOUR_01 = REPO / "pipeline" / "missions" / "factory_tour_01.pddl"
PLAN_03 = REPO / "factory_mission_plans" / "factory_tour_03.txt"
DELIVERY = REPO / "evolve_stl_pddl" / "jackal" / "in" / "factory_mission_08.pddl"
DOMAIN = REPO / "evolve_stl_pddl" / "jackal" / "in" / "factory_jackal_domain.pddl"
FD = REPO / "fast_downward" / "fast-downward.py"

needs_fd = pytest.mark.skipif(not FD.is_file(), reason="fast_downward not present")


@pytest.fixture(scope="module")
def val():
    """`validate_with_val`, with VAL on PATH -- replan_service.sh puts it there.

    VAL ships in-repo at .local/bin rather than being installed, so a plain
    pytest run does not see it and validate_with_val reports every plan invalid.
    """
    from planners import val_binary, validate_with_val

    local_bin = REPO / ".local" / "bin"
    os.environ["PATH"] = f"{local_bin}{os.pathsep}{os.environ['PATH']}"
    if val_binary() is None:
        pytest.skip("VAL not present in .local/bin")

    def run(problem_text, plan_lines):
        return validate_with_val(DOMAIN.read_text(), problem_text, plan_lines)

    return run


class Action:
    """Duck type of pddl_stl.pipeline.GroundAction, parsed from a plan line."""

    def __init__(self, line):
        tokens = line.strip().strip("()").split()
        self.name, self.args = tokens[0], tuple(tokens[1:])


def run_fd(problem_text):
    """(status, [plan lines]) from a real Fast Downward run."""
    with tempfile.TemporaryDirectory() as tmp:
        problem = Path(tmp) / "p.pddl"
        problem.write_text(problem_text)
        plan = Path(tmp) / "p.plan"
        proc = subprocess.run(
            [sys.executable, str(FD), "--plan-file", str(plan),
             str(DOMAIN), str(problem), "--search", "astar(lmcut())"],
            cwd=tmp, capture_output=True, text=True, timeout=300,
        )
        if "Solution found" not in proc.stdout:
            status = ("unsolvable" if "unsolvable" in proc.stdout.lower()
                      else f"failed: {proc.stdout[-400:]}")
            return status, []
        lines = [ln.strip() for ln in plan.read_text().splitlines()
                 if ln.strip() and not ln.startswith(";")]
        return "ok", lines


def code(text):
    """PDDL with ';;' comments stripped.

    Not optional here: these missions explain the find-object mechanism in
    prose, so the header names (location-unknown cone) and (object-at ...)
    literally. Asserting against the raw text passes and fails for reasons that
    have nothing to do with the facts.
    """
    return re.sub(r";[^\n]*", "", text)


def goal_of(text):
    return re.search(r"\(:goal(.*)", code(text), re.S).group(1)


def init_of(text):
    return code(text).split("(:goal")[0]


def approach(mission=TOUR_03, current="r5", region="r1", cls="traffic cone",
             visited=("r1", "r2", "r5"), preserve_goal=False):
    """The problem the service builds once perception has located the object."""
    return build_runtime_problem(
        mission.read_text(), robot="jackal_1", current_region=current,
        blocked_regions=[], visited_regions=list(visited),
        target_region=region, preserve_goal=preserve_goal,
        found_object={"class": cls, "region": region},
    )


class TestPhaseOneAsksForMoreThanItCanGet:
    """The mission goal names the cone, and phase 1 is unsolvable on purpose.

    Classical planning cannot sense, so (reached jackal_1 cone) over an object
    with no known position has no achiever. Rather than weaken the goal, the
    caller takes the plan's executable PREFIX -- the survey of every region the
    cone could be in. These tests pin both halves: the problem really is
    unsolvable, and the prefix really is the survey.

    A previous revision added a `search-for` action letting the planner assume
    the discovery. It made the goal reachable and left nothing to truncate, so
    it was removed; `test_no_action_can_invent_the_objects_position` is what
    stops it coming back by accident.
    """

    def test_mission_declares_a_target(self):
        assert targets_in_problem(TOUR_03.read_text()) == {"cone"}

    def test_location_is_unknown_before_the_tour(self):
        assert "(location-unknown cone)" in init_of(TOUR_03.read_text())

    def test_goal_names_the_cone_alongside_the_coverage(self):
        goal = goal_of(TOUR_03.read_text())
        assert "(reached jackal_1 cone)" in goal
        for region in ("r1", "r2", "r5"):
            assert f"(visited {region})" in goal, "coverage must survive"

    def test_no_action_can_invent_the_objects_position(self):
        """Only the runtime may assert (object-at ...). If a domain action ever
        adds it, the goal becomes reachable by assumption and the prefix
        silently stops being a prefix of anything.

        Scanned comment-stripped: `approach`'s banner explains the mechanism in
        prose and names (object-at ?t ?l) literally.
        """
        domain = code(DOMAIN.read_text())
        actions = re.split(r"\(:action ", domain)[1:]
        assert len(actions) >= 5, "action split failed; the check would be vacuous"
        for block in actions:
            name = block.split()[0]
            effect = re.search(r":effect(.*)", block, re.S)
            assert not re.search(r"\(object-at\s", effect.group(1)), (
                f"action '{name}' asserts (object-at ...)")

    @needs_fd
    def test_phase_one_is_unsolvable(self):
        status, plan = run_fd(TOUR_03.read_text())
        assert status == "unsolvable", f"got {status} with plan {plan}"

    def test_the_prefix_of_a_survey_is_the_whole_survey(self):
        """Every move is applicable; only the goal goes unmet."""
        survey = ["(move jackal_1 r5 r1)", "(move jackal_1 r1 r2)",
                  "(move jackal_1 r2 r5)"]
        r = executable_prefix(DOMAIN.read_text(), TOUR_03.read_text(), survey)
        assert r.plan == survey
        assert not r.truncated
        assert not r.reaches_goal, "the cone conjunct is still open"

    def test_the_prefix_stops_at_the_cone(self):
        """The action the caller must not drive, and the reason it cannot."""
        r = executable_prefix(DOMAIN.read_text(), TOUR_03.read_text(), [
            "(move jackal_1 r5 r1)", "(move jackal_1 r1 r2)",
            "(move jackal_1 r2 r5)", "(approach jackal_1 cone r5)"])
        assert r.prefix_len == 3 and r.truncated
        assert any("location-unknown" in e for e in r.errors)

    def test_the_shipped_plan_runs_past_what_it_can_achieve(self):
        """What the executor loads at startup. It is authored to overshoot --
        survey, then act on the cone, then move on -- so that the executor's
        truncation is exercised on every run rather than only when a planner
        happens to return something unachievable."""
        plan = [ln.strip() for ln in PLAN_03.read_text().splitlines() if ln.strip()]
        r = executable_prefix(DOMAIN.read_text(), TOUR_03.read_text(), plan)
        assert r.truncated, "nothing to truncate means the prefix is untested"
        assert plan[r.prefix_len].startswith("(approach ")
        assert all(a.startswith("(move ") for a in r.plan)
        assert {a.split()[-1].rstrip(")") for a in r.plan} == {"r1", "r2", "r5"}, (
            "the surviving prefix must still cover every region in the goal")


class TestDiscoveryRewritesTheProblem:
    def test_goal_names_the_object_not_the_region(self):
        text = approach()
        assert "(reached jackal_1 cone)" in goal_of(text)
        assert "(at jackal_1 r1)" not in goal_of(text), (
            "the region is now derived by the planner, not asserted as the goal"
        )

    def test_the_discovered_position_becomes_a_fact(self):
        assert "(object-at cone r1)" in init_of(approach())

    def test_the_unknown_flag_is_retracted(self):
        """Left in place it is `approach`'s negative precondition -- the plan
        would be unsolvable with the object sitting in plain sight."""
        assert "location-unknown" not in init_of(approach())

    def test_visited_facts_still_survive(self):
        assert "(visited r2)" in init_of(approach())

    @needs_fd
    def test_planner_derives_the_region_from_object_at(self):
        status, plan = run_fd(approach())
        assert status == "ok"
        assert plan == ["(move jackal_1 r5 r1)", "(approach jackal_1 cone r1)"]

    @needs_fd
    def test_object_in_a_different_region_moves_the_plan(self):
        """One fact changes; the goal text does not. That is the point."""
        status, plan = run_fd(approach(current="r5", region="r2"))
        assert status == "ok"
        assert plan[0] == "(move jackal_1 r5 r2)"
        assert plan[-1] == "(approach jackal_1 cone r2)"

    @needs_fd
    def test_five_region_tour_then_approach(self):
        status, plan = run_fd(approach(
            mission=TOUR_02, current="r3", region="r1",
            visited=("r1", "r2", "r3", "r4", "r5")))
        assert status == "ok"
        assert plan[-1] == "(approach jackal_1 cone r1)"


class TestTheExecutorCanStillDriveTheResult:
    """A goal the planner likes is worthless if apply_symbolic_replan drops it.

    `approach` has no executor -- only `move` lowers to a Nav2 goal -- so the
    returned plan now contains an action the executor must discard. Both guards
    it runs afterwards have to survive that.
    """

    @needs_fd
    def test_only_the_approach_action_is_dropped(self):
        _, plan = run_fd(approach())
        executable, dropped = filter_executable_actions([Action(a) for a in plan])
        assert [a.name for a in executable] == ["move"]
        assert [a.name for a in dropped] == ["approach"]

    @needs_fd
    def test_plan_reaches_target_still_holds(self):
        """target_region is what the executor tracks; the move chain must end
        there even though the goal no longer mentions it."""
        _, plan = run_fd(approach(region="r1"))
        executable, _ = filter_executable_actions([Action(a) for a in plan])
        assert plan_reaches_target(executable, "r1")

    @needs_fd
    def test_stripping_the_approach_never_empties_the_plan(self):
        """A move-free plan is rejected outright, which would strand the robot."""
        _, plan = run_fd(approach(current="r5", region="r2"))
        executable, _ = filter_executable_actions([Action(a) for a in plan])
        assert executable


class TestValAcceptsTheApproachPlan:
    """VAL gates every returned plan, so a goal FD solves but VAL rejects is a
    plan the service throws away after paying the full LLM deliberation cost."""

    @needs_fd
    def test_val_accepts_the_full_plan(self, val):
        _, plan = run_fd(approach())
        valid, output = val(approach(), plan)
        assert valid, output[-800:]

    @needs_fd
    def test_val_rejects_the_plan_the_executor_will_actually_drive(self, val):
        """A caveat, pinned rather than fixed.

        The executor drops `approach` (only `move` lowers to a Nav2 goal), so
        the action sequence that really runs no longer entails
        (reached jackal_1 cone). Harmless today -- VAL scores the plan the
        SERVICE returns, which still has the approach in it, and arrival is
        judged geometrically by at_target_region -- but it means the symbolic
        goal and the executed trace have diverged by one action, and anything
        that later validates the trace must know that.
        """
        problem = approach()
        _, plan = run_fd(problem)
        moves = [a for a in plan if a.startswith("(move")]
        assert not val(problem, moves)[0], (
            "if this passes, `approach` has become inert and the goal could be "
            "stated over moves alone"
        )


class TestFallsBackWhenTheObjectCannotBeModelled:
    """Every mission predating this declares no target; none may regress."""

    def test_mission_without_a_target_keeps_the_region_goal(self):
        text = build_runtime_problem(
            DELIVERY.read_text(), robot="jackal_1", current_region="r5",
            blocked_regions=[], visited_regions=[], target_region="r7",
            preserve_goal=False,
            found_object={"class": "traffic cone", "region": "r7"})
        assert "(at jackal_1 r7)" in goal_of(text)
        assert "reached" not in goal_of(text)

    def test_undeclared_object_region_falls_back(self):
        """r9 is outside tour_03's R1/R2/R5. Asserting (object-at cone r9)
        would abort FD's translator on an undeclared object -- the same exit-31
        failure that killed a whole replan via a drifted pose."""
        text = approach(region="r9")
        assert "object-at" not in code(text)
        assert "(at jackal_1 r9)" in goal_of(text)

    def test_no_found_object_is_the_old_behaviour(self):
        text = build_runtime_problem(
            TOUR_03.read_text(), robot="jackal_1", current_region="r5",
            blocked_regions=[], visited_regions=["r1"], target_region="r1",
            preserve_goal=False)
        assert "(at jackal_1 r1)" in goal_of(text)
        assert "(location-unknown cone)" in init_of(text), "nothing discovered yet"

    def test_preserve_goal_keeps_the_tour_and_leaves_the_object_alone(self):
        """A mid-tour replan (obstacle, not discovery) must not retarget.

        The preserved goal legitimately contains (reached jackal_1 cone) now --
        it is the mission's own goal. What must not happen is the discovery
        splice firing: no (object-at ...) fact, and (location-unknown cone)
        still standing, because nothing has been found yet.
        """
        text = approach(preserve_goal=True)
        assert "(visited r2)" in goal_of(text)
        assert "(reached jackal_1 cone)" in goal_of(text)
        assert "object-at" not in code(text)
        assert "(location-unknown cone)" in init_of(text)


class TestResolveTargetObject:
    """The detector says "traffic cone"; the PDDL says `cone`. Neither should
    have to know about the other, so the symbol is resolved from the problem."""

    def test_exact_slug_match_wins(self):
        problem = TOUR_03.read_text().replace("cone      - target",
                                              "traffic_cone - target\n    box_x - target")
        assert resolve_target_object(problem, "traffic cone") == "traffic_cone"

    def test_last_word_match_when_the_slug_does_not_exist(self):
        problem = TOUR_03.read_text().replace("cone      - target",
                                              "cone - target\n    box_x - target")
        assert resolve_target_object(problem, "traffic cone") == "cone"

    def test_sole_target_wins_without_any_name_match(self):
        """A mission declaring one findable object has nothing to disambiguate,
        so a class rename must not silently disable the whole mechanism."""
        assert resolve_target_object(TOUR_03.read_text(), "backpack") == "cone"

    def test_ambiguous_declaration_refuses_to_guess(self):
        problem = TOUR_03.read_text().replace("cone      - target",
                                              "pallet - target\n    crate - target")
        assert resolve_target_object(problem, "backpack") is None

    def test_no_targets_declared(self):
        assert resolve_target_object(DELIVERY.read_text(), "traffic cone") is None


class TestProseAboutPddlIsInert:
    """A mission file must not be broken by its own documentation.

    These files are written to be read -- they are the seed EvoPlan hands the
    LLM -- so headers explain the mechanism in prose and name the blocks they
    describe. find_block searched the raw text, so a header sentence containing
    "(:init ...)" was matched as the block itself; paren counting then started
    inside the comment and every splice failed with the misleading
    "no (at jackal_1 ...) fact in (:init ...)".
    """

    @pytest.mark.parametrize("mission", [TOUR_01, TOUR_02, TOUR_03, DELIVERY])
    def test_real_missions_still_splice(self, mission):
        text = build_runtime_problem(
            mission.read_text(), robot="jackal_1", current_region="r5",
            blocked_regions=[], visited_regions=["r5"], target_region="r5",
            preserve_goal=True)
        assert "(at jackal_1 r5)" in init_of(text).lower()

    def test_a_header_naming_init_does_not_capture_the_block(self):
        text = (";; Prose about (:init ...) and its (at jackal_1 r9) fact.\n"
                "(define (problem p) (:init (at jackal_1 r5)) (:goal (x)))")
        assert set_robot_location(text, "jackal_1", "r1").endswith(
            "(define (problem p) (:init (at jackal_1 r1)) (:goal (x)))")

    def test_a_header_naming_goal_does_not_capture_the_block(self):
        text = (";; The goal is (:goal (visited r9)) in phase one.\n"
                "(define (problem p) (:init (a)) (:goal (visited r5)))")
        assert "(reached jackal_1 cone)" in set_goal(text, "(reached jackal_1 cone)")
        assert "(visited r9)" in set_goal(text, "(reached jackal_1 cone)"), (
            "the comment must be left exactly as written"
        )


class TestRemoveInitFact:
    def test_removes_only_from_init(self):
        text = ("(define (problem p) (:init (location-unknown cone) (at r x))\n"
                "  (:goal (location-unknown cone)))")
        out = remove_init_fact(text, "location-unknown", "cone")
        assert out.count("location-unknown") == 1
        assert "(:goal (location-unknown cone))" in out

    def test_is_case_insensitive(self):
        """Missions author `Cone`; runtime facts arrive lowercased. PDDL does
        not distinguish, and a missed retraction is an unsolvable problem."""
        text = "(define (problem p) (:init (Location-Unknown CONE)) (:goal (x)))"
        assert "unknown" not in remove_init_fact(text, "location-unknown", "cone").lower()

    def test_a_trailing_comment_goes_with_the_fact(self):
        """Left behind, it annotates a fact that is no longer there -- in the
        very text the LLM is asked to reason from."""
        text = ("(define (problem p) (:init\n"
                "    (location-unknown cone)   ;; retracted on discovery\n"
                "    (at jackal_1 r5)) (:goal (x)))")
        out = remove_init_fact(text, "location-unknown", "cone")
        assert "retracted on discovery" not in out
        assert "(at jackal_1 r5)" in out

    def test_a_comment_on_its_own_line_is_left_alone(self):
        """It may describe the facts that FOLLOW. Eating it on a guess is how a
        splice destroys something load-bearing."""
        text = ("(define (problem p) (:init\n"
                "    ;; Connectivity below\n"
                "    (location-unknown cone)\n"
                "    (connected r1 r2)) (:goal (x)))")
        out = remove_init_fact(text, "location-unknown", "cone")
        assert ";; Connectivity below" in out
        assert "location-unknown" not in out

    def test_a_neighbouring_fact_on_the_same_line_survives(self):
        text = "(define (problem p) (:init (location-unknown cone) (at r x)) (:goal (y)))"
        out = remove_init_fact(text, "location-unknown", "cone")
        assert "(at r x)" in out and "location-unknown" not in out

    def test_absent_fact_is_a_no_op(self):
        text = "(define (problem p) (:init (at jackal_1 r5)) (:goal (x)))"
        assert remove_init_fact(text, "location-unknown", "cone") == text
