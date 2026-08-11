"""The open-world mission: objects the problem never declared.

Every other mission in this repo names its objects. This one cannot -- how many
traffic cones are in the factory is the question the robot was sent to answer --
so the object set arrives from perception and is spliced into the problem as it
grows. That inverts an assumption classical planning makes everywhere else, and
these tests pin the consequences:

* a problem declaring no targets is still SOLVABLE (it is a survey), unlike the
  deliberately-unsolvable find-an-object tours;
* discovered objects reach the planner as real declarations, facts and goal
  conjuncts, not as a region string computed in Python;
* the mission's own goal survives the splice, so a replan issued mid-survey
  finishes the sweep instead of abandoning it for the first cone found;
* re-splicing is a no-op, because the same objects are sent on every replan and
  a duplicated declaration is a PDDL error rather than a duplicate line.

Fast Downward runs wherever the claim is about what a planner does. Every bug
worth catching here (translator aborts on undeclared names, cost-0 plans,
unsolvable goals) is invisible to string inspection.
"""

import re
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "pipeline" / "evoplan_bridge_host"))
sys.path.insert(0, str(REPO / "src" / "planning_ros_pkgs" / "evoplan_bridge"))

from evoplan_bridge.symbolic_replan import (  # noqa: E402
    EXECUTABLE_ACTION_PREFIXES,
    filter_executable_actions,
    plan_end_region,
)
from pddl_splice import goal_conjuncts  # noqa: E402
from problem_builder import (  # noqa: E402
    build_runtime_problem,
    goal_is_executable,
    objects_in_problem,
    splice_inspection_targets,
    targets_in_problem,
)

SURVEY = REPO / "pipeline" / "missions" / "factory_survey_01.pddl"
SURVEY_PLAN = REPO / "factory_mission_plans" / "factory_survey_01.txt"
#: The five-region survey (R1-R5). Used where a test needs r3/r4 -- survey_01
#: declares only R1/R2/R5, and build_runtime_problem correctly DROPS an object
#: in an undeclared region, which silently empties a fixture that assumes one.
SURVEY_5 = REPO / "pipeline" / "missions" / "factory_survey_02.pddl"
DOMAIN = REPO / "evolve_stl_pddl" / "jackal" / "in" / "factory_jackal_domain.pddl"
FD = REPO / "fast_downward" / "fast-downward.py"

needs_fd = pytest.mark.skipif(not FD.is_file(), reason="fast_downward not present")


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
    """PDDL with ';;' comments stripped -- the mission explains itself in prose
    that names the very facts under test."""
    return re.sub(r";[^\n]*", "", text)


class Action:
    """Duck type of pddl_stl.pipeline.GroundAction, parsed from a plan line."""

    def __init__(self, line):
        tokens = line.strip().strip("()").split()
        self.name, self.args = tokens[0], tuple(tokens[1:])


def found(*pairs):
    """``(name, region)`` or ``(name, region, inspected)`` -> instance dicts."""
    return [{"name": p[0], "region": p[1], "class": "traffic cone",
             "inspected": p[2] if len(p) > 2 else False}
            for p in pairs]


class TestTheAuthoredMission:
    def test_declares_no_targets(self):
        """The premise. A declared object would be an invented one."""
        assert targets_in_problem(SURVEY.read_text()) == set()

    def test_says_nothing_about_objects_at_all(self):
        body = code(SURVEY.read_text())
        assert "object-at" not in body
        assert "location-unknown" not in body
        assert "inspected-object" not in body

    @needs_fd
    def test_is_solvable_as_written(self):
        """Unlike the find-an-object tours, which are unsolvable on purpose:
        there is no object in this problem to be unable to reach, so the survey
        is a plan in its own right rather than a prefix of a broken one."""
        status, plan = run_fd(SURVEY.read_text())
        assert status == "ok"
        assert plan and all(ln.startswith("(move") for ln in plan)

    @needs_fd
    def test_the_shipped_plan_matches_what_a_planner_produces(self):
        """The executor drives factory_survey_01.txt, not the .pddl. A drift
        between them is a mission that surveys something other than it says."""
        status, plan = run_fd(SURVEY.read_text())
        assert status == "ok"
        shipped = [ln.strip() for ln in SURVEY_PLAN.read_text().splitlines()
                   if ln.strip() and not ln.strip().startswith(";")]
        assert shipped == plan


class TestSplicingDiscoveries:
    def test_objects_are_declared_with_their_positions(self):
        text = splice_inspection_targets(
            SURVEY.read_text(), found(("traffic_cone_1", "r1"),
                                      ("traffic_cone_2", "r2")))
        assert objects_in_problem(text)["traffic_cone_1"] == "target"
        assert "(object-at traffic_cone_1 r1)" in code(text)
        assert "(object-at traffic_cone_2 r2)" in code(text)

    def test_every_object_gets_a_goal_conjunct(self):
        text = splice_inspection_targets(
            SURVEY.read_text(), found(("traffic_cone_1", "r1"),
                                      ("traffic_cone_2", "r2")))
        goals = goal_conjuncts(text)
        assert "(inspected-object traffic_cone_1)" in goals
        assert "(inspected-object traffic_cone_2)" in goals

    def test_the_missions_own_goal_survives(self):
        """A discovery must not cancel the search. The robot may still have
        regions to cover when the first object turns up."""
        text = splice_inspection_targets(SURVEY.read_text(),
                                         found(("traffic_cone_1", "r1")))
        goals = goal_conjuncts(text)
        assert {"(visited r1)", "(visited r2)", "(visited r5)"} <= set(goals)

    def test_re_splicing_changes_nothing(self):
        """Every replan sends the whole registry, so this runs repeatedly. A
        redeclared object is a PDDL error, not a cosmetic repeat."""
        once = splice_inspection_targets(SURVEY.read_text(),
                                         found(("traffic_cone_1", "r1")))
        twice = splice_inspection_targets(once, found(("traffic_cone_1", "r1")))
        assert code(twice).count("traffic_cone_1 - target") == 1
        assert goal_conjuncts(twice) == goal_conjuncts(once)

    def test_a_moved_object_replaces_its_position(self):
        once = splice_inspection_targets(SURVEY.read_text(),
                                         found(("traffic_cone_1", "r1")))
        moved = splice_inspection_targets(once, found(("traffic_cone_1", "r2")))
        assert "(object-at traffic_cone_1 r2)" in code(moved)
        assert "(object-at traffic_cone_1 r1)" not in code(moved)

    def test_an_object_in_an_undeclared_region_is_dropped(self):
        """An undeclared name aborts FD's translator (exit 31) and kills the
        whole replan; dropping one sighting keeps the rest."""
        text = build_runtime_problem(
            SURVEY.read_text(), robot="jackal_1", current_region="r5",
            inspect_objects=found(("traffic_cone_1", "r1"),
                                  ("traffic_cone_9", "r99")))
        assert "traffic_cone_9" not in objects_in_problem(text)
        assert "traffic_cone_1" in objects_in_problem(text)

    def test_nothing_found_leaves_the_problem_alone(self):
        text = build_runtime_problem(SURVEY.read_text(), robot="jackal_1",
                                     current_region="r5", inspect_objects=[])
        assert targets_in_problem(text) == set()
        assert goal_conjuncts(text) == goal_conjuncts(SURVEY.read_text())

    def test_the_goal_is_not_narrowed_away(self):
        """`inspected-object` must count as executable: a non-executable goal is
        NARROWED to a single (at ...), which would drop every inspection."""
        text = build_runtime_problem(
            SURVEY.read_text(), robot="jackal_1", current_region="r5",
            inspect_objects=found(("traffic_cone_1", "r1")))
        assert goal_is_executable(text)


class TestFinishedWorkIsRecorded:
    """A goal conjunct with no matching `:init` fact means "not done yet".

    The goal grows an `(inspected-object ?t)` per object and never loses one, so
    a problem that omits the completed ones is telling the planner those goals
    are still open. It then plans to satisfy them again -- correctly. A live run
    produced a six-action plan re-inspecting two finished cones and driving back
    across the map for one of them, which became the mission's target region and
    left it reporting incomplete. Fast Downward, handed the same problem,
    produced the same plan: the defect was in the problem, not the planner.
    """

    def test_completed_objects_are_asserted_in_init(self):
        text = splice_inspection_targets(
            SURVEY.read_text(), found(("traffic_cone_1", "r1", True),
                                      ("traffic_cone_2", "r2", False)))
        body = code(text)
        assert "(inspected-object traffic_cone_1)" in body
        # The pending one must NOT be asserted, or the mission is over by fiat.
        init = body[body.index("(:init"):body.index("(:goal")]
        assert "(inspected-object traffic_cone_2)" not in init

    def test_the_goal_still_names_every_object(self):
        """Recording the work does not remove the requirement."""
        text = splice_inspection_targets(
            SURVEY.read_text(), found(("traffic_cone_1", "r1", True),
                                      ("traffic_cone_2", "r2", False)))
        goals = goal_conjuncts(text)
        assert "(inspected-object traffic_cone_1)" in goals
        assert "(inspected-object traffic_cone_2)" in goals

    def test_re_splicing_does_not_duplicate_the_fact(self):
        once = splice_inspection_targets(SURVEY.read_text(),
                                         found(("traffic_cone_1", "r1", True)))
        twice = splice_inspection_targets(once,
                                          found(("traffic_cone_1", "r1", True)))
        assert code(twice).count("(inspected-object traffic_cone_1)") == \
               code(once).count("(inspected-object traffic_cone_1)")

    def test_the_fact_tracks_the_registry_and_is_not_sticky(self):
        """Built on a base that already carries it, but reported as pending:
        the splice must follow the executor rather than the stale file."""
        done = splice_inspection_targets(SURVEY.read_text(),
                                         found(("traffic_cone_1", "r1", True)))
        again = splice_inspection_targets(done,
                                          found(("traffic_cone_1", "r1", False)))
        body = code(again)
        init = body[body.index("(:init"):body.index("(:goal")]
        assert "(inspected-object traffic_cone_1)" not in init

    @needs_fd
    def test_the_planner_stops_re_inspecting(self):
        """The whole point, measured in actions: 3 objects, 2 already done."""
        objects = found(("traffic_cone_1", "r1", True),
                        ("traffic_cone_2", "r3", True),
                        ("traffic_cone_3", "r3", False))
        base = build_runtime_problem(
            SURVEY_5.read_text(), robot="jackal_1", current_region="r3",
            visited_regions=["r1", "r2", "r3", "r4", "r5"],
            inspect_objects=objects)
        status, plan = run_fd(base)
        assert status == "ok"
        assert plan == ["(inspect-object jackal_1 traffic_cone_3 r3)"]

    @needs_fd
    def test_without_the_facts_it_re_inspects_everything(self):
        """The regression this guards, stated as the contrast."""
        objects = found(("traffic_cone_1", "r1"), ("traffic_cone_2", "r3"),
                        ("traffic_cone_3", "r3"))
        base = build_runtime_problem(
            SURVEY_5.read_text(), robot="jackal_1", current_region="r3",
            visited_regions=["r1", "r2", "r3", "r4", "r5"],
            inspect_objects=objects)
        status, plan = run_fd(base)
        assert status == "ok"
        redone = [ln for ln in plan if ln.startswith("(inspect-object")]
        assert len(redone) == 3
        assert len(plan) > 1, "the plan drives back across the map"


class TestWhatThePlannerDoes:
    @needs_fd
    def test_a_plan_inspects_every_object(self):
        text = build_runtime_problem(
            SURVEY.read_text(), robot="jackal_1", current_region="r5",
            visited_regions=["r1", "r2", "r5"],
            inspect_objects=found(("traffic_cone_1", "r1"),
                                  ("traffic_cone_2", "r2")))
        status, plan = run_fd(text)
        assert status == "ok"
        inspections = {ln for ln in plan if ln.startswith("(inspect-object")}
        assert inspections == {"(inspect-object jackal_1 traffic_cone_1 r1)",
                               "(inspect-object jackal_1 traffic_cone_2 r2)"}

    @needs_fd
    def test_an_inspection_is_planned_at_the_objects_own_region(self):
        text = build_runtime_problem(
            SURVEY.read_text(), robot="jackal_1", current_region="r5",
            visited_regions=["r1", "r2", "r5"],
            inspect_objects=found(("traffic_cone_1", "r2")))
        status, plan = run_fd(text)
        assert status == "ok"
        inspect = next(Action(ln) for ln in plan if ln.startswith("(inspect-object"))
        assert inspect.args[-1] == "r2"
        # And the robot is driven there first -- the region is not assumed.
        assert "(move jackal_1 r5 r2)" in plan

    @needs_fd
    def test_the_planner_derives_the_route_not_the_caller(self):
        """The object's region is a FACT, not a target handed over as a string.
        Move it and the plan changes with no other input."""
        def plan_for(region):
            status, plan = run_fd(build_runtime_problem(
                SURVEY.read_text(), robot="jackal_1", current_region="r5",
                visited_regions=["r1", "r2", "r5"],
                inspect_objects=found(("traffic_cone_1", region))))
            assert status == "ok"
            return plan
        assert plan_for("r1") != plan_for("r2")

    @needs_fd
    def test_the_survey_is_finished_when_a_discovery_arrives_mid_tour(self):
        """R2 unvisited: the plan must still cover it, not just inspect."""
        text = build_runtime_problem(
            SURVEY.read_text(), robot="jackal_1", current_region="r1",
            visited_regions=["r5", "r1"],
            inspect_objects=found(("traffic_cone_1", "r1")))
        status, plan = run_fd(text)
        assert status == "ok"
        assert any(ln.endswith("r2)") and ln.startswith("(move") for ln in plan)


class TestTheExecutorKeepsThePlan:
    @needs_fd
    def test_inspections_are_not_filtered_out(self):
        """They used to be: only `move` had an executor, so anything else was
        dropped before driving. An inspection dropped that way is a mission
        that reports success having looked at nothing."""
        text = build_runtime_problem(
            SURVEY.read_text(), robot="jackal_1", current_region="r5",
            visited_regions=["r1", "r2", "r5"],
            inspect_objects=found(("traffic_cone_1", "r1")))
        status, plan = run_fd(text)
        assert status == "ok"
        executable, dropped = filter_executable_actions([Action(ln) for ln in plan])
        assert any(a.name == "inspect-object" for a in executable)
        assert dropped == []

    def test_inspect_object_is_declared_executable(self):
        assert "inspect-object".startswith(EXECUTABLE_ACTION_PREFIXES)

    def test_the_plans_end_region_is_read_off_the_last_action(self):
        """An inspection plan has no target region agreed in advance -- it ends
        wherever the last object it inspects is."""
        plan = [Action("(move jackal_1 r5 r1)"),
                Action("(inspect-object jackal_1 traffic_cone_1 r1)"),
                Action("(move jackal_1 r1 r2)"),
                Action("(inspect-object jackal_1 traffic_cone_2 r2)")]
        assert plan_end_region(plan) == "r2"

    def test_end_region_ignores_actions_with_no_executor(self):
        plan = [Action("(move jackal_1 r5 r1)"),
                Action("(approach jackal_1 cone r1)")]
        assert plan_end_region(plan) == "r1"

    def test_end_region_of_nothing_is_none(self):
        assert plan_end_region([]) is None
