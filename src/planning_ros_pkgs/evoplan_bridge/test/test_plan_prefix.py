"""Taking the executable prefix of a plan that cannot reach its goal.

The find-object mission states a goal the planner provably cannot meet: the
cone's region is unknown, so ``(reached jackal_1 cone)`` has no achiever. The
useful answer is not "no plan" but the part that runs -- the region survey.

The distinction these tests protect is between *inapplicable* and *unmet*. A
plan can stop early because an action's preconditions fail, or it can run to
the end and still leave the goal open. Both produce a prefix worth driving, and
conflating them either throws away a good survey or drives an action that
cannot fire.
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "pipeline" / "evoplan_bridge_host"))

from plan_prefix import executable_prefix  # noqa: E402

DOMAIN = (REPO / "evolve_stl_pddl" / "jackal" / "in" / "factory_jackal_domain.pddl")
TOUR_03 = (REPO / "pipeline" / "missions" / "factory_tour_03.pddl")

SURVEY = ["(move jackal_1 r5 r1)", "(move jackal_1 r1 r2)", "(move jackal_1 r2 r5)"]


@pytest.fixture(scope="module")
def texts():
    return DOMAIN.read_text(), TOUR_03.read_text()


def prefix(texts, plan):
    return executable_prefix(texts[0], texts[1], plan)


class TestUnmetGoalIsNotTruncation:
    def test_a_full_survey_is_kept_whole(self, texts):
        r = prefix(texts, SURVEY)
        assert r.plan == SURVEY
        assert r.prefix_len == r.total == 3
        assert not r.truncated

    def test_but_it_does_not_reach_the_goal(self, texts):
        """require_goal must not collapse the prefix. Asking the validator for
        goal satisfaction reports accepted_prefix_len as len(plan) with ok
        False, which reads identically to 'everything applied' -- the two have
        to be measured separately."""
        assert not prefix(texts, SURVEY).reaches_goal

    def test_empty_plan(self, texts):
        r = prefix(texts, [])
        assert r.plan == [] and r.prefix_len == 0 and not r.reaches_goal


class TestTruncation:
    def test_stops_at_the_unlocated_object(self, texts):
        r = prefix(texts, SURVEY + ["(approach jackal_1 cone r5)"])
        assert r.plan == SURVEY
        assert r.truncated and r.prefix_len == 3 and r.total == 4

    def test_says_why(self, texts):
        """The reason is what tells a reader the survey is the whole story."""
        r = prefix(texts, SURVEY + ["(approach jackal_1 cone r5)"])
        joined = " ".join(r.errors)
        assert "approach" in joined
        assert "location-unknown" in joined or "object-at" in joined

    def test_stops_at_a_disconnected_move(self, texts):
        """R1 and R2 are not adjacent to nothing -- but r1 -> r5 -> r2 is not
        the graph. A move whose (connected ...) is absent must truncate."""
        r = prefix(texts, ["(move jackal_1 r5 r1)", "(move jackal_1 r1 r9)"])
        assert r.prefix_len == 1

    def test_stops_when_the_robot_is_not_where_the_move_starts(self, texts):
        r = prefix(texts, ["(move jackal_1 r1 r2)"])
        assert r.prefix_len == 0, "robot starts at r5, not r1"

    def test_everything_after_the_stop_is_discarded(self, texts):
        """Not a filter: once an action cannot fire, the state the rest was
        planned against never exists."""
        r = prefix(texts, ["(approach jackal_1 cone r5)"] + SURVEY)
        assert r.prefix_len == 0 and r.plan == []


class TestMalformedInput:
    """The plan text comes from an LLM."""

    def test_unknown_action_truncates_rather_than_raises(self, texts):
        r = prefix(texts, SURVEY + ["(teleport jackal_1 r9)"])
        assert r.plan == SURVEY
        assert any("unknown action" in e for e in r.errors)

    def test_wrong_arity_truncates(self, texts):
        r = prefix(texts, ["(move jackal_1 r5)"])
        assert r.prefix_len == 0

    def test_comments_and_blank_lines_are_ignored(self, texts):
        r = prefix(texts, ["; generated", "", "(move jackal_1 r5 r1)", "   "])
        assert r.plan == ["(move jackal_1 r5 r1)"]


class TestOnceTheObjectIsFound:
    """After discovery the same plan is no longer a prefix -- it is a solution."""

    def test_approach_becomes_applicable(self, texts):
        domain, problem = texts
        problem = (problem.replace("(location-unknown cone)", "(object-at cone r5)")
                          .replace("(:goal", "(:goal", 1))
        r = executable_prefix(domain, problem, SURVEY + ["(approach jackal_1 cone r5)"])
        assert not r.truncated
        assert r.reaches_goal, "coverage plus the cone: the whole goal is met"


class TestTheShippedMissionPlansRunPastWhatIsAchievable:
    """Each tour plan is authored as: survey, cone action, then more moves.

    The tail is deliberate. It is what distinguishes taking a PREFIX from
    filtering: `filter_executable_actions` keeps every `move` wherever it
    sits, so on its own it would drive those trailing legs and send the robot
    on a second lap that was planned against a state which never happens.
    """

    PLANS = {
        "factory_tour_03": 3,
        "factory_tour_02": 5,
        "factory_tour_01": 14,
    }

    @pytest.mark.parametrize("tour,survey_len", sorted(PLANS.items()))
    def test_prefix_is_the_survey_and_the_tail_is_dropped(self, tour, survey_len):
        problem = (REPO / "pipeline" / "missions" / f"{tour}.pddl").read_text()
        plan = [ln.strip() for ln
                in (REPO / "factory_mission_plans" / f"{tour}.txt").read_text().splitlines()
                if ln.strip()]
        r = executable_prefix(DOMAIN.read_text(), problem, plan)
        assert r.total > survey_len, "the plan must run past what is achievable"
        assert r.prefix_len == survey_len
        assert r.truncated and not r.reaches_goal
        assert all(a.startswith("(move ") for a in r.plan)
        assert plan[r.prefix_len].startswith("(approach "), "must stop at the cone"

    @pytest.mark.parametrize("tour,survey_len", sorted(PLANS.items()))
    def test_the_survey_still_covers_every_region_in_the_goal(self, tour, survey_len):
        """Truncating must not cost coverage -- the survey is the whole point."""
        import re
        problem = (REPO / "pipeline" / "missions" / f"{tour}.pddl").read_text()
        plan = [ln.strip() for ln
                in (REPO / "factory_mission_plans" / f"{tour}.txt").read_text().splitlines()
                if ln.strip()]
        r = executable_prefix(DOMAIN.read_text(), problem, plan)
        goal = re.sub(r";[^\n]*", "", problem).split("(:goal")[1]
        required = set(re.findall(r"\(visited\s+(\w+)\)", goal))
        assert {a.split()[-1].rstrip(")") for a in r.plan} == required

    @pytest.mark.parametrize("tour", sorted(PLANS))
    def test_filtering_alone_would_drive_the_tail(self, tour):
        """The regression this guards. Not hypothetical: the executor used to
        run filter_executable_actions and nothing else."""
        sys.path.insert(0, str(REPO / "src" / "planning_ros_pkgs" / "evoplan_bridge"))
        from evoplan_bridge.symbolic_replan import filter_executable_actions

        class A:
            def __init__(self, line):
                t = line.strip("()").split()
                self.name, self.args = t[0], tuple(t[1:])

        plan = [ln.strip() for ln
                in (REPO / "factory_mission_plans" / f"{tour}.txt").read_text().splitlines()
                if ln.strip()]
        kept, _ = filter_executable_actions([A(a) for a in plan])
        problem = (REPO / "pipeline" / "missions" / f"{tour}.pddl").read_text()
        prefix = executable_prefix(DOMAIN.read_text(), problem, plan)
        assert len(kept) > prefix.prefix_len, (
            "filtering keeps the post-cone moves; the prefix must not")
