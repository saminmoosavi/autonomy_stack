"""Unit tests for the Tier-2 state machine and its pure helpers.

No ROS, no sim, no LLM.
"""

import pytest

from evoplan_bridge.symbolic_replan import (
    APPLYING,
    HOLDING,
    IDLE,
    SymbolicReplanState,
    build_reason_text,
    derive_blocked_regions,
    filter_executable_actions,
    plan_reaches_target,
)


class FakeAction:
    """Stand-in for pddl_stl.pipeline.GroundAction (same duck type)."""

    def __init__(self, name, *args):
        self.name = name
        self.args = tuple(args)

    def text(self):
        return "(" + " ".join((self.name,) + self.args) + ")"


def move(frm, to):
    return FakeAction("move", "jackal_1", frm, to)


class TestGuards:
    def test_idle_state_allows_escalation(self):
        st = SymbolicReplanState()
        allowed, why = st.can_escalate(100.0, max_replans=2, cooldown_s=30.0,
                                       deliberation_budget_s=120.0)
        assert allowed, why

    @pytest.mark.parametrize("busy", [HOLDING, APPLYING])
    def test_only_one_replan_in_flight(self, busy):
        st = SymbolicReplanState(state=busy)
        allowed, why = st.can_escalate(100.0, 2, 30.0, 120.0)
        assert not allowed
        assert "already" in why

    def test_replan_budget_bounds_the_loop(self):
        """Without this the tiers can escalate each other indefinitely."""
        st = SymbolicReplanState(count=2)
        allowed, why = st.can_escalate(100.0, max_replans=2, cooldown_s=0.0,
                                       deliberation_budget_s=120.0)
        assert not allowed
        assert "budget exhausted" in why

    def test_cooldown_blocks_rapid_reescalation(self):
        st = SymbolicReplanState(last_escalation_sim_s=100.0)
        allowed, why = st.can_escalate(110.0, 5, cooldown_s=30.0,
                                       deliberation_budget_s=120.0)
        assert not allowed
        assert "cooldown" in why
        # ...and clears once the cooldown elapses.
        allowed, _ = st.can_escalate(131.0, 5, 30.0, 120.0)
        assert allowed

    def test_deliberation_budget_is_wall_clock_total(self):
        st = SymbolicReplanState(deliberation_wall_s=125.0)
        allowed, why = st.can_escalate(1000.0, 5, 0.0, deliberation_budget_s=120.0)
        assert not allowed
        assert "deliberation budget" in why


class TestBlockedRegionDerivation:
    def test_shield_blocks_the_obstacle_region(self):
        assert derive_blocked_regions(
            "stl_shield", current_region="r9", target_region="r12",
            obstacle_region="r10",
        ) == ["r10"]

    def test_abort_blocks_the_failed_region(self):
        assert derive_blocked_regions(
            "nav2_goal_aborted", current_region="r9", target_region="r12",
            failed_region="r10",
        ) == ["r10"]

    def test_never_blocks_the_current_region(self):
        """Blocking where the robot stands makes the problem unsolvable."""
        assert derive_blocked_regions(
            "stl_shield", current_region="r9", target_region="r12",
            obstacle_region="r9",
        ) == []

    def test_never_blocks_the_goal_region(self):
        """Blocking the goal makes the mission unreachable by construction."""
        assert derive_blocked_regions(
            "stl_shield", current_region="r9", target_region="r12",
            obstacle_region="r12",
        ) == []

    def test_case_insensitive(self):
        assert derive_blocked_regions(
            "stl_shield", current_region="R9", target_region="R12",
            obstacle_region="R10",
        ) == ["r10"]
        assert derive_blocked_regions(
            "stl_shield", current_region="R9", target_region="R12",
            obstacle_region="R9",
        ) == []

    def test_missing_region_blocks_nothing(self):
        assert derive_blocked_regions(
            "stl_shield", current_region="r9", target_region="r12",
        ) == []


class TestReasonText:
    def test_shield_text_names_the_conjunct_and_blocked_region(self):
        text = build_reason_text("stl_shield", {
            "from_region": "r9", "to_region": "r10",
            "violated_conjunct": "ped_ttc", "robustness": -0.31,
            "blocked_regions": ["r10"],
        })
        assert "ped_ttc" in text
        assert "r9 -> r10" in text
        assert "-0.31" in text
        assert "r10" in text

    def test_budget_text_explains_local_avoidance_failed(self):
        text = build_reason_text("reactive_budget_exhausted", {
            "from_region": "r9", "to_region": "r10",
            "reactive_attempts": 3, "blocked_regions": ["r10"],
        })
        assert "3 reactive" in text
        assert "local obstacle avoidance is not enough" in text

    def test_says_so_when_nothing_could_be_blocked(self):
        """The LLM must be told to re-route rather than wait for a blocked fact."""
        text = build_reason_text("stl_shield", {
            "from_region": "r9", "to_region": "r10", "blocked_regions": [],
        })
        assert "No region could be marked blocked" in text
        assert "alternative" in text

    def test_unknown_trigger_still_produces_text(self):
        assert build_reason_text("something_new", {"blocked_regions": []})


class TestActionFiltering:
    def test_splits_move_from_unexecutable(self):
        actions = [move("r5", "r6"),
                   FakeAction("pickup-box", "jackal_1", "box_1", "r6"),
                   move("r6", "r7")]
        executable, dropped = filter_executable_actions(actions)
        assert [a.text() for a in executable] == [
            "(move jackal_1 r5 r6)", "(move jackal_1 r6 r7)"]
        assert [a.text() for a in dropped] == ["(pickup-box jackal_1 box_1 r6)"]

    def test_move_through_narrow_area_counts_as_executable(self):
        """plan_to_nav2_goals lowers anything starting with 'move'."""
        actions = [FakeAction("move-through-narrow-area", "jackal_1", "r5", "r6")]
        executable, dropped = filter_executable_actions(actions)
        assert len(executable) == 1 and not dropped

    def test_target_reached_checks_the_last_move(self):
        plan = [move("r5", "r6"), move("r6", "r12")]
        assert plan_reaches_target(plan, "r12")
        assert plan_reaches_target(plan, "R12")  # case-insensitive
        assert not plan_reaches_target(plan, "r10")

    def test_empty_move_chain_never_reaches_target(self):
        """A plan of only non-move actions must be rejected, not accepted."""
        assert not plan_reaches_target([], "r12")
        assert not plan_reaches_target(
            [FakeAction("inspect-shelf", "jackal_1", "r7")], "r12")


class TestWaypointSpliceArithmetic:
    """The #1 ranked risk: index bookkeeping across a plan swap.

    These mirror what ``apply_symbolic_replan`` and ``resend_waypoint_suffix``
    do to ``_waypoints_offset`` / ``_last_feedback_waypoint``, using a minimal
    fake so the arithmetic is checked without standing up a ROS node.
    """

    class FakeNode:
        def __init__(self, waypoints, actions):
            self.active_waypoints = list(waypoints)
            self.active_plan_actions = list(actions)
            self._waypoints_offset = 0
            self._last_feedback_waypoint = None
            self.sent = []

        # mirrors resend_waypoint_suffix
        def resend_suffix(self, start_idx):
            start = max(0, int(start_idx or 0))
            remaining = self.active_waypoints[start:] or self.active_waypoints
            self._waypoints_offset += start
            self.active_waypoints = remaining
            self._last_feedback_waypoint = None
            self.sent.append(list(remaining))
            return remaining

        # mirrors the reset block in apply_symbolic_replan
        def apply_new_plan(self, waypoints, actions):
            self.active_plan_actions = list(actions)
            self.active_waypoints = list(waypoints)
            self._waypoints_offset = 0
            self._last_feedback_waypoint = None
            self.sent.append(list(waypoints))

        # mirrors feedback_callback's label lookup
        def step_label(self, current_waypoint):
            completed = current_waypoint - 1
            moves = [a for a in self.active_plan_actions if a.name.startswith("move")]
            idx = completed + self._waypoints_offset
            if idx < len(moves):
                return f"({idx + 1}/{len(moves)}) {moves[idx].text()}"
            return None

    def test_reactive_resend_accumulates_offset(self):
        """Tier 1 keeps the same plan, so the offset must accumulate."""
        node = self.FakeNode([(0, 0), (1, 0), (2, 0), (3, 0)],
                             [move("r1", "r2"), move("r2", "r3"),
                              move("r3", "r4"), move("r4", "r5")])
        node._last_feedback_waypoint = 2
        node.resend_suffix(2)
        assert node._waypoints_offset == 2
        assert len(node.active_waypoints) == 2
        # Completing the next waypoint must still label step 3 of 4.
        assert node.step_label(1) == "(3/4) (move jackal_1 r3 r4)"

    def test_plan_swap_resets_offset(self):
        """A new plan is not a suffix: a stale offset mislabels every step."""
        node = self.FakeNode([(0, 0), (1, 0), (2, 0), (3, 0)],
                             [move("r1", "r2"), move("r2", "r3"),
                              move("r3", "r4"), move("r4", "r5")])
        node._last_feedback_waypoint = 2
        node.resend_suffix(2)
        assert node._waypoints_offset == 2

        new_actions = [move("r3", "r8"), move("r8", "r5")]
        node.apply_new_plan([(9, 0), (10, 0)], new_actions)

        assert node._waypoints_offset == 0, "offset must reset on a plan swap"
        assert node._last_feedback_waypoint is None
        # The proof from the plan's Stage 3: step numbering restarts at 1
        # against the NEW plan's length.
        assert node.step_label(1) == "(1/2) (move jackal_1 r3 r8)"

    def test_stale_offset_would_mislabel(self):
        """Documents the bug the reset prevents, so the test has teeth."""
        node = self.FakeNode([(0, 0)], [move("r1", "r2"), move("r2", "r3")])
        node._waypoints_offset = 2  # deliberately not reset
        node.active_plan_actions = [move("r3", "r8"), move("r8", "r5")]
        # Index 2 is past the end of the 2-action plan: the guard returns None,
        # so the step line silently vanishes instead of crashing.
        assert node.step_label(1) is None

    def test_abandon_resumes_old_route_not_a_fresh_one(self):
        node = self.FakeNode([(0, 0), (1, 0), (2, 0)],
                             [move("r1", "r2"), move("r2", "r3"), move("r3", "r4")])
        node._last_feedback_waypoint = 1
        node.resend_suffix(1)
        assert node.sent[-1] == [(1, 0), (2, 0)]
        assert node._waypoints_offset == 1

    def test_resend_from_zero_is_a_noop_on_offset(self):
        node = self.FakeNode([(0, 0), (1, 0)], [move("r1", "r2"), move("r2", "r3")])
        node.resend_suffix(0)
        assert node._waypoints_offset == 0
        assert len(node.active_waypoints) == 2

    def test_resend_past_the_end_falls_back_to_whole_route(self):
        """Never send an empty goal list -- that strands the robot."""
        node = self.FakeNode([(0, 0), (1, 0)], [move("r1", "r2"), move("r2", "r3")])
        node.resend_suffix(99)
        assert node.active_waypoints, "must not send an empty waypoint list"
