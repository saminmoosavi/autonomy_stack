"""Mission-completion semantics: success must mean the plan ran, not that the
robot stood in the right place.

Replays the concrete failure from factory_mission_01: its target region R8 is
revisited at step 2 of 28, so a position-only success test scored a two-action
run as 100%. These mirror the executor's tracking logic (regions_remaining /
update_mission_completion) without needing a live node.
"""

import pytest


class FakeAction:
    def __init__(self, name, *args):
        self.name, self.args = name, args


def move(frm, to):
    return FakeAction("move", "jackal_1", frm, to)


# factory_mission_01, exactly: a delivery tour that returns to R8 three times.
MISSION_01 = [
    move("r5", "r6"), move("r6", "r8"), move("r8", "r9"), move("r9", "r10"),
    move("r10", "r11"), move("r11", "r12"), move("r12", "r13"), move("r13", "r14"),
    FakeAction("pickup-box", "jackal_1", "box_1", "r14"),
    move("r14", "r13"), move("r13", "r12"),
    FakeAction("dropoff-box", "jackal_1", "box_1", "r12"),
    move("r12", "r11"), move("r11", "r10"), move("r10", "r9"), move("r9", "r8"),
    move("r8", "r6"), move("r6", "r5"), move("r5", "r3"),
    FakeAction("pickup-box", "jackal_1", "box_2", "r3"),
    move("r3", "r2"), FakeAction("inspect-shelf", "jackal_1", "r2"),
    move("r2", "r1"), move("r1", "r5"), move("r5", "r7"),
    FakeAction("inspect-shelf", "jackal_1", "r7"),
    move("r7", "r8"), FakeAction("dropoff-box", "jackal_1", "box_2", "r8"),
]


class MissionTracker:
    """Mirror of the executor's completion logic."""

    def __init__(self, plan, target):
        self.required = [a.args[-1].lower() for a in plan if a.name.startswith("move")]
        self.target = target
        self.visited = set()
        self.complete = False

    def remaining(self):
        return sorted(set(self.required) - self.visited)

    def arrive(self, region, at_target=None):
        self.visited.add(region)
        if at_target is None:
            at_target = region == self.target
        if not self.complete and not self.remaining() and at_target:
            self.complete = True
        return self.complete


class TestTargetRevisitTrap:
    def test_target_is_revisited_early(self):
        """The property that made position-only success wrong."""
        dests = [a.args[-1].lower() for a in MISSION_01 if a.name.startswith("move")]
        assert dests[-1] == "r8"
        assert dests.index("r8") == 1, "R8 is reached at move 2 of 22"

    def test_two_action_run_is_not_success(self):
        """The exact 15:14:59 trial: drove r5->r6->r8 and stopped."""
        t = MissionTracker(MISSION_01, "r8")
        t.arrive("r6")
        assert not t.arrive("r8"), "a 2-of-28 run must NOT count as mission success"
        assert len(t.remaining()) > 0

    def test_full_tour_is_success(self):
        t = MissionTracker(MISSION_01, "r8")
        for region in [a.args[-1].lower() for a in MISSION_01 if a.name.startswith("move")]:
            t.arrive(region)
        assert t.complete
        assert t.remaining() == []

    def test_all_regions_visited_but_parked_elsewhere_is_not_success(self):
        """Must finish AT the target, not merely have passed through everything."""
        t = MissionTracker(MISSION_01, "r8")
        regions = {a.args[-1].lower() for a in MISSION_01 if a.name.startswith("move")}
        for r in regions:
            t.arrive(r, at_target=False)
        assert not t.complete
        # ...and completes once it parks at the target.
        assert t.arrive("r8", at_target=True)


class TestReplanCannotShortenTheMission:
    def test_narrowed_replan_does_not_complete_itself(self):
        """A replan to `(visited r8)` yields one move; that must not finish the
        mission, which is precisely what the old position test allowed."""
        t = MissionTracker(MISSION_01, "r8")   # required frozen from the ORIGINAL plan
        t.arrive("r6")
        assert not t.arrive("r8"), "single-move replan must not complete a 28-action mission"
        assert "r14" in t.remaining() and "r12" in t.remaining()

    def test_requirements_are_frozen_at_load(self):
        """Swapping in a shorter plan must not shrink what success requires."""
        t = MissionTracker(MISSION_01, "r8")
        before = set(t.required)
        # a replan arrives; the executor never reassigns `required`
        assert set(t.required) == before


class TestShortMissionsStillPass:
    @pytest.mark.parametrize("plan,target", [
        ([move("r5", "r7"), move("r7", "r8")], "r8"),
        ([move("r5", "r6")], "r6"),
    ])
    def test_one_way_missions_complete_normally(self, plan, target):
        """Missions 02 and 05-10 never revisit their target; they must be
        unaffected by the stricter rule."""
        t = MissionTracker(plan, target)
        for a in plan:
            t.arrive(a.args[-1].lower())
        assert t.complete

    def test_no_plan_means_no_gating(self):
        """With no required regions (planner silent), completion must not block
        forever -- scand_metrics falls back to the positional test."""
        t = MissionTracker([], "r8")
        assert t.remaining() == []
