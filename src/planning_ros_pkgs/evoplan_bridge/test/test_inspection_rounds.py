"""A failed inspection round must be retried, and must not be retried forever.

Replays the concrete failure from
`factory_survey_02_p0_evoplan_only_i2_20260811_153359`: the survey found both
cones, the one inspection replan timed out, and the run ended having inspected
neither -- with its replan budget still unspent. The cause was a latch,
`_inspection_replan_pending`, set before escalating and cleared only on success.
After the failure nothing ever asked again.

The fix has three moving parts and each can fail on its own, so each is pinned
here:

* the latch means "a round is in flight", so it clears on failure too;
* a cooldown stops the 1 Hz exhaustion tick turning the retry into a flood;
* a round cap stops an unreachable object keeping the mission alive forever.

Mirrors the executor's rule rather than driving a live node, as
test_mission_completion.py does for mission completion -- the arithmetic is the
part that was wrong, and it is decidable without a simulator.
"""

import pytest


class Rounds:
    """Mirror of begin_inspection_round's guards and abandon_replan's release."""

    def __init__(self, pending=2, retry_s=15.0, max_rounds=6):
        self.pending = pending
        self.retry_s = retry_s
        self.max_rounds = max_rounds
        self.in_flight = False
        self.last_round_s = None
        self.attempts = 0
        self.escalations = 0

    def begin(self, now):
        """True when a round was escalated."""
        if not self.pending:
            return False
        if self.in_flight:
            return False
        if (self.last_round_s is not None
                and now - self.last_round_s < self.retry_s):
            return False
        if self.attempts >= self.max_rounds:
            return False
        self.in_flight = True
        self.last_round_s = now
        self.attempts += 1
        self.escalations += 1
        return True

    def failed(self):
        """abandon_replan: the round is no longer in flight."""
        self.in_flight = False

    def applied(self, inspected=0):
        self.in_flight = False
        self.pending = max(0, self.pending - inspected)

    def retry_possible(self):
        return bool(self.pending) and self.attempts < self.max_rounds


class TestTheRegression:
    def test_a_failed_round_is_retried(self):
        """The run that motivated this inspected nothing after one timeout."""
        r = Rounds()
        assert r.begin(now=0.0)
        r.failed()
        assert r.begin(now=20.0), "a failed round was never retried"

    def test_the_retry_can_still_finish_the_mission(self):
        r = Rounds(pending=2)
        r.begin(now=0.0)
        r.failed()
        r.begin(now=20.0)
        r.applied(inspected=2)
        assert r.pending == 0
        assert r.escalations == 2

    def test_a_failure_leaves_the_run_alive(self):
        """plan_exhaustion_tick asks this before declaring the plan finished;
        answering False during a cooldown is what ended the mission early."""
        r = Rounds()
        r.begin(now=0.0)
        r.failed()
        assert r.retry_possible()


class TestTheCooldown:
    def test_a_1hz_tick_does_not_flood_the_service(self):
        r = Rounds(retry_s=15.0)
        r.begin(now=0.0)
        r.failed()
        for tick in range(1, 15):        # one call per second, still cooling
            assert not r.begin(now=float(tick))
        assert r.escalations == 1

    def test_the_next_attempt_lands_once_the_cooldown_elapses(self):
        r = Rounds(retry_s=15.0)
        r.begin(now=0.0)
        r.failed()
        assert not r.begin(now=14.9)
        assert r.begin(now=15.0)

    def test_a_round_in_flight_is_never_stacked(self):
        """The latch's original and still-valid purpose."""
        r = Rounds(retry_s=0.0)
        assert r.begin(now=0.0)
        for tick in range(1, 40):
            assert not r.begin(now=float(tick))
        assert r.escalations == 1


class TestTheCap:
    def test_retries_are_bounded(self):
        r = Rounds(retry_s=15.0, max_rounds=6)
        now = 0.0
        for _ in range(20):
            if r.begin(now=now):
                r.failed()
            now += 15.0
        assert r.escalations == 6

    def test_the_run_ends_once_the_cap_is_reached(self):
        r = Rounds(retry_s=0.0, max_rounds=3)
        for _ in range(3):
            assert r.begin(now=0.0)
            r.failed()
        assert not r.retry_possible(), "the mission would never terminate"

    def test_successful_rounds_count_against_the_cap_too(self):
        """Each is a real deliberation with a real cost; a mission that keeps
        discovering objects must not be able to run indefinitely."""
        r = Rounds(pending=9, retry_s=0.0, max_rounds=3)
        for _ in range(3):
            assert r.begin(now=0.0)
            r.applied(inspected=1)
        assert not r.begin(now=0.0)

    def test_nothing_pending_means_no_round_at_all(self):
        r = Rounds(pending=0)
        assert not r.begin(now=0.0)
        assert not r.retry_possible()


@pytest.mark.parametrize("failures", [0, 1, 2, 5])
def test_the_mission_survives_n_consecutive_failures(failures):
    """Up to the cap, a run of bad replans costs time, not the mission."""
    r = Rounds(pending=1, retry_s=15.0, max_rounds=6)
    now = 0.0
    for _ in range(failures):
        assert r.begin(now=now)
        r.failed()
        now += 15.0
    assert r.begin(now=now), "the mission gave up while retries remained"
    r.applied(inspected=1)
    assert r.pending == 0
