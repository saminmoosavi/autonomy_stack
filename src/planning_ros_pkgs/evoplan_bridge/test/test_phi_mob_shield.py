"""Unit tests for the Phi_mob shield. No ROS, no sim, no LLM."""

import math

import pytest

from evoplan_bridge.envelope import ENVELOPE, conjunct_margin
from evoplan_bridge.phi_mob_shield import PhiMobShield

DT = 0.1  # 10 Hz, the shield's wiring rate


def drive_straight(shield, speed, steps, t0=0.0, dt=DT):
    """Feed odometry for constant-speed straight-line motion along +x."""
    t = t0
    x = 0.0
    # Prime the finite differences: the first two samples establish pose and
    # speed history, so margins only become meaningful from the third.
    for _ in range(steps):
        shield.update_odom(x, 0.0, 0.0, t)
        x += speed * dt
        t += dt
    return t


class TestEnvelope:
    def test_margin_sign_and_scale(self):
        # 50% under a <= bound reads as +0.5; 50% over reads as -0.5.
        assert conjunct_margin(1.0, "<=", 2.0) == pytest.approx(0.5)
        assert conjunct_margin(3.0, "<=", 2.0) == pytest.approx(-0.5)
        # >= bounds mirror.
        assert conjunct_margin(3.0, ">=", 2.0) == pytest.approx(0.5)
        assert conjunct_margin(1.0, ">=", 2.0) == pytest.approx(-0.5)

    def test_undefined_signal_never_vetoes(self):
        # Infinite TTC means nothing ahead: trivially compliant, not NaN.
        assert conjunct_margin(math.inf, ">=", 0.452) == math.inf
        # A missing <= signal must not manufacture a violation either.
        assert conjunct_margin(math.nan, "<=", 2.603) == math.inf

    def test_rejects_unknown_operator(self):
        with pytest.raises(ValueError):
            conjunct_margin(1.0, "<", 2.0)


def close_on_person(shield, steps=6, gap=1.4, t0=0.0, dt=DT):
    """Drive head-on at a pedestrian until ped_ttc goes negative.

    The veto-capable analogue of drive_straight: robot and person converge at
    2.0 m/s combined, ending ~0.4 m apart for TTC ~0.2 s against the 0.422 s
    bound, while the robot's own 1.0 m/s stays well inside the speed bound.
    """
    t, rx, hx = t0, 0.0, gap
    for _ in range(steps):
        shield.update_odom(rx, 0.0, 0.0, t)
        shield.update_tracks([("person_1", hx, 0.0)], t)
        rx += 1.0 * dt
        hx -= 1.0 * dt
        t += dt
    return t


class TestSpeedIsMonitoredButCannotVeto:
    """Speed is measured and reported, but must not trigger a replan.

    A symbolic reroute cannot slow the robot down -- sending it to different
    regions does nothing about exceeding 2.603 m/s. In a pedestrian-free world
    both symbolic replans fired on speed, exhausted the budget, and truncated a
    14-region tour at 5 regions.
    """

    def test_speeding_is_reported(self):
        shield = PhiMobShield(horizon_s=3.0)
        t = drive_straight(shield, speed=3.0, steps=5)
        result = shield.evaluate(t)
        assert result.margins["speed"] < 0.0, "speed must still be measured"

    def test_speeding_does_not_veto(self):
        shield = PhiMobShield(horizon_s=3.0)
        t = drive_straight(shield, speed=3.0, steps=5)
        result = shield.evaluate(t)
        assert result.satisfied, "speed must not be able to trigger a replan"
        assert result.worst_conjunct != "speed"


class TestVetoWindowMechanics:
    """Window/min/reset behaviour, exercised through a veto-capable conjunct."""

    def test_closing_on_person_vetoes(self):
        shield = PhiMobShield(horizon_s=3.0)
        t = close_on_person(shield)
        result = shield.evaluate(t)
        assert not result.satisfied
        assert result.worst_conjunct in ("ttc", "ped_ttc")

    def test_window_expiry_restores_robustness(self):
        """A violation must age out of the window, or one bad tick vetoes
        forever and the replan budget is spent on stale evidence."""
        shield = PhiMobShield(horizon_s=1.0)
        t = close_on_person(shield)
        assert not shield.evaluate(t).satisfied
        for _ in range(25):                      # person gone, longer than horizon
            shield.update_odom(0.0, 0.0, 0.0, t)
            shield.update_tracks([], t)
            t += DT
        result = shield.evaluate(t)
        assert result.satisfied, f"stale violation stuck in window: {result.margins}"

    def test_min_over_window_not_just_latest(self):
        """Robustness is a window minimum: the person walking away must not
        clear the veto while the breach is still inside the horizon."""
        shield = PhiMobShield(horizon_s=10.0)
        t = close_on_person(shield)
        for _ in range(5):
            shield.update_odom(0.0, 0.0, 0.0, t)
            shield.update_tracks([], t)
            t += DT
        assert not shield.evaluate(t).satisfied

    def test_reset_clears_history(self):
        """Used after a plan swap, so margins accrued under the old plan cannot
        immediately re-trigger under the new one."""
        shield = PhiMobShield(horizon_s=10.0)
        t = close_on_person(shield)
        assert not shield.evaluate(t).satisfied
        shield.reset()
        assert shield.evaluate(t).robustness == math.inf


class TestSpeedConjunct:
    def test_compliant_speed_is_positive(self):
        shield = PhiMobShield(horizon_s=3.0)
        drive_straight(shield, speed=1.0, steps=5)
        result = shield.evaluate(5 * DT)
        assert result.satisfied
        assert result.robustness > 0.0




class TestPedestrianConjuncts:
    def test_head_on_approach_violates_ttc(self):
        """Closing fast on a person ahead must fire ped_ttc, not speed.

        Geometry: robot and person converge at 2.0 m/s combined. The gap is
        chosen so the final separation (0.4 m) gives TTC = 0.2 s, comfortably
        under the mined 0.422 s bound, while the robot's own 1.0 m/s stays well
        under the 2.603 speed bound -- so only the TTC conjuncts can fire.
        """
        shield = PhiMobShield(horizon_s=3.0)
        t = 0.0
        rx = 0.0
        hx = 1.4
        for _ in range(6):
            shield.update_odom(rx, 0.0, 0.0, t)       # robot +x at 1.0 m/s
            shield.update_tracks([("person_1", hx, 0.0)], t)
            rx += 1.0 * DT
            hx -= 1.0 * DT                             # person walks toward robot
            t += DT
        result = shield.evaluate(t)
        assert not result.satisfied
        assert result.worst_conjunct in ("ttc", "ped_ttc")
        assert result.signals["ped_approach_rate"] > 0.0
        assert result.margins["speed"] > 0.0, "speed must not be the trigger here"

    def test_safe_following_distance_does_not_veto(self):
        """The mirror of the above: same closing speed, more room, no veto.

        Guards the threshold itself -- if TTC were computed wrong by a factor,
        this case and the one above would not straddle the bound.
        """
        shield = PhiMobShield(horizon_s=3.0)
        t = 0.0
        rx = 0.0
        hx = 4.0
        for _ in range(6):
            shield.update_odom(rx, 0.0, 0.0, t)
            shield.update_tracks([("person_1", hx, 0.0)], t)
            rx += 1.0 * DT
            hx -= 1.0 * DT
            t += DT
        assert shield.evaluate(t).satisfied

    def test_person_behind_is_ignored(self):
        """Only the frontal cone counts, or a crowd vetoes permanently."""
        shield = PhiMobShield(horizon_s=3.0)
        t = 0.0
        rx = 0.0
        for _ in range(6):
            shield.update_odom(rx, 0.0, 0.0, t)        # heading +x
            shield.update_tracks([("person_1", rx - 0.5, 0.0)], t)  # directly behind
            rx += 1.0 * DT
            t += DT
        result = shield.evaluate(t)
        assert result.satisfied
        assert not math.isfinite(result.signals["ttc"])

    def test_empty_track_list_is_compliant(self):
        shield = PhiMobShield(horizon_s=3.0)
        t = 0.0
        for _ in range(5):
            shield.update_odom(0.0, 0.0, 0.0, t)
            shield.update_tracks([], t)
            t += DT
        assert shield.evaluate(t).satisfied

    def test_stale_tracks_are_evicted(self):
        shield = PhiMobShield(horizon_s=3.0)
        shield.update_odom(0.0, 0.0, 0.0, 0.0)
        shield.update_tracks([("a", 1.0, 0.0), ("b", 2.0, 0.0)], 0.0)
        shield.update_tracks([("a", 1.0, 0.0)], DT)
        assert set(shield._prev_tracks) == {"a"}


class TestNumericalGuards:
    def test_zero_dt_does_not_explode(self):
        """Duplicate timestamps must not divide by ~0 and fake a huge accel."""
        shield = PhiMobShield(horizon_s=3.0)
        shield.update_odom(0.0, 0.0, 0.0, 1.0)
        shield.update_odom(0.5, 0.0, 0.0, 1.0)   # same stamp, big jump
        result = shield.evaluate(1.0)
        assert result.satisfied

    def test_large_dt_gap_is_skipped(self):
        """A pause in odometry must not read as a teleport at infinite speed."""
        shield = PhiMobShield(horizon_s=10.0)
        shield.update_odom(0.0, 0.0, 0.0, 0.0)
        shield.update_odom(50.0, 0.0, 0.0, 5.0)  # 5 s gap, way over MAX_DT
        assert shield.evaluate(5.0).satisfied

    def test_yaw_wrap_is_not_a_spin(self):
        """Crossing +/-pi must use the shortest angular difference."""
        shield = PhiMobShield(horizon_s=3.0)
        shield.update_odom(0.0, 0.0, math.pi - 0.01, 0.0)
        shield.update_odom(0.0, 0.0, -math.pi + 0.01, DT)
        result = shield.evaluate(DT)
        # True turn is 0.02 rad over 0.1 s = 0.2 rad/s, nowhere near the
        # 3.293 bound. Naive subtraction would give ~62 rad/s and veto.
        assert result.signals["yaw_rate"] < 1.0
        assert result.satisfied

    def test_no_data_yields_infinite_robustness(self):
        """Before any samples the shield must abstain, not veto."""
        shield = PhiMobShield(horizon_s=3.0)
        result = shield.evaluate(0.0)
        assert result.robustness == math.inf
        assert result.satisfied
        assert result.worst_conjunct is None

