"""Where the robot stands and which way it looks during an inspection.

The specified behaviour is "go to the region closest to the object, and pause
facing it". Both halves are load-bearing and both have a failure that only
shows up on a real robot:

* standing anywhere other than a region centroid risks a goal Nav2 cannot
  reach -- inside the obstacle, or in an aisle with no room to turn;
* a heading computed by subtracting angles spins the robot the long way round
  whenever the difference crosses +/-pi, which looks exactly like a controller
  fault and is not one.

The route-cutting rule is here too: it is what makes a five-second pause
possible at all, given Nav2's only pause is a global plugin parameter that
would stop the robot at every waypoint for the same duration.
"""

import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "src" / "planning_ros_pkgs" / "evoplan_bridge"))

from evoplan_bridge.inspection import (  # noqa: E402
    inspection_pose,
    segment_end,
    yaw_error,
)


class TestWhereToStand:
    def test_the_default_is_the_region_centroid(self):
        """standoff 0: drive to the region and turn. The centroid is a waypoint
        the robot is known to be able to reach; a computed pose is not."""
        stand, _ = inspection_pose((1.0, 2.0), (5.0, 2.0))
        assert stand == (1.0, 2.0)

    def test_a_standoff_stops_short_of_the_object(self):
        stand, _ = inspection_pose((0.0, 0.0), (10.0, 0.0), standoff_m=2.0)
        assert stand == pytest.approx((8.0, 0.0))
        assert math.dist(stand, (10.0, 0.0)) == pytest.approx(2.0)

    def test_a_standoff_larger_than_the_gap_stays_at_the_centroid(self):
        """Never past the object, and never onto it."""
        stand, _ = inspection_pose((0.0, 0.0), (1.0, 0.0), standoff_m=5.0)
        assert stand == (0.0, 0.0)

    def test_the_standoff_follows_the_centroid_to_object_line(self):
        stand, _ = inspection_pose((0.0, 0.0), (3.0, 4.0), standoff_m=2.5)
        assert math.dist(stand, (3.0, 4.0)) == pytest.approx(2.5)
        # On the line: cross product of (stand) and (object) is zero.
        assert stand[0] * 4.0 - stand[1] * 3.0 == pytest.approx(0.0)


class TestWhichWayToLook:
    @pytest.mark.parametrize("object_xy,expected_deg", [
        ((5.0, 0.0), 0.0),
        ((0.0, 5.0), 90.0),
        ((-5.0, 0.0), 180.0),
        ((0.0, -5.0), -90.0),
    ])
    def test_the_yaw_points_at_the_object(self, object_xy, expected_deg):
        _, yaw = inspection_pose((0.0, 0.0), object_xy)
        assert math.degrees(yaw) == pytest.approx(expected_deg)

    def test_an_unlocated_object_has_no_heading(self):
        """A plan can name an object this executor never discovered. Drive
        there and dwell anyway -- a degraded inspection beats none."""
        stand, yaw = inspection_pose((1.0, 2.0), None)
        assert stand == (1.0, 2.0)
        assert yaw is None

    def test_standing_on_the_object_does_not_divide_by_zero(self):
        stand, yaw = inspection_pose((1.0, 1.0), (1.0, 1.0), standoff_m=2.0)
        assert stand == (1.0, 1.0)
        assert yaw == 0.0


class TestYawError:
    def test_it_takes_the_short_way_round(self):
        """The +/-pi discontinuity: 10 degrees of correction, not 350."""
        error = yaw_error(math.radians(-175), math.radians(175))
        assert math.degrees(error) == pytest.approx(10.0)

    def test_the_sign_says_which_way_to_turn(self):
        assert yaw_error(math.radians(30), math.radians(0)) > 0
        assert yaw_error(math.radians(-30), math.radians(0)) < 0

    def test_no_error_when_already_aligned(self):
        assert yaw_error(1.234, 1.234) == pytest.approx(0.0)


class TestRouteCutting:
    def move(self):
        return {"kind": "move"}

    def inspect(self, name="traffic_cone_1"):
        return {"kind": "inspect", "object": name}

    def test_a_route_with_no_inspections_is_driven_whole(self):
        """The pre-existing behaviour, unchanged: one goal, no pauses."""
        assert segment_end([self.move(), self.move(), self.move()]) is None

    def test_the_segment_ends_at_the_first_inspection(self):
        tasks = [self.move(), self.move(), self.inspect(), self.move(),
                 self.inspect("traffic_cone_2")]
        assert segment_end(tasks) == 2

    def test_an_inspection_first_is_a_one_waypoint_segment(self):
        assert segment_end([self.inspect(), self.move()]) == 0

    def test_an_empty_route_has_no_segment(self):
        assert segment_end([]) is None
        assert segment_end(None) is None

    def test_padding_entries_do_not_crash_the_scan(self):
        assert segment_end([None, self.inspect()]) == 1
