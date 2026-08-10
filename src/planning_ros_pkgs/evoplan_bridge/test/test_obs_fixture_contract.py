"""Pin the test fixtures to the shape observation_logger actually writes.

This is the test that was missing. ``test_find_object.py`` and
``test_observation_memory.py`` built ``position_map`` as ``[x, y]``; the logger
writes ``{"x": .., "y": .., "z": ..}``. Every test passed, and the disagreement
surfaced instead as a live crash -- ``KeyError: 0`` in ``locate_object``,
killing the executor at the moment the tour finished and the object approach
began, with the cone already correctly detected 64 times.

Reading the logger's source rather than trusting a copied comment is the whole
point: if someone changes the wire format, this fails immediately and loudly
rather than one mission later. Same approach as test_status_contract.py, which
pins the published-status dict against its consumer.
"""

import ast
from pathlib import Path

import pytest

from evoplan_bridge.observation_memory import position_xy

import obs_fixtures

REPO = Path(__file__).resolve().parents[4]
LOGGER = (REPO / "src/planning_ros_pkgs/evo_skill_ros/evo_skill_ros/nodes"
                 "/observation_logger.py")


def logger_position_map_keys():
    """Keys of the ``position_map`` dict literal in observation_logger.py.

    Parsed from source because the logger is a ROS node: importing it would
    pull in rclpy and a live graph, which no unit test should need.
    """
    tree = ast.parse(LOGGER.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if (isinstance(key, ast.Constant) and key.value == "position_map"
                    and isinstance(value, ast.Dict)):
                return tuple(
                    k.value for k in value.keys
                    if isinstance(k, ast.Constant)
                )
    return None


class TestFixtureMatchesLogger:
    def test_logger_writes_a_mapping(self):
        """If this fails the logger changed shape -- update obs_fixtures too."""
        assert LOGGER.is_file(), f"logger not found at {LOGGER}"
        keys = logger_position_map_keys()
        assert keys is not None, (
            "no `\"position_map\": {...}` dict literal in observation_logger.py; "
            "the wire format changed and obs_fixtures.py must follow it"
        )
        assert keys == obs_fixtures.POSITION_KEYS, (
            f"logger writes position_map keys {keys}, fixtures build "
            f"{obs_fixtures.POSITION_KEYS}"
        )

    def test_fixture_emits_that_exact_shape(self):
        pos = obs_fixtures.detection("cone", (1.0, 2.0))["position_map"]
        assert isinstance(pos, dict), "fixtures must not build a list again"
        assert tuple(pos) == obs_fixtures.POSITION_KEYS

    def test_accessor_reads_the_fixture(self):
        """The end the executor actually depends on."""
        assert position_xy(obs_fixtures.detection("cone", (1.0, 2.0))) == (1.0, 2.0)


class TestAccessorToleratesBothShapes:
    """position_xy is the single reader; it must not care which shape it gets."""

    @pytest.mark.parametrize("pos,expected", [
        ({"x": 1.5, "y": 2.5, "z": 0.3}, (1.5, 2.5)),   # the logger's shape
        ([1.5, 2.5], (1.5, 2.5)),                        # legacy / hand-written
        ((1.5, 2.5, 0.3), (1.5, 2.5)),
        ({"x": 1.5, "y": 2.5}, (1.5, 2.5)),              # no z
    ])
    def test_reads(self, pos, expected):
        assert position_xy({"position_map": pos}) == expected

    @pytest.mark.parametrize("pos", [
        None, "nope", [1.5], {}, {"x": None, "y": 2.5}, {"x": 1.5},
    ])
    def test_unusable_position_is_none_not_an_exception(self, pos):
        """A crash here takes the executor down mid-mission; None does not."""
        assert position_xy({"position_map": pos}) is None

    def test_missing_key_and_none_object(self):
        assert position_xy({}) is None
        assert position_xy(None) is None
