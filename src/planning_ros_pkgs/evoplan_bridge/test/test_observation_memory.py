"""Tests for searching the robot's observation memory.

Fixtures mirror the real observations.jsonl schema, including the two quirks a
live run exposed: spurious low-evidence labels (kite, snowboard, surfboard,
teddy_bear all appeared in one trial), and `region: null` on detections that
nonetheless carry a usable `position_map`.
"""

import json

import pytest

from evoplan_bridge.observation_memory import (
    DEFAULT_MIN_HITS,
    load_observations,
    locate_object,
    observed_in_region,
    summarize_for_planner,
)

import obs_fixtures

# graph.json coordinates for the regions used below, and the surveyed position
# of testobj_backpack_r5 from warehouse_objects_test.sdf.
REGIONS = {"r5": [1.44, 2.74], "r11": [-3.72, 23.68], "r8": [3.52, 13.01]}
BACKPACK_XY = [3.04, 2.74]


# Both delegate to obs_fixtures, which builds the position_map MAPPING the
# logger really writes. These used to hand-build `"position_map": xy` as a
# list, which no live run has ever produced -- see test_obs_fixture_contract.
det = obs_fixtures.detection
rec = obs_fixtures.record


@pytest.fixture
def memory():
    """12 backpack sightings at r5, plus noise and a moving person."""
    records = [rec(t, [det("backpack", BACKPACK_XY, 0.71)]) for t in range(10, 22)]
    # Single-frame hallucinations, exactly as observed in a real trial.
    records.append(rec(30, [det("kite", [3.0, 2.7], 0.55)]))
    records.append(rec(31, [det("snowboard", [3.1, 2.8], 0.52)]))
    # A person near r11 -- moves, so must not be planned against.
    records += [rec(t, [det("person", [-3.7, 23.6], 0.9, obj_type="human")])
                for t in (40, 41, 42, 43)]
    return records


class TestLoad:
    def test_skips_malformed_lines(self, tmp_path):
        """The logger writes continuously; a truncated final line is normal."""
        p = tmp_path / "obs.jsonl"
        p.write_text(json.dumps(rec(1, [])) + "\n"
                     + "{not valid json\n"
                     + json.dumps(rec(2, [])) + "\n"
                     + '{"seq": 3, "objects": [')          # truncated mid-write
        assert len(load_observations(p)) == 2

    def test_missing_file_is_empty_not_an_error(self, tmp_path):
        assert load_observations(tmp_path / "nope.jsonl") == []


class TestPresence:
    def test_object_present_where_it_is(self, memory):
        assert len(observed_in_region(memory, "backpack", "r5", regions=REGIONS)) == 12

    def test_object_absent_where_it_is_not(self, memory):
        """The inspection-failure case that triggers the replan."""
        assert observed_in_region(memory, "backpack", "r11", regions=REGIONS) == []

    def test_time_window_scopes_the_check(self, memory):
        """Presence must mean 'while I was standing there', not 'ever'."""
        assert observed_in_region(memory, "backpack", "r5",
                                  since_ros_sec=100.0, regions=REGIONS) == []
        assert observed_in_region(memory, "backpack", "r5", since_ros_sec=15.0,
                                  until_ros_sec=18.0, regions=REGIONS)

    def test_score_floor_applies(self, memory):
        assert observed_in_region(memory, "backpack", "r5",
                                  min_score=0.9, regions=REGIONS) == []

    def test_case_insensitive(self, memory):
        assert observed_in_region(memory, "BACKPACK", "R5", regions=REGIONS)


class TestRecall:
    def test_finds_the_unique_object(self, memory):
        found = locate_object(memory, "backpack", regions=REGIONS)
        assert len(found) == 1
        assert found[0]["region"] == "r5"
        assert found[0]["hits"] == 12
        assert found[0]["xy"] == pytest.approx(BACKPACK_XY, abs=0.05)

    def test_never_seen_returns_empty(self, memory):
        assert locate_object(memory, "elephant", regions=REGIONS) == []

    def test_single_frame_noise_is_rejected(self, memory):
        """kite/snowboard appeared once each in a real run -- must not count."""
        assert locate_object(memory, "kite", regions=REGIONS) == []
        assert locate_object(memory, "snowboard", regions=REGIONS) == []

    def test_min_hits_is_the_boundary(self):
        obs = [rec(t, [det("banana", [3.52, 13.01])]) for t in range(DEFAULT_MIN_HITS - 1)]
        assert locate_object(obs, "banana", regions=REGIONS) == []
        obs.append(rec(99, [det("banana", [3.52, 13.01])]))
        assert locate_object(obs, "banana", regions=REGIONS)[0]["region"] == "r8"

    def test_strongest_evidence_first(self):
        """An ambiguous object must rank regions, not pick arbitrarily."""
        obs = [rec(t, [det("suitcase", [1.44, 2.74])]) for t in range(3)]
        obs += [rec(t, [det("suitcase", [3.52, 13.01])]) for t in range(10, 20)]
        found = locate_object(obs, "suitcase", regions=REGIONS)
        assert [f["region"] for f in found] == ["r8", "r5"]
        assert found[0]["hits"] > found[1]["hits"]


class TestRegionAttribution:
    def test_position_map_beats_a_null_region_label(self):
        """observation_logger writes region=null when TF is unavailable, but
        position_map is still good -- geometry must win."""
        obs = [rec(t, [det("backpack", BACKPACK_XY, region=None)]) for t in range(5)]
        assert locate_object(obs, "backpack", regions=REGIONS)[0]["region"] == "r5"

    def test_far_from_any_region_is_unattributed(self):
        """Regions are points, not polygons; do not snap to a distant centroid."""
        obs = [rec(t, [det("backpack", [500.0, 500.0])]) for t in range(5)]
        assert locate_object(obs, "backpack", regions=REGIONS) == []

    def test_falls_back_to_logged_region_without_a_table(self):
        obs = [rec(t, [det("backpack", BACKPACK_XY, region="r5")]) for t in range(5)]
        assert locate_object(obs, "backpack")[0]["region"] == "r5"


class TestPlannerSummary:
    def test_excludes_people(self, memory):
        """People move; 'where a person was' is not a fact to plan against."""
        summary = summarize_for_planner(memory, regions=REGIONS)
        assert "person" not in summary
        assert summary["backpack"][0]["region"] == "r5"

    def test_excludes_noise(self, memory):
        summary = summarize_for_planner(memory, regions=REGIONS)
        assert "kite" not in summary and "snowboard" not in summary

    def test_class_filter(self, memory):
        summary = summarize_for_planner(memory, classes=["backpack"], regions=REGIONS)
        assert set(summary) == {"backpack"}

    def test_empty_memory_is_empty_summary(self):
        assert summarize_for_planner([]) == {}

    def test_summary_is_json_serialisable(self, memory):
        """It crosses to the host service as JSON."""
        json.dumps(summarize_for_planner(memory, regions=REGIONS))
