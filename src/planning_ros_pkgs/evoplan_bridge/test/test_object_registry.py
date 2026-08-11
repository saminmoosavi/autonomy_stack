"""Identity is the whole job: the same object must keep the same name.

``cluster_detections`` re-clusters the entire observation log from scratch on
every poll, because a later detection can sharpen a centroid or merge two
clusters. That is correct and it is also why identity cannot live there: the
clustering has no memory, and a mission that renames its objects between polls
cannot tell one it has already inspected from one it has not.

These tests pin the two failures that would follow from getting it wrong:

* **re-minting** -- the same cone comes back as ``traffic_cone_2`` on the next
  poll, the goal gains a conjunct for an object that is already done, and the
  robot drives back to inspect it again;
* **over-merging** -- two real cones a metre and a half apart collapse into one
  registry entry, so one of them is inspected twice while the other is never
  visited and nothing in the logs says so.

The mission's own memory (``inspected``) is tested here too, because it is
carried across polls by the same objects: perception seeing something again is
not a reason to un-inspect it.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "src" / "planning_ros_pkgs" / "evoplan_bridge"))

from evoplan_bridge.object_registry import ObjectRegistry  # noqa: E402


def cluster(class_name, xy, region="r1", hits=5, mean_score=0.8):
    return {"class_name": class_name, "xy": list(xy), "region": region,
            "hits": hits, "mean_score": mean_score}


class TestMinting:
    def test_first_sighting_mints_a_pddl_legal_name(self):
        registry = ObjectRegistry()
        new = registry.update([cluster("traffic cone", (1.0, 1.0))])
        assert [t.name for t in new] == ["traffic_cone_1"]
        # A space is not legal in a PDDL symbol, and the detector class has one.
        assert " " not in new[0].name

    def test_names_are_numbered_per_class(self):
        registry = ObjectRegistry()
        registry.update([cluster("traffic cone", (1.0, 1.0)),
                         cluster("chair", (9.0, 9.0), region="r2"),
                         cluster("traffic cone", (20.0, 20.0), region="r3")])
        assert {t.name for t in registry.all()} == {
            "traffic_cone_1", "chair_1", "traffic_cone_2"}

    def test_two_nearby_objects_stay_two_objects(self):
        """The over-merge failure: 3 m apart is two cones, not one."""
        registry = ObjectRegistry(merge_radius_m=2.5)
        registry.update([cluster("traffic cone", (0.0, 0.0)),
                         cluster("traffic cone", (3.0, 0.0))])
        assert len(registry) == 2

    def test_one_registry_entry_absorbs_at_most_one_cluster(self):
        """Two clusters inside the merge radius must not both match the same
        entry -- the second would overwrite the first's position and the
        further object would never get a name."""
        registry = ObjectRegistry(merge_radius_m=5.0)
        registry.update([cluster("traffic cone", (0.0, 0.0))])
        registry.update([cluster("traffic cone", (0.2, 0.0), hits=9),
                         cluster("traffic cone", (2.0, 0.0), hits=4)])
        assert len(registry) == 2


class TestStability:
    def test_re_polling_does_not_mint_again(self):
        registry = ObjectRegistry()
        registry.update([cluster("traffic cone", (1.0, 1.0), hits=3)])
        again = registry.update([cluster("traffic cone", (1.0, 1.0), hits=7)])
        assert again == [], "the same object was minted a second time"
        assert registry.get("traffic_cone_1").hits == 7, "evidence did not update"

    def test_a_drifting_centroid_is_the_same_object(self):
        """A cluster's centroid moves as detections accumulate. That is not a
        second cone."""
        registry = ObjectRegistry(merge_radius_m=2.5)
        registry.update([cluster("traffic cone", (1.0, 1.0))])
        registry.update([cluster("traffic cone", (2.2, 1.4))])
        assert len(registry) == 1
        assert registry.get("traffic_cone_1").xy == (2.2, 1.4)

    def test_region_follows_the_newest_evidence(self):
        registry = ObjectRegistry()
        registry.update([cluster("chair", (1.0, 1.0), region="r1")])
        registry.update([cluster("chair", (1.2, 1.0), region="r2")])
        assert registry.get("chair_1").region == "r2"

    def test_incomplete_clusters_are_ignored(self):
        registry = ObjectRegistry()
        assert registry.update([
            {"class_name": "chair", "xy": None, "region": "r1"},
            {"class_name": "", "xy": [1.0, 1.0], "region": "r1"},
            {"class_name": "chair", "xy": [1.0, 1.0], "region": ""},
        ]) == []
        assert len(registry) == 0


class TestInspectionMemory:
    def test_seeing_it_again_does_not_un_inspect_it(self):
        registry = ObjectRegistry()
        registry.update([cluster("traffic cone", (1.0, 1.0))])
        registry.mark_inspected("traffic_cone_1", at=42.0)
        registry.update([cluster("traffic cone", (1.1, 1.0), hits=30)])
        target = registry.get("traffic_cone_1")
        assert target.inspected and target.inspected_at == 42.0
        assert registry.pending() == []

    def test_pending_is_what_drives_the_next_round(self):
        registry = ObjectRegistry()
        registry.update([cluster("traffic cone", (1.0, 1.0)),
                         cluster("chair", (9.0, 9.0), region="r2")])
        registry.mark_inspected("chair_1")
        assert [t.name for t in registry.pending()] == ["traffic_cone_1"]

    def test_marking_an_unknown_name_is_a_no_op(self):
        assert ObjectRegistry().mark_inspected("nothing_1") is False

    def test_instances_keep_inspected_objects(self):
        """They must stay in the problem: dropping one deletes the
        (object-at ...) that explains its (inspected-object ...) goal."""
        registry = ObjectRegistry()
        registry.update([cluster("traffic cone", (1.0, 1.0)),
                         cluster("chair", (9.0, 9.0), region="r2")])
        registry.mark_inspected("chair_1")
        assert {i["name"] for i in registry.instances()} == {
            "traffic_cone_1", "chair_1"}
        assert {i["name"] for i in registry.instances(pending_only=True)} == {
            "traffic_cone_1"}

    def test_summary_counts_what_the_mission_reports(self):
        registry = ObjectRegistry()
        registry.update([cluster("traffic cone", (1.0, 1.0)),
                         cluster("traffic cone", (9.0, 9.0), region="r2")])
        registry.mark_inspected("traffic_cone_1")
        summary = registry.summary()
        assert summary["total"] == 2
        assert summary["inspected"] == 1
        assert summary["pending"] == ["traffic_cone_2"]
        assert summary["by_class"] == {"traffic cone": 2}
