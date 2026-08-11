"""An open-world mission file must not contain the answer.

The problem text is not only a diagnostic artifact -- it is the seed EvoPlan
hands the LLM, prose and all. So a survey mission whose header says how many
objects are out there, where they are, or which regions are empty has told the
model everything the run was supposed to discover. The plan can then be written
from the comment with perception contributing nothing, and the trial still
reports success. That is not a subtle failure of the experiment; it is the
experiment inverted, and nothing else in the pipeline would notice.

It happened: a header read "cone_r1 at (-12.29, -10.69) snaps to R1, cone_r3 at
(2.75, -11.57) snaps to R3" and "R4 HOLDS NO CONE".

The line these tests draw:

* the mission's INTENT is legitimate -- what to search, which regions, that the
  object set is unknown and discovered at runtime. The operator chose it;
  discovering it is not the robot's job.
* the world's CONTENTS are not -- counts, positions, which region holds what.
  That is what perception is being asked to produce.

Checked mechanically against the world files, so the check is tied to the actual
ground truth rather than to a hand-maintained list of forbidden words.
"""

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
MISSIONS = sorted((REPO / "pipeline" / "missions").glob("factory_survey_*.pddl"))
WORLDS = sorted((REPO / "worlds").glob("warehouse_people*.sdf"))

#: Models that are scenery in every world: walls, shelves, the robot's
#: surroundings. Naming one gives nothing away -- they are not what a
#: find-and-inspect mission searches for.
SCENERY = re.compile(r"^(ground_plane|wall|shelf|barrier|column|pallet|table|"
                     r"chair_\d+|fchair\d*|_?chair_?\d*)", re.I)


def world_object_names():
    """Model names from the world files that denote a searchable object.

    These are the names of the things a mission is sent to find, so a mission
    file mentioning one is quoting ground truth by definition.
    """
    names = set()
    for world in WORLDS:
        text = world.read_text()
        for match in re.finditer(r"<name>([^<]+)</name>|<model name='([^']+)'",
                                 text):
            name = (match.group(1) or match.group(2) or "").strip()
            if name and not SCENERY.match(name):
                names.add(name)
    return names


@pytest.mark.skipif(not MISSIONS, reason="no survey missions present")
class TestNoGroundTruth:
    @pytest.mark.parametrize("mission", MISSIONS, ids=lambda p: p.name)
    def test_declares_no_targets(self, mission):
        """The premise: a declared object would be an invented one."""
        objects = re.search(r"\(:objects(.*?)\n\s*\)", mission.read_text(), re.S)
        assert objects, f"{mission.name} has no (:objects ...) block"
        body = re.sub(r";[^\n]*", "", objects.group(1))
        assert "- target" not in body, (
            f"{mission.name} declares a target up front; the object set is "
            f"supposed to be the unknown"
        )

    @pytest.mark.parametrize("mission", MISSIONS, ids=lambda p: p.name)
    def test_does_not_name_a_world_object(self, mission):
        """Comments included. The LLM reads the comments."""
        text = mission.read_text().lower()
        named = sorted(n for n in world_object_names() if n.lower() in text)
        assert not named, (
            f"{mission.name} names world object(s) {named}. That is ground "
            f"truth about what is out there, and this file is the seed handed "
            f"to the LLM."
        )

    @pytest.mark.parametrize("mission", MISSIONS, ids=lambda p: p.name)
    def test_states_no_object_count(self, mission):
        """"Two cones" in a header is the answer, written out."""
        text = re.sub(r"\s+", " ", mission.read_text().lower())
        counts = r"(one|two|three|four|five|six|\d+)"
        objects = r"(cone|cones|object|objects|target|targets|instance|instances)"
        # "N objects exist / there are N objects / both objects"
        offenders = re.findall(rf"there (?:are|is) {counts} {objects}", text)
        offenders += re.findall(rf"both {objects}", text)
        offenders += re.findall(rf"{counts} instances? of", text)
        assert not offenders, (
            f"{mission.name} states how many objects exist ({offenders}); the "
            f"count is what the mission is sent to determine"
        )

    @pytest.mark.parametrize("mission", MISSIONS, ids=lambda p: p.name)
    def test_asserts_no_object_position(self, mission):
        """No object-at facts, and no coordinates outside the region model.

        Region centroids and the spawn pose are the problem's own frame of
        reference and are fine. A coordinate that matches an object's pose in a
        world file is not.
        """
        text = mission.read_text()
        assert "object-at" not in re.sub(r";[^\n]*", "", text), (
            f"{mission.name} asserts an object position in :init"
        )
        coords = {(round(float(x), 2), round(float(y), 2))
                  for x, y in re.findall(r"\(\s*(-?\d+\.\d+)[,\s]+(-?\d+\.\d+)", text)}
        if not coords:
            return
        world_poses = set()
        for world in WORLDS:
            for pose in re.findall(r"<pose>(-?\d+\.?\d*)\s+(-?\d+\.?\d*)", world.read_text()):
                world_poses.add((round(float(pose[0]), 2), round(float(pose[1]), 2)))
        leaked = sorted(coords & world_poses)
        assert not leaked, (
            f"{mission.name} contains coordinate(s) {leaked} that match a model "
            f"pose in a world file"
        )
