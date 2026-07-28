#!/usr/bin/env python3
"""Build a TEMPORARY warehouse world with searchable objects along plan.txt.

Places one Fuel-model object near each region the plan routes through, so the
observation logger has non-person, non-scenery objects to detect and attribute
to a region. Output goes to worlds/_variants/ (gitignored, same place
make_density_world.py writes its variants).

  python3 worlds/gen_object_test_world.py
  # then, inside the container:
  WORLD=$WS/worlds/_variants/warehouse_objects_test \
  YOLO_CLASSES=person,chair,suitcase,backpack,banana \
  OBS_LOG=true ./run_sim.sh

Objects sit OFF the region centroid (the robot's actual nav goal) by
PLACEMENT_OFFSET_M, so they are inside camera view without standing on the
waypoint. Each placement is checked against the existing world models and
rejected if it would land within MIN_CLEARANCE_M of one.
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
BASE_WORLD = HERE / "warehouse_people.sdf"
OUT_WORLD = HERE / "_variants" / "warehouse_objects_test.sdf"
GRAPH = REPO / "src/planning_ros_pkgs/evo_skill_ros/config/graph.json"
PLAN = REPO / "src/planning_ros_pkgs/evo_skill_ros/config/plan.txt"

PLACEMENT_OFFSET_M = 1.6   # how far off the region centroid to stand the object
MIN_CLEARANCE_M = 1.2      # reject a placement this close to an existing model

FUEL = "https://fuel.gazebosim.org/1.0"

# region -> (fuel owner/model, yolo-ish label used only for the SDF model name)
# Deliberately large, visually distinct objects: small props (bowls, balls) are
# only a few pixels at camera height from several metres away and YOLO-world
# will never fire on them.
PLACEMENTS = [
    ("r1",  "OpenRobotics/VisitorChair",         "chair"),
    ("r2",  "OpenRobotics/Suitcase1",            "suitcase"),
    ("r5",  "OpenRobotics/JanSport Backpack Red", "backpack"),
    ("r6",  "OpenRobotics/foldable_chair",       "chair"),
    ("r8",  "mjcarroll/Big Banana for Scale",    "banana"),
    ("r9",  "OpenRobotics/Suitcase2",            "suitcase"),
    ("r10", "OpenRobotics/Chair",                "chair"),
    ("r11", "mjcarroll/Banana for Scale",        "banana"),
]

# Candidate offset directions, tried in order until one clears existing models.
DIRECTIONS = [(1, 0), (0, 1), (-1, 0), (0, -1), (0.7, 0.7), (-0.7, 0.7),
              (0.7, -0.7), (-0.7, -0.7)]


def existing_model_xy(world_text: str):
    """(x, y) of every <include> in the base world, ignoring commented-out ones."""
    stripped = re.sub(r"<!--.*?-->", "", world_text, flags=re.S)
    out = []
    for m in re.finditer(r"<include>(.*?)</include>", stripped, re.S):
        pose = re.search(r"<pose>(.*?)</pose>", m.group(1))
        if pose:
            parts = pose.group(1).split()
            out.append((float(parts[0]), float(parts[1])))
    return out


def plan_regions(plan_path: Path):
    """Regions the plan routes through, in order, deduplicated."""
    text = plan_path.read_text()
    seq = []
    for m in re.finditer(r"\(move\s+\S+\s+(\S+)\s+(\S+)\)", text, re.I):
        for r in m.groups():
            r = r.lower()
            if not seq or seq[-1] != r:
                seq.append(r)
    return seq


def main() -> None:
    world = BASE_WORLD.read_text()
    graph = json.loads(GRAPH.read_text())
    regions = {r["name"].lower(): tuple(r["coords"]) for r in graph["regions"]}
    occupied = existing_model_xy(world)
    route = plan_regions(PLAN)

    blocks, placed, skipped = [], [], []
    for region, uri, label in PLACEMENTS:
        if region not in regions:
            skipped.append((region, label, "not in graph.json"))
            continue
        if region not in route:
            skipped.append((region, label, "not on the plan route"))
            continue
        cx, cy = regions[region]

        spot = None
        for dx, dy in DIRECTIONS:
            x = cx + dx * PLACEMENT_OFFSET_M
            y = cy + dy * PLACEMENT_OFFSET_M
            if all(math.hypot(x - ox, y - oy) >= MIN_CLEARANCE_M for ox, oy in occupied):
                spot = (x, y)
                break
        if spot is None:
            skipped.append((region, label, "no clear offset near centroid"))
            continue

        x, y = spot
        occupied.append((x, y))          # later objects avoid this one too
        name = f"testobj_{label}_{region}"
        owner, model = uri.split("/", 1)
        blocks.append(
            f"    <!-- searchable test object for {region.upper()} -->\n"
            f"    <include>\n"
            f"      <uri>{FUEL}/{owner}/models/{model.replace(' ', '%20')}</uri>\n"
            f"      <name>{name}</name>\n"
            f"      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>\n"
            f"      <static>true</static>\n"
            f"    </include>\n"
        )
        placed.append((region, label, x, y, model))

    if "</world>" not in world:
        raise SystemExit(f"{BASE_WORLD} has no </world> tag")
    OUT_WORLD.parent.mkdir(parents=True, exist_ok=True)
    OUT_WORLD.write_text(world.replace("</world>", "".join(blocks) + "  </world>", 1))

    print(f"plan route: {' -> '.join(r.upper() for r in route)}")
    print(f"wrote {OUT_WORLD}  ({len(placed)} objects)")
    for region, label, x, y, model in placed:
        print(f"  {region.upper():4s} {label:9s} ({x:7.2f},{y:7.2f})  {model}")
    for region, label, why in skipped:
        print(f"  SKIPPED {region.upper()} {label}: {why}")
    labels = sorted({p[1] for p in placed})
    print(f"\nYOLO_CLASSES should include: person,{','.join(labels)}")


if __name__ == "__main__":
    main()
