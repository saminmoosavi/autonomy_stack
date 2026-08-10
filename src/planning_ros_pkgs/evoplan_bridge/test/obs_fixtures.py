"""One builder for synthetic observations.jsonl records, shared by every test.

Why this module exists: the fixtures used to write ``position_map`` as
``[x, y]`` while ``observation_logger.py:490`` writes
``{"x": .., "y": .., "z": ..}``. Twelve find-object tests passed against a
shape the system has never produced, and the mismatch surfaced only in a live
run, as ``KeyError: 0`` inside ``locate_object`` -- which killed the executor at
the precise moment the tour finished and the object approach began. The tests
were validating the fixture, not the logger.

So: exactly one definition of "what a logged detection looks like", used by all
tests, and :mod:`test_obs_fixture_contract` pins it against the logger's own
source. Drift now fails a test instead of a mission.

``xy`` is accepted as a plain ``(x, y)`` for the caller's convenience and
converted here -- call sites should never hand-write the wire shape, because
hand-writing it is what broke.
"""

from __future__ import annotations

import json

#: Keys of the ``position_map`` mapping written by observation_logger.
POSITION_KEYS = ("x", "y", "z")


def position_map(xy, z=0.5):
    """The logger's ``position_map`` mapping for a plain (x, y)."""
    return {"x": float(xy[0]), "y": float(xy[1]), "z": float(z)}


def detection(cls, xy, score=0.7, region=None, obj_type="object",
              camera=0, track_id="0", z=0.5):
    """One perceived object, shaped exactly as the logger writes it."""
    return {
        "class_name": cls,
        "object_type": obj_type,
        "score": score,
        "position_map": position_map(xy, z),
        "region": region,
        "region_dist": 1.6,
        "camera": camera,
        "uid": f"{camera}:{cls}",
        "track_id": track_id,
    }


def record(t, objects):
    """One JSONL line: the objects perceived during one log period."""
    return {
        "seq": int(t),
        "ros_time_sec": float(t),
        "frame": "map",
        "n_objects": len(objects),
        "objects": list(objects),
    }


def write_jsonl(path, records):
    """Write records as JSONL and return the path as a str."""
    with open(path, "w") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")
    return str(path)
