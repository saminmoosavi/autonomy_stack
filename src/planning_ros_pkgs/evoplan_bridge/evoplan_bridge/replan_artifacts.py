#!/usr/bin/env python3
"""One naming scheme for every plan a trial produces, from both sides of it.

A trial's plans come from two processes that never talk to each other about
files. The host replan service writes an artifact per replan; the executor
loads the *initial* plan straight off disk from a mission `.txt` and never
consults the service at all. So iteration 0 -- the plan the robot actually
starts driving -- had no artifact, and reconstructing a run meant reading
`_replan_1.pddl` and inferring backwards what it had replaced.

Both sides now write through here:

    <tag>_replan_0.pddl    the authored mission problem (executor)
    <tag>_replan_0.plan    the plan file it loaded
    <tag>_replan_0.json    where both came from
    <tag>_replan_1.pddl    the first runtime problem (service)
    ...

Sharing the module is the point, not an economy. The sequence is only readable
if every filename in it is built the same way, and two independent
implementations of "sanitise a tag, then find the next index" would drift on
exactly the inputs that matter -- an unusual tag, a restarted service, a
re-run trial. The service's numbering already derives the next index from
disk (``max(existing) + 1``), so a ``_replan_0`` written first is absorbed
without the service needing to know it exists.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

__all__ = ["safe_tag", "next_index", "artifact_stem", "write_artifacts"]

#: Characters kept in a tag. The tag becomes a filename and, service-side,
#: arrives over HTTP -- anything else (notably "/" and "..") is dropped rather
#: than escaped, so it cannot steer a write out of the results directory.
_TAG_SAFE = re.compile(r"[^A-Za-z0-9._-]+")


def safe_tag(tag: str | None, fallback: str = "unknown") -> str:
    """Sanitise a trial tag into a filename component.

    Falls back rather than returning empty: a tag that sanitises away is a
    reason to file the artifact somewhere odd, never a reason to discard the
    only record of a plan.
    """
    cleaned = _TAG_SAFE.sub("_", str(tag or "").strip()).strip("._-")
    return cleaned or _TAG_SAFE.sub("_", str(fallback)).strip("._-") or "unknown"


def next_index(out_dir: Path, tag: str, minimum: int = 0) -> int:
    """Next free ``_replan_N`` index for ``tag``, read from disk.

    From disk and not an in-memory counter because no single process sees the
    whole sequence: index 0 is written by the executor in the container, 1..N
    by a host service that outlives the trial (KEEP_SERVICE=1) and may restart
    mid-run.

    ``minimum`` is how index 0 stays reserved. The service passes 1, so a trial
    whose executor never archived an initial plan still numbers its replans
    from 1 -- the index means "the Nth replan" in every trial, including the
    ones recorded before index 0 existed, and a run with no ``_replan_0`` reads
    as "the initial plan was not captured" rather than shifting everything down
    by one.

    ``max`` and not a count, so a deleted artifact cannot make the next write
    land on top of a later one.
    """
    pattern = re.compile(rf"{re.escape(tag)}_replan_(\d+)\.pddl")
    existing = [int(m.group(1)) for p in Path(out_dir).glob(f"{tag}_replan_*.pddl")
                if (m := pattern.fullmatch(p.name))]
    return max(max(existing, default=-1) + 1, minimum)


def artifact_stem(out_dir: Path, tag: str, index: int) -> Path:
    return Path(out_dir) / f"{tag}_replan_{index}"


def write_artifacts(out_dir, tag: str, index: int, problem_text: str | None,
                    plan_lines, meta: dict) -> Path:
    """Write the ``.pddl``/``.plan``/``.json`` triple. Returns the stem.

    All three are written on every path, including failure -- an empty
    ``.plan`` next to a populated ``.pddl`` is itself the finding, and a
    missing file is indistinguishable from a trial that never got that far.
    Callers are expected to wrap this: losing a diagnostic must never break the
    thing being diagnosed.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = artifact_stem(out_dir, tag, index)
    stem.with_suffix(".pddl").write_text(problem_text or "")
    stem.with_suffix(".plan").write_text(
        "".join(f"{line}\n" for line in (plan_lines or [])))
    stem.with_suffix(".json").write_text(json.dumps(meta, indent=2) + "\n")
    return stem
