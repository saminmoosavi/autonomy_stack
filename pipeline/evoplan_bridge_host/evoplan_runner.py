#!/usr/bin/env python3
"""Drive OpenEvolve to repair a plan online.

The ``evolve_stl_pddl`` submodule is owned by another repository and is kept
pristine, so this does not fork ``run_evoplan.py`` -- it *imports* it and reuses
``write_seed`` and ``extract_plan`` verbatim. Only the ~6 lines that choose the
evaluator differ, and ``run_evoplan.main`` hardcodes ``HERE / "evaluator.py"``,
which is the one thing that had to be replaced.

Online this is a *repair*, not a synthesis: the seed plan is the remaining
suffix of a plan that was already working, so a handful of iterations usually
suffices where the offline benchmark needed 15-30.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
EVOLVE = REPO / "evolve_stl_pddl"
sys.path.insert(0, str(EVOLVE / "pddl_evolve"))

from planners import PlanResult, validate_with_val, _plan_lines  # noqa: E402

__all__ = ["run_evoplan_repair"]


def _import_run_evoplan():
    """Import the submodule's driver for its seed/extract helpers."""
    import run_evoplan  # noqa: PLC0415 - deliberately late, needs sys.path set up
    return run_evoplan


def run_evoplan_repair(job, domain_text, problem_text, payload, deadline_s,
                       fallback: PlanResult, args, effective_blocked=None,
                       relaxed=False) -> PlanResult:
    """Evolve a repaired plan. Falls back to ``fallback`` on any failure.

    The robot is holding station while this runs, so every path must terminate:
    a hard subprocess timeout, and on timeout the best validated program
    OpenEvolve produced so far rather than nothing.
    """
    started = time.monotonic()
    openevolve_run = Path(args.openevolve_run)
    config = Path(args.evoplan_config)
    if not openevolve_run.is_file():
        fallback.error = f"openevolve-run.py not found at {openevolve_run}; using FD result"
        return fallback
    if not config.is_file():
        fallback.error = f"evoplan config not found at {config}; using FD result"
        return fallback

    try:
        rp = _import_run_evoplan()
    except ImportError as exc:
        fallback.error = f"could not import run_evoplan ({exc}); using FD result"
        return fallback

    work = Path(os.environ.get("EVOPLAN_WORK_DIR", "/tmp")) / f"evoplan_{job.id}"
    work.mkdir(parents=True, exist_ok=True)

    # The seed is the plan suffix still to be executed: repair, not synthesis.
    seed_plan = "\n".join(payload.get("remaining_plan") or fallback.plan or [])
    seed = rp.write_seed(work, domain_text, problem_text, seed_plan,
                         label=f"{payload.get('mission_id')} replan {job.id}")

    reason = (payload.get("reason") or {})
    env = dict(os.environ)
    env["EVOPLAN_CONTEXT_JSON"] = json.dumps({
        "current_region": payload.get("current_region"),
        # The blocked list actually reflected in the problem. If the service
        # relaxed it (the goal was unreachable otherwise), this is empty --
        # otherwise the evaluator scores every candidate 0.0 for entering a
        # region the problem no longer forbids.
        "blocked_regions": (effective_blocked
                            if effective_blocked is not None
                            else payload.get("blocked_regions") or []),
        "blocked_relaxed": relaxed,
        # The prose that becomes OpenEvolve feedback in the next iteration.
        "reason_text": reason.get("human_text"),
        "shortest_known_length": len(fallback.plan) if fallback.plan else 0,
    })
    # The base evaluator falls back to the seed if the LLM drops DOMAIN_PDDL.
    env["INITIAL_PROGRAM_PATH"] = str(seed)
    env.setdefault("FD_PATH", str(args.fd_path))

    # Model override without editing the YAML: rewrite primary_model into a
    # per-run copy of the config. This mirrors how the submodule's .job files
    # sed the api_base into their configs before each SLURM run.
    model = os.environ.get("EVOPLAN_MODEL") or getattr(args, "evoplan_model", None)
    if model:
        run_config = work / "config.yaml"
        run_config.write_text(
            re.sub(r'^(\s*primary_model:\s*).*$', rf'\g<1>"{model}"',
                   config.read_text(), count=1, flags=re.M)
        )
        config = run_config

    out_dir = work / "oe"
    remaining = max(5.0, deadline_s - (time.monotonic() - started))
    command = [
        sys.executable, str(openevolve_run), str(seed), str(HERE / "evaluator_sim.py"),
        "--config", str(config), "--output", str(out_dir),
        "--iterations", str(args.evoplan_iterations),
    ]

    timed_out = False
    try:
        proc = subprocess.run(command, capture_output=True, text=True,
                              timeout=remaining, env=env, cwd=work)
        tail = ((proc.stdout or "") + (proc.stderr or ""))[-3000:]
    except subprocess.TimeoutExpired:
        timed_out = True
        tail = f"openevolve exceeded the {remaining:.0f}s online deadline"
    except Exception as exc:  # noqa: BLE001
        fallback.error = f"openevolve failed ({exc}); using FD result"
        return fallback

    # Best-so-far even on timeout: partial credit beats stranding the robot.
    plan_text = rp.extract_plan(out_dir / "best" / "best_program.py")
    if not plan_text:
        fallback.error = (
            f"evoplan produced no plan ({'timeout' if timed_out else 'no best program'}); "
            "using FD result"
        )
        fallback.validator_output = tail
        return fallback

    plan = _plan_lines(plan_text)
    valid, validator_output = validate_with_val(domain_text, problem_text, plan)

    if not valid and fallback.status == "ok" and fallback.valid:
        # A validated FD plan beats an unvalidated evolved one every time.
        fallback.error = "evoplan plan failed validation; using FD result"
        fallback.validator_output = validator_output
        return fallback

    return PlanResult(
        status="timeout" if timed_out else ("ok" if valid else "failed"),
        plan=plan,
        planner="evoplan",
        valid=valid,
        elapsed_s=time.monotonic() - started,
        validator_output=validator_output,
        error=None if valid else "evolved plan did not validate",
        relaxed=getattr(fallback, "relaxed", False),
    )
