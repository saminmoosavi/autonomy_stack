#!/usr/bin/env python3
"""Planner backends for the replan service.

Three modes, trading latency against quality:

``fd_only``
    Fast Downward, 1-3 s. The ablation baseline and the degraded fallback.
``fd_first``
    Try FD under a short cap; fall through to EvoPlan if it fails. The best
    default when latency matters, which online it always does.
``evoplan``
    The LLM evolutionary loop. Seconds to tens of seconds.

Every mode runs an unsolvability check first: if ``(blocked ...)`` has made the
problem impossible, FD proves it in seconds and the service can say so, instead
of burning the entire deliberation budget on an LLM that cannot succeed.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

__all__ = ["PlanResult", "validate_with_val", "run_fast_downward", "FD_SEARCH"]

FD_SEARCH = "astar(lmcut())"


@dataclass
class PlanResult:
    status: str = "failed"          # ok | failed | timeout | unsolvable
    plan: list = field(default_factory=list)
    planner: str = "none"
    valid: bool = False
    elapsed_s: float = 0.0
    validator_output: str = ""
    error: str | None = None
    #: True when the blocked-region hints had to be dropped to find any plan.
    relaxed: bool = False


def _plan_lines(text: str) -> list[str]:
    """Keep only grounded-action lines, lowercased, comments dropped."""
    out = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith(";"):
            continue
        match = re.search(r"\(([^)]+)\)", line)
        if match:
            out.append("(" + " ".join(match.group(1).lower().split()) + ")")
    return out


def val_binary() -> str | None:
    """Locate VAL the same way ``pddl_evolve/evaluator.py`` does."""
    return (
        shutil.which("validate")
        or os.environ.get("VAL_BIN")
        or (("/usr/local/bin/validate")
            if Path("/usr/local/bin/validate").is_file() else None)
    )


def validate_with_val(domain_text: str, problem_text: str, plan_lines: list[str],
                      timeout_s: float = 20.0) -> tuple[bool, str]:
    """Validate a plan with VAL. Returns ``(valid, validator_output)``.

    The output text matters as much as the boolean: VAL's "Plan Repair Advice"
    names the exact unsatisfied precondition, and that is what gets fed back to
    the LLM as OpenEvolve artifacts.
    """
    binary = val_binary()
    if binary is None:
        return False, "VAL not available"
    with tempfile.TemporaryDirectory(prefix="evoplan_val_") as tmp:
        tmp = Path(tmp)
        (tmp / "domain.pddl").write_text(domain_text)
        (tmp / "problem.pddl").write_text(problem_text)
        (tmp / "plan.txt").write_text("\n".join(plan_lines) + "\n")
        try:
            proc = subprocess.run(
                [binary, "-v", str(tmp / "domain.pddl"), str(tmp / "problem.pddl"),
                 str(tmp / "plan.txt")],
                capture_output=True, text=True, timeout=timeout_s,
            )
        except subprocess.TimeoutExpired:
            return False, "VAL timed out"
        output = (proc.stdout or "") + (proc.stderr or "")
        return bool(re.search(r"^Plan valid", output, re.M)), output[-3000:]


def run_fast_downward(fd_path: str, domain_text: str, problem_text: str,
                      timeout_s: float = 10.0, search: str = FD_SEARCH) -> PlanResult:
    """Run Fast Downward and parse ``sas_plan``.

    Distinguishes "no solution exists" (exit 12, ``unsolvable``) from a crash or
    a timeout -- the caller must not retry an unsolvable problem with an LLM.
    """
    started = time.monotonic()
    result = PlanResult(planner="fd")
    # Absolute: FD runs with cwd set to the tempdir (so its intermediate
    # output.sas lands there), which would otherwise break a relative path.
    fd_path = Path(fd_path).resolve()
    with tempfile.TemporaryDirectory(prefix="evoplan_fd_") as tmp:
        tmp = Path(tmp)
        domain = tmp / "domain.pddl"
        problem = tmp / "problem.pddl"
        sas = tmp / "sas_plan"
        domain.write_text(domain_text)
        problem.write_text(problem_text)
        try:
            proc = subprocess.run(
                ["python3", str(fd_path), "--plan-file", str(sas),
                 str(domain), str(problem), "--search", search],
                capture_output=True, text=True, timeout=timeout_s, cwd=tmp,
            )
        except subprocess.TimeoutExpired:
            result.status = "timeout"
            result.error = f"fast-downward exceeded {timeout_s}s"
            result.elapsed_s = time.monotonic() - started
            return result
        except FileNotFoundError as exc:
            result.error = str(exc)
            result.elapsed_s = time.monotonic() - started
            return result

        output = (proc.stdout or "") + (proc.stderr or "")
        result.validator_output = output[-3000:]
        result.elapsed_s = time.monotonic() - started

        # FD exit codes: 0/12 unsolvable-ish family; sas_plan present == solved.
        if sas.is_file():
            result.plan = _plan_lines(sas.read_text())
            result.status = "ok" if result.plan else "failed"
            result.valid = bool(result.plan)
            return result
        if "Search stopped without finding a solution" in output or proc.returncode == 12:
            result.status = "unsolvable"
            result.error = "no plan exists for this problem"
            return result
        result.status = "failed"
        result.error = f"fast-downward returned {proc.returncode} with no plan"
        return result
