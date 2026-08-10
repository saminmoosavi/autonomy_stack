#!/usr/bin/env python3
"""Runtime-aware evaluator for online EvoPlan repair.

Derived from ``pddl_evolve/evaluator.py`` following the pattern established by
``blocksworld/evaluator_hybrid.py``: import the base evaluator, reuse its
validator and scoring helpers, insert extra steps, return the identical
``EvaluationResult`` shape so OpenEvolve's database config needs no change.

What it adds over the base:

1. **Runtime feasibility.** A plan that does not start where the robot actually
   is, or that routes through a region execution just proved impassable, scores
   zero. VAL catches the blocked case via ``(not (blocked ?to))``, but catching
   it here is instant and produces far better feedback text.
2. **Length shaping.** Online, a short valid plan beats a long valid one -- the
   robot has to drive it while people walk around it.
3. **Runtime context in artifacts.** The prose explaining *why* execution failed
   is surfaced to the next iteration's prompt. That feedback channel is the
   whole reason to use EvoPlan here instead of a classical replanner.

Context arrives via ``$EVOPLAN_CONTEXT_JSON``, mirroring how ``bw_worker.sh``
uses ``EVAL=`` and ``owt_loop.py`` uses ``OWT_EVALUATOR`` to parameterise an
evaluator without editing it.
"""

from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO / "evolve_stl_pddl" / "pddl_evolve"))

import evaluator as base  # noqa: E402  pddl_evolve/evaluator.py
from evaluator import EvaluationResult  # noqa: E402

#: Per-action penalty for exceeding the shortest known plan length.
LENGTH_PENALTY = 0.02


def _context() -> dict:
    raw = os.environ.get("EVOPLAN_CONTEXT_JSON")
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {}


def _actions(plan_text: str):
    """``(name, [args])`` per grounded action line."""
    out = []
    for line in plan_text.splitlines():
        match = re.search(r"\(([^)]+)\)", line.strip().lower())
        if match:
            parts = match.group(1).split()
            if parts:
                out.append((parts[0], parts[1:]))
    return out


def check_runtime_feasibility(plan_text: str, ctx: dict) -> list[str]:
    """Runtime violations, as prose the LLM can act on. Empty means feasible."""
    problems = []
    actions = _actions(plan_text)
    if not actions:
        return ["The plan is empty."]

    current = (ctx.get("current_region") or "").lower()
    moves = [(n, a) for n, a in actions if n.startswith("move")]
    if current and moves:
        first_from = moves[0][1][-2] if len(moves[0][1]) >= 2 else None
        if first_from and first_from != current:
            problems.append(
                f"The first (move ...) departs from '{first_from}', but the robot is "
                f"currently at '{current}'. The plan must start from '{current}'."
            )

    blocked = {r.lower() for r in (ctx.get("blocked_regions") or [])}
    for name, args in moves:
        if args and args[-1].lower() in blocked:
            problems.append(
                f"({name} {' '.join(args)}) enters '{args[-1]}', which execution has "
                f"proved impassable. Route around it."
            )
    return problems


def evaluate(program_path: str) -> EvaluationResult:
    """OpenEvolve fitness entry point."""
    ctx = _context()
    result = base.evaluate(program_path)

    metrics = dict(result.metrics)
    artifacts = dict(result.artifacts or {})

    # Always give the LLM the runtime story, valid plan or not.
    if ctx.get("reason_text"):
        artifacts["runtime_context"] = ctx["reason_text"]

    try:
        module = base._load_evolved_program(program_path)
        plan_text = base.normalize_plan_text(base._extract(module, "PLAN", "get_plan") or "")
    except Exception:
        plan_text = ""

    violations = check_runtime_feasibility(plan_text, ctx) if plan_text else []
    if violations:
        # A symbolically valid plan the robot cannot actually start is worthless
        # online, so it must not outrank a plan that is merely incomplete.
        artifacts["runtime_violations"] = "\n".join(violations)
        metrics["combined_score"] = 0.0
        metrics["score"] = 0.0
        metrics["runtime_feasible"] = 0.0
        return EvaluationResult(metrics=metrics, artifacts=artifacts)

    metrics["runtime_feasible"] = 1.0
    if metrics.get("valid"):
        shortest = int(ctx.get("shortest_known_length") or 0)
        length = int(metrics.get("plan_length") or len(_actions(plan_text)))
        if shortest and length > shortest:
            penalty = LENGTH_PENALTY * (length - shortest)
            metrics["combined_score"] = max(0.0, 1.0 - penalty)
            artifacts["length_note"] = (
                f"Valid, but {length} actions against a {shortest}-action reference. "
                "Shorter plans score higher."
            )
    return EvaluationResult(metrics=metrics, artifacts=artifacts)
