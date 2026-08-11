#!/usr/bin/env python3
"""Take the executable prefix of a plan that does not reach its goal.

A find-an-object mission asks for something classical planning cannot deliver:
the goal names an object whose position is unknown, so no sound plan reaches
it. The useful answer is not "no plan". It is the part of the plan that IS
executable -- which, for a goal of "survey these regions, then go to the cone",
is the survey. Drive that, look at what perception found, replan the approach
against a real position.

VAL answers valid/invalid and nothing in between, so the prefix is computed
here by forward simulation against the same STRIPS semantics the executor's own
validator uses (``pddl_stl.pipeline.validate_plan``, which already returns an
``accepted_prefix_len`` for exactly this reason). Reusing that module rather
than reimplementing it keeps one definition of "applicable" in the codebase.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "src" / "planning_ros_pkgs" / "evo_skill_ros"))

from evo_skill_ros.pddl_stl import pipeline as pddl  # noqa: E402

__all__ = ["PrefixResult", "executable_prefix"]


class PrefixResult:
    """Outcome of trimming a plan to what can actually run.

    ``reaches_goal`` is the distinction that matters to a caller: a full-length
    prefix with ``reaches_goal=False`` means every action was applicable but
    the goal is still open -- the normal outcome when the goal names an
    unlocated object, and the signal that a survey has been produced rather
    than a solution.
    """

    def __init__(self, actions, prefix_len, total, reaches_goal, errors):
        self.actions = actions
        self.prefix_len = prefix_len
        self.total = total
        self.reaches_goal = reaches_goal
        self.errors = errors

    @property
    def truncated(self) -> bool:
        return self.prefix_len < self.total

    @property
    def plan(self) -> list[str]:
        return [a.text() for a in self.actions]

    def __repr__(self) -> str:
        return (f"PrefixResult({self.prefix_len}/{self.total} actions, "
                f"reaches_goal={self.reaches_goal})")


def executable_prefix(domain_text: str, problem_text: str, plan_lines) -> PrefixResult:
    """Longest applicable prefix of ``plan_lines`` under ``domain``/``problem``.

    Stops at the first action whose preconditions are unmet in the state
    reached so far. Unparseable or unknown actions truncate rather than raise:
    the plan text comes from an LLM, and one malformed line late in a candidate
    should cost that line, not the whole survey.
    """
    lines = [ln.strip() for ln in plan_lines
             if ln.strip() and not ln.strip().startswith(";")]
    with tempfile.TemporaryDirectory(prefix="plan_prefix_") as tmp:
        tmp = Path(tmp)
        (tmp / "domain.pddl").write_text(domain_text)
        (tmp / "problem.pddl").write_text(problem_text)
        (tmp / "plan.txt").write_text("".join(f"{ln}\n" for ln in lines))
        domain = pddl.parse_domain(tmp / "domain.pddl")
        problem = pddl.parse_problem(tmp / "problem.pddl")
        actions = pddl.parse_plan(tmp / "plan.txt")
        # require_goal=False: an unmet goal is the expected case here and must
        # not collapse accepted_prefix_len, which require_goal=True reports as
        # len(plan) alongside ok=False. Applicability is what is being asked.
        applicable = pddl.validate_plan(domain, problem, actions, require_goal=False)
        prefix = actions[:applicable.accepted_prefix_len]
        reaches = pddl.validate_plan(domain, problem, prefix, require_goal=True).ok
    return PrefixResult(prefix, len(prefix), len(actions), reaches,
                        applicable.errors)
