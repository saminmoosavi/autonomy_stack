#!/usr/bin/env python3
"""Textual splicing of runtime facts into a PDDL problem's ``(:init ...)`` block.

The replan service takes a curated mission problem from
``evolve_stl_pddl/jackal/in/factory_mission_NN.pddl`` and injects what the robot
learned at runtime -- where it actually is, which regions turned out to be
impassable, and which it already visited -- before handing it to a planner.

Splicing text rather than parsing/re-emitting PDDL is deliberate: the mission
files carry comments and formatting that a round-trip would destroy, and those
comments are what the LLM reads in the seed program.

``(blocked ?l)`` is the load-bearing fact. ``factory_jackal_domain.pddl`` has
``(not (blocked ?to))`` in ``move``'s precondition, so asserting it is what makes
a planner route around a region instead of re-proposing the corridor that just
failed.
"""

from __future__ import annotations

import re

__all__ = ["find_block", "splice_init", "set_robot_location"]


def find_block(text: str, keyword: str) -> tuple[int, int]:
    """Return ``(start, end)`` character offsets of a ``(:keyword ...)`` block.

    ``start`` indexes the opening paren, ``end`` the matching close paren.
    Paren counting skips parens inside ``;;`` comments, which the mission files
    use heavily -- a naive count lands in the wrong place.
    """
    match = re.search(rf"\(\s*:{keyword}\b", text)
    if match is None:
        raise ValueError(f"no (:{keyword} ...) block found")
    start = match.start()
    depth = 0
    in_comment = False
    for i in range(start, len(text)):
        ch = text[i]
        if in_comment:
            if ch == "\n":
                in_comment = False
            continue
        if ch == ";":
            in_comment = True
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return start, i
    raise ValueError(f"unbalanced parens in (:{keyword} ...) block")


def splice_init(problem_text: str, facts: list[str]) -> str:
    """Append ``facts`` just inside the problem's ``(:init ...)`` close paren."""
    if not facts:
        return problem_text
    _, end = find_block(problem_text, "init")
    added = "".join(f"    {f}\n" for f in facts)
    return problem_text[:end].rstrip() + "\n" + added + "  " + problem_text[end:]


def set_goal(problem_text: str, goal_body: str) -> str:
    """Replace the problem's ``(:goal ...)`` with ``goal_body``.

    The curated missions carry delivery/inspection goals, but the executor can
    only drive: ``plan_to_nav2_goals`` lowers ``move`` actions and nothing else,
    and ``scand_metrics`` scores progress against the waypoint route. So an
    online replan has to solve the *navigation* problem the robot is actually
    running, or it comes back with a valid plan that ends at the wrong place and
    gets thrown away.
    """
    start, end = find_block(problem_text, "goal")
    return problem_text[:start] + f"(:goal\n    {goal_body}\n  )" + problem_text[end + 1:]


def set_robot_location(problem_text: str, robot: str, region: str) -> str:
    """Rewrite ``(at <robot> <anything>)`` in ``:init`` to the robot's live region.

    Only the ``:init`` block is touched -- an ``(at ...)`` inside ``:goal`` is a
    mission requirement and must survive untouched.
    """
    start, end = find_block(problem_text, "init")
    init = problem_text[start:end]
    pattern = re.compile(rf"\(\s*at\s+{re.escape(robot)}\s+[^\s)]+\s*\)")
    if not pattern.search(init):
        raise ValueError(f"no (at {robot} ...) fact in (:init ...)")
    return problem_text[:start] + pattern.sub(f"(at {robot} {region})", init, count=1) + problem_text[end:]
