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

__all__ = ["find_block", "splice_init", "set_goal", "set_robot_location",
           "remove_init_fact"]


def _mask_comments(text: str) -> str:
    """``text`` with ``;`` comments blanked to spaces, offsets preserved.

    Every scan below runs on this rather than on the raw text, so a comment can
    never be mistaken for code while character positions still index the
    original string.
    """
    return re.sub(r";[^\n]*", lambda m: " " * len(m.group(0)), text)


def find_block(text: str, keyword: str) -> tuple[int, int]:
    """Return ``(start, end)`` character offsets of a ``(:keyword ...)`` block.

    ``start`` indexes the opening paren, ``end`` the matching close paren.

    Both the search for the block and the paren counting ignore ``;`` comments,
    which the mission files use heavily. Counting always did; SEARCHING did
    not, and that asymmetry was a live trap. A header comment in
    factory_tour_03.pddl that mentioned "(:init ...)" in prose was matched as
    the block itself, so paren counting started inside the comment and every
    splice thereafter failed with "no (at jackal_1 ...) fact in (:init ...)" --
    a mission file made unusable by its own documentation. These files are
    written to be read (they are the LLM's seed), so prose naming a block is
    expected and must be inert.
    """
    scan = _mask_comments(text)
    match = re.search(rf"\(\s*:{keyword}\b", scan)
    if match is None:
        raise ValueError(f"no (:{keyword} ...) block found")
    start = match.start()
    depth = 0
    for i in range(start, len(scan)):
        ch = scan[i]
        if ch == "(":
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


def remove_init_fact(problem_text: str, predicate: str, *args: str) -> str:
    """Delete ``(predicate arg...)`` from ``:init``, if present.

    Every other splice here is additive, because runtime facts are things
    execution *revealed*. Finding an object is the one case that also
    invalidates a fact the mission asserted: ``(location-unknown cone)`` is
    ``approach``'s negative precondition, so leaving it in place would make the
    approach unplannable no matter what else is spliced in. Matching is
    case-insensitive -- PDDL is, and the missions author regions as ``R5``
    while runtime facts arrive lowercased.

    A trailing same-line comment goes with the fact. Otherwise removing
    ``(location-unknown cone)   ;; retracted on discovery`` leaves the comment
    behind as a line describing a fact that is no longer there -- and this text
    is not merely a diagnostic, it is the seed problem EvoPlan hands the LLM,
    so a stranded annotation contradicting the state below it is bad input, not
    just untidy output. A comment on its own line is NOT touched: it may belong
    to the facts that follow, and guessing is how a splice eats something
    load-bearing. Mission files put their prose in the header for that reason.

    Only ``:init`` is touched; the same fact appearing in ``:goal`` would be a
    mission requirement.
    """
    start, end = find_block(problem_text, "init")
    init = problem_text[start:end]
    body = r"\s+".join([re.escape(predicate)] + [re.escape(a) for a in args])
    pattern = re.compile(rf"[ \t]*\(\s*{body}\s*\)[ \t]*(?:;[^\n]*)?\n?",
                         re.IGNORECASE)
    return problem_text[:start] + pattern.sub("", init) + problem_text[end:]


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
