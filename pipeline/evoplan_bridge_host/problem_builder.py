#!/usr/bin/env python3
"""Build a PDDL problem describing the robot's live situation.

Starts from a curated mission problem in
``evolve_stl_pddl/jackal/in/factory_mission_NN.pddl`` rather than synthesising
one, because those files carry the real mission goals (``delivered``,
``inspected``), are already VAL-clean, and their ``(:objects ...)`` block agrees
with ``graph.json``. Onto that base it splices what execution revealed:

* where the robot actually is now;
* which regions turned out to be impassable, as ``(blocked ?l)``;
* which regions have already been visited, so a replan does not redo work.
"""

from __future__ import annotations

import re
from pathlib import Path

from pddl_splice import find_block, set_goal, set_robot_location, splice_init

#: Goal predicates the executor can actually discharge. `move`'s effect adds
#: BOTH (at ?r ?to) and (visited ?to), and `move` is the only action with an
#: executor -- so a goal built solely from these is directly achievable and
#: must NOT be narrowed. No mission authors an (at ...) goal today; `at` is
#: listed because it is the predicate narrowing now produces, so a narrowed
#: problem fed back through here is recognised as already executable instead of
#: being narrowed a second time.
EXECUTABLE_GOAL_PREDICATES = frozenset({"visited", "at"})

__all__ = ["MissionLibrary", "build_runtime_problem", "regions_in_problem",
           "goal_is_executable", "EXECUTABLE_GOAL_PREDICATES"]


def goal_is_executable(problem_text: str) -> bool:
    """True when every predicate in ``(:goal ...)`` is one `move` can achieve.

    This is what makes goal narrowing phase-aware. Narrowing to
    ``(visited <target>)`` exists because a delivery goal
    (``delivered``/``inspected``) produces plans whose final action is a dropoff
    the executor cannot perform -- so the robot would end somewhere the metrics
    do not treat as the endpoint.

    A region-coverage tour is the opposite case: its goal is nothing but
    ``(visited ...)``, every action in a solution is a `move`, and narrowing
    would throw away thirteen of fourteen regions and abandon the tour. Deciding
    from the goal's own predicates means no caller has to remember a flag.
    """
    try:
        start, end = find_block(problem_text, "goal")
    except ValueError:
        return False
    body = problem_text[start:end]
    preds = set(re.findall(r"\(([a-z][a-z0-9-]*)\s", body)) - {"and", "or", "not", "exists", "forall"}
    return bool(preds) and preds <= EXECUTABLE_GOAL_PREDICATES


def regions_in_problem(problem_text: str) -> set[str]:
    """Location names declared in the problem's ``(:objects ...)`` block."""
    start, end = find_block(problem_text, "objects")
    body = problem_text[start:end]
    regions: set[str] = set()
    # Objects are declared as `name1 name2 - type`; keep the location types.
    for chunk in re.finditer(r"([^-()]+)-\s*([A-Za-z_][\w-]*)", body):
        names, type_name = chunk.group(1), chunk.group(2).lower()
        if type_name in ("location", "aisle", "shelf_zone", "loading_zone", "charging_zone"):
            regions.update(n.lower() for n in names.split() if not n.startswith(":"))
    return regions


def build_runtime_problem(
    base_problem_text: str,
    robot: str,
    current_region: str | None,
    blocked_regions=(),
    visited_regions=(),
    target_region: str | None = None,
    preserve_goal: bool | None = None,
) -> str:
    """Return the mission problem updated with runtime facts.

    When ``target_region`` is given the goal is narrowed to
    ``(at <robot> R)``. That is deliberate: only ``move`` actions have an
    executor, so a full delivery goal would produce a plan that ends wherever
    the last dropoff is rather than at the region the executor and the metrics
    both treat as the mission endpoint.

    The narrowed goal is ``(at ...)`` and NOT ``(visited ...)`` because
    ``visited`` is monotone -- ``move``'s effect adds it and nothing ever
    retracts it -- so once a region has been entered, ``(visited R)`` is
    permanently true. The find-object approach runs precisely after a coverage
    tour has entered every region, so the narrowed goal was already satisfied
    in ``:init``: Fast Downward returned "Solution found, cost = 0" with zero
    actions, the executor rejected the empty plan, and the robot never drove to
    the object it had just located. ``(at <robot> R)`` is achievable by the
    same ``move`` action -- whose effect asserts both ``(at ?r ?to)`` and
    ``(visited ?to)`` -- but is false whenever the robot is somewhere else,
    which is exactly the condition worth planning for. It is also the stronger
    and more faithful statement of the intent above: END there, rather than
    merely have passed through at some point.
    """
    text = base_problem_text
    # preserve_goal=None means "decide from the goal itself"; an explicit
    # True/False overrides for callers that know better.
    if preserve_goal is None:
        preserve_goal = goal_is_executable(base_problem_text)
    if target_region and not preserve_goal:
        text = set_goal(text, f"(at {robot.lower()} {target_region.lower()})")
    if current_region:
        text = set_robot_location(text, robot, current_region.lower())

    facts = []
    declared = regions_in_problem(base_problem_text)
    for region in sorted({r.lower() for r in blocked_regions}):
        # Asserting a fact about an undeclared object makes VAL reject the whole
        # problem, which would look like "the planner failed" rather than
        # "a region name did not match".
        if region in declared:
            facts.append(f"(blocked {region})")
    for region in sorted({r.lower() for r in visited_regions}):
        if region in declared:
            facts.append(f"(visited {region})")
    return splice_init(text, facts)


class MissionLibrary:
    """Loads and caches mission problems and the domain, once at service boot."""

    def __init__(self, jackal_in_dir: Path, domain_name: str = "factory_jackal_domain.pddl",
                 extra_dirs=()):
        self.dir = Path(jackal_in_dir)
        # Missions authored in autonomy_stack (e.g. the region-coverage tour)
        # live outside the submodule, which is kept pristine.
        self.extra_dirs = [Path(d) for d in extra_dirs]
        self.domain_path = self.dir / domain_name
        if not self.domain_path.is_file():
            raise FileNotFoundError(f"domain not found: {self.domain_path}")
        self.domain_text = self.domain_path.read_text()
        self._problems: dict[str, str] = {}

    def problem_text(self, mission_id: str) -> str:
        if mission_id not in self._problems:
            path = self.dir / f"{mission_id}.pddl"
            if not path.is_file():
                for extra in self.extra_dirs:
                    candidate = extra / f"{mission_id}.pddl"
                    if candidate.is_file():
                        path = candidate
                        break
            if not path.is_file():
                searched = [str(self.dir)] + [str(d) for d in self.extra_dirs]
                raise FileNotFoundError(
                    f"unknown mission '{mission_id}'; searched {searched}")
            self._problems[mission_id] = path.read_text()
        return self._problems[mission_id]

    def check_against_graph(self, graph_regions) -> list[str]:
        """Compare declared mission regions with the executor's ``graph.json``.

        Returns human-readable warnings. A silent divergence here is nasty: the
        planner happily returns a route through a region Nav2 has no waypoint
        for, and the failure surfaces much later as an unexplained abort.
        """
        warnings = []
        graph = {r.lower() for r in graph_regions}
        for path in sorted(self.dir.glob("factory_mission_*.pddl")):
            declared = regions_in_problem(path.read_text())
            # charge_dock is a PDDL-only location with no graph waypoint.
            missing = declared - graph - {"charge_dock"}
            if missing:
                warnings.append(
                    f"{path.name}: regions absent from graph.json: {sorted(missing)}"
                )
        return warnings
