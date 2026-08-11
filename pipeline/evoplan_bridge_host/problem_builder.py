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

from pddl_splice import (find_block, goal_conjuncts, remove_init_fact, set_goal,
                         set_robot_location, splice_init, splice_objects)

#: Goal predicates a plan can reach without the executor having to perform
#: anything it cannot. `move`'s effect adds BOTH (at ?r ?to) and (visited ?to),
#: and `move` is the only action with an executor -- so a goal built solely
#: from these is directly achievable and must NOT be narrowed. No mission
#: authors an (at ...) goal today; `at` is listed because it is the predicate
#: narrowing produces, so a narrowed problem fed back through here is
#: recognised as already executable instead of being narrowed a second time.
#:
#: `reached` is here on a weaker but sufficient basis: nothing the executor
#: drives asserts it, but the only actions that DO -- `approach`, reachable
#: only by moving to the object's region, and `search-for` -- are pure
#: bookkeeping with no physical effect, and filter_executable_actions drops
#: both. So a plan achieving (reached ?r ?t) still drives the robot exactly
#: where the goal wants it. Omitting it was not an option: the find-object
#: tours now conjoin (reached jackal_1 cone) with their coverage goal, and a
#: goal deemed non-executable is NARROWED -- a mid-tour obstacle replan would
#: have thrown the entire tour away and replaced it with a single move.
#:
#: `inspected-object` is here on the strongest basis of the four: the executor
#: genuinely performs it. plan_to_nav2_goals lowers `inspect-object` to a Nav2
#: goal pose facing the object plus a stationary dwell, so a plan achieving
#: (inspected-object ?t) does exactly what the goal says. Leaving it out would
#: mean any obstacle replan during an inspection run narrowed the goal to a
#: single (at ...) and abandoned every object still uninspected.
EXECUTABLE_GOAL_PREDICATES = frozenset({"visited", "at", "reached",
                                        "inspected-object"})

#: Object types in factory_jackal_domain.pddl that denote a navigable region.
LOCATION_TYPES = frozenset({"location", "aisle", "shelf_zone", "loading_zone",
                            "charging_zone"})

#: The domain type of a findable mission object.
TARGET_TYPE = "target"

__all__ = ["MissionLibrary", "build_runtime_problem", "regions_in_problem",
           "objects_in_problem", "targets_in_problem", "resolve_target_object",
           "resolve_found_objects", "splice_inspection_targets",
           "goal_is_executable", "EXECUTABLE_GOAL_PREDICATES",
           "LOCATION_TYPES", "TARGET_TYPE"]


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


def objects_in_problem(problem_text: str) -> dict[str, str]:
    """Map every declared object name (lowercased) to its declared type.

    Comments are stripped first: the mission files carry prose containing
    hyphens and type words, and ``(:objects ...)`` is parsed by regex.
    """
    start, end = find_block(problem_text, "objects")
    body = re.sub(r";[^\n]*", "", problem_text[start:end])
    objects: dict[str, str] = {}
    # Objects are declared as `name1 name2 - type`.
    for chunk in re.finditer(r"([^-()]+)-\s*([A-Za-z_][\w-]*)", body):
        names, type_name = chunk.group(1), chunk.group(2).lower()
        for name in names.split():
            if not name.startswith(":"):
                objects[name.lower()] = type_name
    return objects


def regions_in_problem(problem_text: str) -> set[str]:
    """Location names declared in the problem's ``(:objects ...)`` block."""
    return {name for name, type_name in objects_in_problem(problem_text).items()
            if type_name in LOCATION_TYPES}


def targets_in_problem(problem_text: str) -> set[str]:
    """Findable objects (``- target``) declared in the problem."""
    return {name for name, type_name in objects_in_problem(problem_text).items()
            if type_name == TARGET_TYPE}


def _slug(text: str) -> str:
    """Detector class -> candidate PDDL symbol ("traffic cone" -> traffic_cone)."""
    return re.sub(r"[^a-z0-9_]+", "_", (text or "").lower()).strip("_")


def resolve_target_object(problem_text: str, object_class: str | None = None) -> str | None:
    """Which declared ``target`` the perception class refers to, if any.

    The executor knows the object as a detector class string -- "traffic cone",
    with a space -- which is not a legal PDDL symbol, and the mission author
    should not have to keep the two spellings in sync. So the name is resolved
    from the problem rather than transmitted:

    1. exact match on the slugified class (``traffic cone`` -> ``traffic_cone``),
       or on its last word (``cone``) -- what a mission would naturally call it;
    2. failing that, the sole declared target, since a find-an-object mission
       that declares exactly one has no ambiguity to resolve;
    3. otherwise ``None``, and the caller falls back to the old
       ``(at <robot> <region>)`` goal rather than guessing.
    """
    targets = targets_in_problem(problem_text)
    if not targets:
        return None
    slug = _slug(object_class)
    for candidate in (slug, slug.rsplit("_", 1)[-1] if slug else ""):
        if candidate and candidate in targets:
            return candidate
    return next(iter(targets)) if len(targets) == 1 else None


def resolve_found_objects(problem_text: str, found_object, declared) -> list[tuple[str, str]]:
    """``[(pddl_name, region), ...]`` for every usable sighting, name-sorted.

    ``found_object`` may be one sighting or several: a mission can hunt more
    than one object (factory_tour_02 wants both the cone and the skateboard),
    and they are found at different moments, so the caller accumulates them and
    hands over everything known so far. Rebuilding from the full set each time
    is what stops the second discovery erasing the first.

    A sighting is dropped when its region is not declared by the problem --
    asserting a fact about an undeclared object aborts Fast Downward's
    translator (exit 31) before search runs -- or when its detector class does
    not resolve to a declared ``target``. Dropping is right: the rest of the
    sightings still produce a solvable problem, where raising would lose them
    all.

    Sorted by name so the emitted problem is stable across calls; an artifact
    that reshuffles between identical inputs is one nobody can diff.
    """
    if not found_object:
        return []
    sightings = [found_object] if isinstance(found_object, dict) else list(found_object)
    by_name: dict[str, str] = {}
    for sighting in sightings:
        if not sighting:
            continue
        region = str(sighting.get("region") or "").strip().lower()
        if region not in declared:
            continue
        name = resolve_target_object(problem_text, sighting.get("class"))
        if name:
            by_name[name] = region      # last report of an object wins
    return sorted(by_name.items())


def splice_inspection_targets(problem_text: str, instances, declared=None) -> str:
    """Declare discovered objects, place them, and require each be inspected.

    This is the open-world half of the pipeline, and it inverts the usual
    direction of a splice. Everywhere else the runtime tells the problem
    something about objects the mission author already named. Here perception
    decides what the objects ARE: a survey mission declares none, because how
    many traffic cones are in the factory is precisely the question it was sent
    to answer, and each instance perception clusters becomes a new
    ``- target`` in ``(:objects ...)`` with its own ``(object-at ...)`` and its
    own ``(inspected-object ...)`` goal conjunct.

    ``instances`` are dicts with ``name`` (a PDDL-legal symbol, minted and kept
    stable by the executor's object registry -- the planner must see the same
    ``traffic_cone_2`` on every replan or it cannot tell an object it has
    already inspected from a new one), ``region``, and ``inspected``.

    ``inspected`` is what stops the robot re-doing finished work. The goal grows
    an ``(inspected-object ?t)`` conjunct per object and never loses one, so a
    problem that does not also assert the completed ones in ``:init`` is telling
    the planner those goals are still open -- and a correct planner then plans
    to satisfy them again. That is exactly what happened on a live run: a
    six-action plan that re-inspected two finished cones and drove back across
    the map to reach one of them, which then became the mission's target region
    and left it reporting incomplete. Handing the same problem to Fast Downward
    reproduced the plan, which is the proof it was never a planner defect;
    asserting the two facts collapsed the plan from five actions to one.

    The mission's own goal conjuncts SURVIVE. A find-and-inspect mission asks
    for two things at once -- cover the regions, inspect what you find -- and a
    replan issued while the survey is still unfinished must not silently drop
    the coverage half. Appending is also what makes this idempotent across the
    several replans one run performs: conjuncts are de-duplicated by text, so
    re-splicing the same instance changes nothing.

    An instance whose region the problem does not declare is dropped, for the
    reason every other splice here drops one: an undeclared name aborts Fast
    Downward's translator (exit 31) before search runs, losing the whole replan
    rather than one object.
    """
    usable = []
    seen = set()
    for instance in instances or ():
        name = str((instance or {}).get("name") or "").strip().lower()
        region = str((instance or {}).get("region") or "").strip().lower()
        if not name or not region or name in seen:
            continue
        if declared is not None and region not in declared:
            continue
        seen.add(name)
        usable.append((name, region, bool((instance or {}).get("inspected"))))
    if not usable:
        return problem_text
    usable.sort()

    text = problem_text
    already = objects_in_problem(text)
    text = splice_objects(text, [(name, TARGET_TYPE) for name, _, _ in usable
                                 if name not in already])
    regions = regions_in_problem(text)
    facts = []
    for name, region, inspected in usable:
        text = remove_init_fact(text, "location-unknown", name)
        # Idempotent: a re-splice after the object was re-localised must replace
        # its position, not assert a second one alongside the first.
        for known in sorted(regions | {region}):
            text = remove_init_fact(text, "object-at", name, known)
        # Retract before re-asserting for the same reason. Removing it
        # unconditionally also means the fact tracks the executor's registry
        # rather than accumulating: this splice runs on a base problem that may
        # already carry it from an earlier replan.
        text = remove_init_fact(text, "inspected-object", name)
        facts.append(f"(object-at {name} {region})")
        if inspected:
            facts.append(f"(inspected-object {name})")
    text = splice_init(text, facts)

    goals = list(goal_conjuncts(text))
    for name, _, _ in usable:
        conjunct = f"(inspected-object {name})"
        if conjunct not in goals:
            goals.append(conjunct)
    return set_goal(text, goals[0] if len(goals) == 1
                    else "(and " + " ".join(goals) + ")")


def build_runtime_problem(
    base_problem_text: str,
    robot: str,
    current_region: str | None,
    blocked_regions=(),
    visited_regions=(),
    target_region: str | None = None,
    preserve_goal: bool | None = None,
    found_object: dict | None = None,
    inspect_objects=(),
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

    ``found_object`` upgrades that narrowed goal when the mission declares what
    it is hunting. Given ``{"class": "traffic cone", "region": "r1"}`` and a
    problem declaring ``cone - target``, the object's discovered position
    becomes a *fact* -- ``(object-at cone r1)``, with ``(location-unknown
    cone)`` retracted -- and the goal becomes ``(reached jackal_1 cone)``. The
    planner then derives r1 for itself.

    ``inspect_objects`` is the open-world case and takes precedence over both:
    the mission declares no targets at all, and every object perception
    clustered arrives here as ``{"name": "traffic_cone_2", "region": "r3"}`` to
    be declared, placed and added to the goal. See
    :func:`splice_inspection_targets`.

    That is not cosmetic. With ``(at jackal_1 r1)`` the region was computed in
    Python and the PDDL was told only where to end up: the problem handed to
    the LLM contained no evidence that an object existed, so nothing it read
    could explain why r1. Stating the goal in the mission's own vocabulary is
    also what lets the object move -- a second sighting in a different region
    changes one fact, not the goal.
    """
    text = base_problem_text
    # preserve_goal=None means "decide from the goal itself"; an explicit
    # True/False overrides for callers that know better.
    if preserve_goal is None:
        preserve_goal = goal_is_executable(base_problem_text)
    declared = regions_in_problem(base_problem_text)

    resolved = resolve_found_objects(base_problem_text, found_object, declared)

    if inspect_objects:
        # Runs regardless of preserve_goal, and before the branches below --
        # it is the only path that ADDS to the goal rather than replacing it,
        # so "keep the authored goal" and "also inspect what was found" are not
        # in conflict. Handing the same problem back through here re-splices the
        # same conjuncts, which is a no-op by construction.
        text = splice_inspection_targets(text, inspect_objects, declared)
    elif resolved and not preserve_goal:
        facts = []
        for object_name, object_region in resolved:
            # Order matters only for readability: retract before asserting so
            # the two never coexist in the emitted text.
            text = remove_init_fact(text, "location-unknown", object_name)
            # Idempotent. The base may ALREADY carry an (object-at ...) -- the
            # approach replan is built on the problem /observation wrote when
            # perception first reported the object, and a second replan (or a
            # sighting that moved) would otherwise splice a contradictory
            # second position rather than replacing the first.
            for region in sorted(declared | {object_region}):
                text = remove_init_fact(text, "object-at", object_name, region)
            facts.append(f"(object-at {object_name} {object_region})")
        text = splice_init(text, facts)
        goals = " ".join(f"(reached {robot.lower()} {n})" for n, _ in resolved)
        text = set_goal(text, goals if len(resolved) == 1 else f"(and {goals})")
    elif target_region and not preserve_goal:
        text = set_goal(text, f"(at {robot.lower()} {target_region.lower()})")
    # Only assert a start location the problem actually declares. Unlike the
    # blocked/visited facts below, this was spliced unchecked -- and an
    # undeclared object is not a soft error: Fast Downward's TRANSLATOR aborts
    # (exit 31) before search ever runs, so the whole replan fails rather than
    # planning from a slightly wrong place. A live tour_02 run emitted
    # `(at jackal_1 r6)` into a problem declaring only R1-R5 (r6 came from a
    # drifted odom-frame pose); FD aborted, fd_first fell through to the LLM,
    # and the 45 s deadline expired with no plan while the robot sat at r5 with
    # the object already located. Dropping the fact leaves the problem's
    # authored start, which is wrong but solvable -- and the executor's
    # align_plan_start_with_current_region repairs the prefix. Missions
    # declaring every region (factory_tour_01) never hit this; tour_02's
    # 5-region subset is what exposed it.
    if current_region and current_region.lower() in declared:
        text = set_robot_location(text, robot, current_region.lower())

    facts = []
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
