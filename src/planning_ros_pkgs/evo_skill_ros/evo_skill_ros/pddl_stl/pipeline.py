#!/usr/bin/env python3

import argparse
import itertools
import json
import math
import os
import random
import re
from dataclasses import dataclass, field
from pathlib import Path


Fact = tuple[str, ...]


@dataclass
class ActionSchema:
    name: str
    parameters: list[str]
    parameter_types: dict[str, str]
    positive_preconditions: list[Fact] = field(default_factory=list)
    negative_preconditions: list[Fact] = field(default_factory=list)
    add_effects: list[Fact] = field(default_factory=list)
    del_effects: list[Fact] = field(default_factory=list)


@dataclass
class DomainModel:
    actions: dict[str, ActionSchema]
    type_parents: dict[str, str] = field(default_factory=dict)


@dataclass
class ProblemState:
    objects: dict[str, str]
    facts: set[Fact]
    goals: set[Fact]


@dataclass
class GroundAction:
    name: str
    args: tuple[str, ...]

    def text(self) -> str:
        return "(" + " ".join((self.name, *self.args)) + ")"


@dataclass
class ValidationResult:
    ok: bool
    final_facts: set[Fact]
    accepted_prefix_len: int
    errors: list[str] = field(default_factory=list)


def _strip_comments(text):
    return re.sub(r";.*", "", text)


def _parse_sexp(text):
    tokens = re.findall(r"\(|\)|[^\s()]+", _strip_comments(text).lower())
    pos = 0

    def parse_node():
        nonlocal pos
        if pos >= len(tokens):
            raise ValueError("Unexpected end of PDDL")
        token = tokens[pos]
        pos += 1
        if token != "(":
            return token
        node = []
        while pos < len(tokens) and tokens[pos] != ")":
            node.append(parse_node())
        if pos >= len(tokens):
            raise ValueError("Unclosed PDDL list")
        pos += 1
        return node

    parsed = parse_node()
    if pos != len(tokens):
        raise ValueError("Trailing tokens after PDDL document")
    return parsed


def _items_after(tree, tag):
    return [item for item in tree if isinstance(item, list) and item and item[0] == tag]


def _typed_names(items):
    names = {}
    pending = []
    idx = 0
    while idx < len(items):
        token = items[idx]
        if token == "-":
            typ = items[idx + 1]
            for name in pending:
                names[name] = typ
            pending = []
            idx += 2
        else:
            pending.append(token)
            idx += 1
    for name in pending:
        names[name] = "object"
    return names


def _unwrap_temporal(expr):
    if (
        isinstance(expr, list)
        and len(expr) >= 3
        and ((expr[0] == "at" and expr[1] in {"start", "end"}) or (expr[0] == "over" and expr[1] == "all"))
    ):
        return expr[-1]
    return expr


def _collect_facts(expr, positives, negatives):
    expr = _unwrap_temporal(expr)
    if not isinstance(expr, list) or not expr:
        return
    head = expr[0]
    if head == "and":
        for part in expr[1:]:
            _collect_facts(part, positives, negatives)
    elif head == "not":
        neg = _unwrap_temporal(expr[1])
        if isinstance(neg, list) and neg:
            negatives.append(tuple(neg))
    elif head in {"decrease", "assign", "increase", "="}:
        return
    else:
        positives.append(tuple(expr))


def _parameter_types(param_expr):
    typed = _typed_names(param_expr)
    return {name: typ for name, typ in typed.items() if name.startswith("?")}


def parse_domain(path):
    tree = _parse_sexp(Path(path).read_text())
    actions = {}
    type_parents = {}
    for item in _items_after(tree, ":types"):
        typed = _typed_names(item[1:])
        for typ, parent in typed.items():
            type_parents[typ] = parent
    for item in tree:
        if not isinstance(item, list) or not item:
            continue
        if item[0] not in {":action", ":durative-action"}:
            continue
        name = item[1]
        fields = {item[i]: item[i + 1] for i in range(2, len(item) - 1, 2) if isinstance(item[i], str)}
        parameter_types = _parameter_types(fields.get(":parameters", []))
        schema = ActionSchema(name=name, parameters=list(parameter_types), parameter_types=parameter_types)
        _collect_facts(fields.get(":precondition", fields.get(":condition", [])), schema.positive_preconditions, schema.negative_preconditions)
        _collect_facts(fields.get(":effect", []), schema.add_effects, schema.del_effects)
        actions[name] = schema
    return DomainModel(actions=actions, type_parents=type_parents)


def parse_problem(path):
    tree = _parse_sexp(Path(path).read_text())
    objects = {}
    facts = set()
    goals = set()
    for item in tree:
        if not isinstance(item, list) or not item:
            continue
        if item[0] == ":objects":
            objects.update(_typed_names(item[1:]))
        elif item[0] == ":init":
            for fact in item[1:]:
                if isinstance(fact, list) and fact and fact[0] != "=":
                    facts.add(tuple(fact))
        elif item[0] == ":goal":
            positives, negatives = [], []
            _collect_facts(item[1], positives, negatives)
            goals.update(positives)
    return ProblemState(objects=objects, facts=facts, goals=goals)


def parse_plan(path):
    actions = []
    for line in Path(path).read_text().splitlines():
        line = line.strip().lower()
        if not line or line.startswith(";"):
            continue
        match = re.search(r"\(([^)]+)\)", line)
        if not match:
            raise ValueError(f"Plan line is not an action: {line}")
        parts = match.group(1).split()
        actions.append(GroundAction(parts[0], tuple(parts[1:])))
    return actions


def _ground(fact, binding):
    return tuple(binding.get(token, token) for token in fact)


def validate_plan(domain, problem, plan, require_goal=True):
    facts = set(problem.facts)
    errors = []
    for idx, ground_action in enumerate(plan):
        schema = domain.actions.get(ground_action.name)
        if schema is None:
            return ValidationResult(False, facts, idx, [f"{ground_action.text()}: unknown action"])
        if len(ground_action.args) != len(schema.parameters):
            return ValidationResult(False, facts, idx, [f"{ground_action.text()}: wrong arity"])
        binding = dict(zip(schema.parameters, ground_action.args))
        missing = [_ground(f, binding) for f in schema.positive_preconditions if _ground(f, binding) not in facts]
        forbidden = [_ground(f, binding) for f in schema.negative_preconditions if _ground(f, binding) in facts]
        if missing or forbidden:
            if missing:
                errors.append(f"{ground_action.text()}: missing preconditions {missing}")
            if forbidden:
                errors.append(f"{ground_action.text()}: negative preconditions violated {forbidden}")
            return ValidationResult(False, facts, idx, errors)
        for fact in schema.del_effects:
            facts.discard(_ground(fact, binding))
        for fact in schema.add_effects:
            facts.add(_ground(fact, binding))
    unmet = sorted(problem.goals - facts)
    if require_goal and unmet:
        return ValidationResult(False, facts, len(plan), [f"unmet goals {unmet}"])
    return ValidationResult(True, facts, len(plan), errors)


def load_world(path):
    with open(path, "r") as stream:
        world = json.load(stream)
    regions = {region["name"].lower(): tuple(region["coords"]) for region in world.get("regions", [])}
    objects = {obj["name"].lower(): tuple(obj["coords"]) for obj in world.get("objects", [])}
    return world, regions, objects


def _objects_by_type(problem):
    by_type = {}
    for name, typ in problem.objects.items():
        by_type.setdefault(typ, []).append(name)
    return by_type


def _is_type_compatible(actual, expected, type_parents):
    typ = actual
    seen = set()
    while typ:
        if typ in seen:
            break
        seen.add(typ)
        if typ == expected:
            return True
        typ = type_parents.get(typ)
    return expected in {"object", None}


def _candidate_args(schema, problem, domain):
    pools = []
    for param in schema.parameters:
        expected = schema.parameter_types.get(param, "object")
        typed = [
            name for name, typ in problem.objects.items()
            if _is_type_compatible(typ, expected, domain.type_parents)
        ]
        pools.append(typed or list(problem.objects.keys()))
    for args in itertools.product(*pools):
        yield tuple(args)


class EvolutionaryPlanEditor:
    def __init__(self, domain, problem, rng=None):
        self.domain = domain
        self.problem = problem
        self.rng = rng or random.Random(7)

    def propose(self, seed_plan, population_size):
        proposals = [list(seed_plan)]
        action_names = list(self.domain.actions)
        for _ in range(max(0, population_size - 1)):
            candidate = list(seed_plan)
            op = self.rng.choice(["insert", "delete", "swap"])
            if op == "delete" and candidate:
                del candidate[self.rng.randrange(len(candidate))]
            else:
                schema = self.domain.actions[self.rng.choice(action_names)]
                args = self.rng.choice(list(_candidate_args(schema, self.problem, self.domain)))
                ground = GroundAction(schema.name, args)
                if op == "swap" and candidate:
                    candidate[self.rng.randrange(len(candidate))] = ground
                else:
                    candidate.insert(self.rng.randrange(len(candidate) + 1), ground)
            proposals.append(candidate)
        proposals.extend(self._goal_directed_repairs(seed_plan))
        return proposals

    def _goal_directed_repairs(self, seed_plan):
        repairs = []
        repairs.extend(self._inspection_repairs(seed_plan))
        repairs.extend(self._visited_repairs(seed_plan))
        for goal in self.problem.goals:
            for schema in self.domain.actions.values():
                for effect in schema.add_effects:
                    if effect[0] != goal[0] or len(effect) != len(goal):
                        continue
                    binding = {}
                    for lhs, rhs in zip(effect, goal):
                        if lhs.startswith("?"):
                            binding[lhs] = rhs
                    args = tuple(binding.get(param, next(iter(self.problem.objects), "")) for param in schema.parameters)
                    repairs.append(list(seed_plan) + [GroundAction(schema.name, args)])
        return repairs

    def _visited_repairs(self, seed_plan):
        repairs = []
        robot = next((name for name, typ in self.problem.objects.items() if typ == "robot"), None)
        if robot is None:
            return repairs
        at_facts = [fact for fact in self.problem.facts if fact[:2] == ("at", robot)]
        start = at_facts[0][2] if at_facts else None
        graph = {}
        for fact in self.problem.facts:
            if len(fact) == 3 and fact[0] == "connected":
                graph.setdefault(fact[1], []).append(fact[2])
        for goal in self.problem.goals:
            if len(goal) != 2 or goal[0] != "visited" or start is None:
                continue
            target = goal[1]
            path = self._shortest_path(graph, start, target)
            if not path:
                continue
            plan = list(seed_plan)
            for src, dst in zip(path, path[1:]):
                plan.append(GroundAction("move", (robot, src, dst)))
            repairs.append(plan)
        return repairs

    def _inspection_repairs(self, seed_plan):
        repairs = []
        robot = next((name for name, typ in self.problem.objects.items() if typ == "robot"), None)
        if robot is None:
            return repairs
        at_facts = [fact for fact in self.problem.facts if fact[:2] == ("at", robot)]
        start = at_facts[0][2] if at_facts else None
        graph = {}
        for fact in self.problem.facts:
            if len(fact) == 3 and fact[0] == "connected":
                graph.setdefault(fact[1], []).append(fact[2])
        for goal in self.problem.goals:
            if len(goal) != 2 or goal[0] != "inspected" or start is None:
                continue
            target = goal[1]
            path = self._shortest_path(graph, start, target)
            if not path:
                continue
            plan = list(seed_plan)
            for src, dst in zip(path, path[1:]):
                plan.append(GroundAction("move", (robot, src, dst)))
            plan.append(GroundAction("inspect-shelf", (robot, target)))
            repairs.append(plan)
        return repairs

    @staticmethod
    def _shortest_path(graph, start, target):
        queue = [(start, [start])]
        seen = {start}
        while queue:
            node, path = queue.pop(0)
            if node == target:
                return path
            for nxt in graph.get(node, []):
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append((nxt, path + [nxt]))
        return None


class SkillLibrary:
    def __init__(self, path):
        self.path = path
        self.data = json.loads(Path(path).read_text())

    def contract_for(self, action_name):
        return self.data.get("skills", {}).get(action_name, {})


def _nearest_object_to_segment(objects, start, goal):
    if not objects:
        return None
    ax, ay = start
    bx, by = goal
    best = None
    best_dist = float("inf")
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    for name, (px, py) in objects.items():
        t = 0.0 if denom == 0 else max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / denom))
        cx, cy = ax + t * abx, ay + t * aby
        dist = math.hypot(px - cx, py - cy)
        if dist < best_dist:
            best = (name, (px, py), dist)
            best_dist = dist
    return best


def _solve_navigation_trace(start_xy, goal_xy, cone_xy, horizon, dt):
    max_speed = 1.0
    segment_distance = math.hypot(goal_xy[0] - start_xy[0], goal_xy[1] - start_xy[1])
    min_reachable_horizon = math.ceil(segment_distance / (max_speed * dt)) + 10
    solve_horizon = max(horizon, min_reachable_horizon)
    try:
        import numpy as np

        from evo_skill_ros.utility.stl_solver import compute_stl_trajectory

        heading = math.atan2(goal_xy[1] - start_xy[1], goal_xy[0] - start_xy[0])
        x0 = np.array([start_xy[0], start_xy[1], heading])
        x_roll = compute_stl_trajectory(x0, goal_xy, cone_xy, T=solve_horizon, dt=dt)
        status = "drake_stl"
        if solve_horizon != horizon:
            status += f": horizon_auto_extended {horizon}->{solve_horizon}"
        return x_roll.T.tolist(), status
    except Exception as exc:
        trace = []
        steps = max(1, solve_horizon)
        for idx in range(steps + 1):
            alpha = idx / steps
            x = (1.0 - alpha) * start_xy[0] + alpha * goal_xy[0]
            y = (1.0 - alpha) * start_xy[1] + alpha * goal_xy[1]
            trace.append([x, y, 0.0])
        status = f"fallback_linear_trace: {exc}"
        if solve_horizon != horizon:
            status += f"; horizon_auto_extended {horizon}->{solve_horizon}"
        return trace, status


def _trace_navigation_metrics(trace, goal_xy, obstacle_xy, goal_tolerance=0.25, safe_distance=0.60):
    if not trace:
        return {}
    final_state = trace[-1]
    final_goal_distance = math.hypot(final_state[0] - goal_xy[0], final_state[1] - goal_xy[1])
    obstacle_distances = [
        math.hypot(state[0] - obstacle_xy[0], state[1] - obstacle_xy[1])
        for state in trace
    ]
    return {
        "final_goal_distance": final_goal_distance,
        "min_obstacle_distance": min(obstacle_distances),
        "goal_tolerance": goal_tolerance,
        "safe_distance": safe_distance,
        "geometric_stl_satisfied": (
            final_goal_distance <= goal_tolerance
            and min(obstacle_distances) >= safe_distance
        ),
    }


def solve_symbolic_prefix(plan, validation, skill_library, regions, objects, horizon, dt):
    traces = []
    effective_regions = dict(regions)
    missing_locations = []
    for action in plan[: validation.accepted_prefix_len]:
        location_args = action.args[-2:] if action.name.startswith("move") and len(action.args) >= 3 else action.args[-1:]
        for arg in location_args:
            if arg not in effective_regions and arg not in missing_locations:
                missing_locations.append(arg)
    for idx, location in enumerate(missing_locations):
        effective_regions[location] = (float(idx * 2), 0.0)

    current_xy = None
    for fact in validation.final_facts:
        if fact[:2] == ("at", "jackal1") and fact[2] in effective_regions:
            current_xy = effective_regions[fact[2]]
    if current_xy is None:
        at_facts = [fact for fact in validation.final_facts if fact and fact[0] == "at" and fact[-1] in effective_regions]
        current_xy = effective_regions[at_facts[0][-1]] if at_facts else (0.0, 0.0)

    for action in plan[: validation.accepted_prefix_len]:
        contract = skill_library.contract_for(action.name)
        if contract.get("solver") != "drake_stl_unicycle":
            traces.append({"action": action.text(), "contract": contract, "trace": []})
            continue
        if len(action.args) >= 3 and action.name.startswith("move"):
            start_location = action.args[-2]
            goal_location = action.args[-1]
            start_xy = effective_regions[start_location]
        else:
            goal_location = action.args[-1]
            start_xy = current_xy
        goal_location = action.args[-1]
        if goal_location not in effective_regions:
            traces.append({"action": action.text(), "contract": contract, "trace": []})
            continue
        goal_xy = effective_regions[goal_location]
        nearest = _nearest_object_to_segment(objects, start_xy, goal_xy)
        cone_xy = nearest[1] if nearest else (999.0, 999.0)
        trace, solver_status = _solve_navigation_trace(start_xy, goal_xy, cone_xy, horizon, dt)
        metrics = _trace_navigation_metrics(trace, goal_xy, cone_xy)
        traces.append({
            "action": action.text(),
            "contract": contract,
            "obstacle": nearest[0] if nearest else None,
            "solver_status": solver_status,
            "metrics": metrics,
            "trace": trace,
        })
        current_xy = goal_xy
    return traces


def dispatch_receding_horizon(traces, prefix_steps):
    executed = []
    certificates = []
    for item in traces:
        trace = item.get("trace", [])
        safe_prefix = trace[:prefix_steps] if trace else []
        solver_status = item.get("solver_status", "symbolic_only")
        symbolic_only = item.get("contract", {}).get("symbolic_only", False)
        stl_certified = bool(safe_prefix) and str(solver_status).startswith("drake_stl")
        cert = {
            "action": item["action"],
            "safe_prefix_len": len(safe_prefix),
            "effect_certificate": stl_certified or symbolic_only,
            "min_robustness": 0.0 if stl_certified else None,
            "solver_status": solver_status,
        }
        executed.append({"action": item["action"], "solver_status": cert["solver_status"], "states": safe_prefix})
        certificates.append(cert)
    return executed, certificates


def apply_certified_effects(domain, problem, plan, certificates):
    facts = set(problem.facts)
    for action, certificate in zip(plan, certificates):
        if not certificate.get("effect_certificate"):
            break
        schema = domain.actions[action.name]
        binding = dict(zip(schema.parameters, action.args))
        for fact in schema.del_effects:
            facts.discard(_ground(fact, binding))
        for fact in schema.add_effects:
            facts.add(_ground(fact, binding))
    return facts


def run_pipeline(args):
    domain = parse_domain(args.domain)
    problem = parse_problem(args.problem)
    world, regions, objects = load_world(args.world)
    skill_library = SkillLibrary(args.skill_library)
    seed_plan = parse_plan(args.seed_plan) if args.seed_plan else []
    editor = EvolutionaryPlanEditor(domain, problem)

    best = None
    history = []
    for generation in range(args.generations):
        for candidate in editor.propose(seed_plan, args.population):
            result = validate_plan(domain, problem, candidate, require_goal=True)
            history.append({
                "generation": generation,
                "plan": [action.text() for action in candidate],
                "ok": result.ok,
                "accepted_prefix_len": result.accepted_prefix_len,
                "errors": result.errors,
            })
            if result.ok:
                best = (candidate, result)
                break
        if best:
            break

    if best is None:
        return {"ok": False, "history": history, "error": "No validated plan found"}

    plan, validation = best
    prefix_validation = validate_plan(domain, problem, plan[: args.symbolic_prefix], require_goal=False)
    traces = solve_symbolic_prefix(plan, prefix_validation, skill_library, regions, objects, args.horizon, args.dt)
    executed, certificates = dispatch_receding_horizon(traces, args.dispatch_steps)
    certified_facts = apply_certified_effects(domain, problem, plan[: prefix_validation.accepted_prefix_len], certificates)
    asserted_facts = sorted([list(fact) for fact in certified_facts])
    return {
        "ok": True,
        "task": args.task,
        "plan": [action.text() for action in plan],
        "history": history,
        "validated_symbolic_prefix": [action.text() for action in plan[: prefix_validation.accepted_prefix_len]],
        "executed_prefixes": executed,
        "certificates": certificates,
        "asserted_facts_after_certificates": asserted_facts,
        "world_robot_location": world.get("robot_location"),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="LLM-guided evolutionary PDDL-STL planning pipeline")
    parser.add_argument("--task", default="inspect shelfA")
    parser.add_argument("--domain", default="config/factory_sim_domain.pddl")
    parser.add_argument("--problem", default="config/factory_problem.pddl")
    parser.add_argument("--world", default="config/graph.json")
    parser.add_argument("--skill-library", default="config/pddl_stl_skill_library.json")
    parser.add_argument("--seed-plan")
    parser.add_argument("--generations", type=int, default=8)
    parser.add_argument("--population", type=int, default=12)
    parser.add_argument("--symbolic-prefix", type=int, default=2)
    parser.add_argument("--horizon", type=int, default=60)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--dispatch-steps", type=int, default=10)
    parser.add_argument("--output", default="pddl_stl_execution_report.json")
    args = parser.parse_args(argv)

    report = run_pipeline(args)
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps({"ok": report["ok"], "output": args.output, "plan": report.get("plan", [])}, indent=2))
    if not report["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
