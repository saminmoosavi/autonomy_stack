#!/usr/bin/env python3
"""State machine and pure helpers for the online symbolic replan (Tier 2).

Three tiers of response, escalating:

* **Tier 0** -- the Phi_mob shield (:mod:`evoplan_bridge.phi_mob_shield`),
  ~10 Hz, observes and reports.
* **Tier 1** -- the geometric monitor already in ``eveo_plan_deploy.py``:
  paint the offending human into the costmap, cancel, re-send the same
  waypoints. Milliseconds. Untouched by this module.
* **Tier 2** -- this: hold the robot, ask the host service for a *new symbolic
  plan* that routes around the region that failed, hot-swap it. Seconds to
  tens of seconds.

Tier 2 fires only when Tier 1 has proven insufficient, on three triggers: a
sustained shield violation, a Nav2 goal that aborted or got stuck, or the
Tier-1 replan budget running out.

The state machine itself::

    IDLE --escalate()--> HOLDING --result--> APPLYING --> IDLE
      ^                     |                              |
      +-- timeout/error/rejected --------------------------+

Everything here is free functions and a small dataclass, deliberately holding no
ROS types, so the splice arithmetic -- the part most likely to be subtly wrong --
is unit-testable without a running node.
"""

from __future__ import annotations

from dataclasses import dataclass, field

__all__ = [
    "IDLE",
    "HOLDING",
    "APPLYING",
    "SymbolicReplanState",
    "derive_blocked_regions",
    "build_reason_text",
    "filter_executable_actions",
    "plan_reaches_target",
    "align_plan_start",
]

IDLE = "idle"
HOLDING = "holding"
APPLYING = "applying"


@dataclass
class SymbolicReplanState:
    """Mission-lifetime bookkeeping for Tier-2 replans."""

    state: str = IDLE
    count: int = 0
    #: Regions proven impassable this mission. Accumulates so a second replan
    #: cannot re-propose the corridor the first one already failed on.
    blocked_regions: set = field(default_factory=set)
    #: Sim-time of the last escalation, for the cooldown.
    last_escalation_sim_s: float | None = None
    #: Total wall seconds spent deliberating, against the mission budget.
    deliberation_wall_s: float = 0.0
    #: Total sim seconds the robot was held.
    held_sim_s: float = 0.0
    #: Waypoint index to resume from if the replan does not pan out.
    resume_idx: int = 0
    #: Sim-time the current hold began.
    hold_started_sim_s: float | None = None

    def can_escalate(
        self,
        now_sim_s: float,
        max_replans: int,
        cooldown_s: float,
        deliberation_budget_s: float,
    ) -> tuple[bool, str]:
        """Guard check. Returns ``(allowed, reason_if_not)``."""
        if self.state != IDLE:
            return False, f"a symbolic replan is already {self.state}"
        if self.count >= max_replans:
            return False, f"symbolic replan budget exhausted ({self.count}/{max_replans})"
        if self.deliberation_wall_s >= deliberation_budget_s:
            return False, (
                f"mission deliberation budget exhausted "
                f"({self.deliberation_wall_s:.1f}s/{deliberation_budget_s:.1f}s)"
            )
        if self.last_escalation_sim_s is not None:
            age = now_sim_s - self.last_escalation_sim_s
            if age < cooldown_s:
                return False, f"symbolic replan cooldown ({age:.1f}s/{cooldown_s:.1f}s)"
        return True, ""


def derive_blocked_regions(
    trigger: str,
    current_region: str | None,
    target_region: str | None,
    obstacle_region: str | None = None,
    failed_region: str | None = None,
) -> list[str]:
    """Decide which regions to assert ``(blocked ?l)`` for.

    ``factory_jackal_domain.pddl`` puts ``(not (blocked ?to))`` in ``move``'s
    precondition, so this is the lever that makes a planner route around a
    region rather than re-proposing the corridor that just failed.

    Two regions are never blocked, because either would make the problem
    unsolvable and burn the whole deliberation budget producing nothing:

    * the region the robot is standing in -- there is no plan out of a blocked
      current position;
    * the goal region -- blocking it makes the mission goal unreachable by
      construction. When the trouble really is at the goal, the prose in
      ``reason.human_text`` still tells the LLM what happened.
    """
    candidate = None
    if trigger in ("stl_shield", "reactive_budget_exhausted"):
        candidate = obstacle_region
    elif trigger in ("nav2_goal_aborted", "nav2_stuck"):
        candidate = failed_region
    if candidate is None:
        return []

    candidate = candidate.lower()
    if current_region and candidate == current_region.lower():
        return []
    if target_region and candidate == target_region.lower():
        return []
    return [candidate]


def build_reason_text(trigger: str, detail: dict) -> str:
    """Compose the prose handed to the LLM as OpenEvolve feedback.

    This is the load-bearing difference between EvoPlan and an ordinary
    replanner. A ``(blocked r10)`` fact says *that* something failed; this says
    *why*, and OpenEvolve surfaces it in the next iteration's prompt via the
    evaluator's ``artifacts`` channel.
    """
    parts = []
    where = detail.get("from_region"), detail.get("to_region")
    leg = f"while moving {where[0]} -> {where[1]}" if all(where) else "during execution"

    if trigger == "stl_shield":
        conjunct = detail.get("violated_conjunct", "a social-compliance bound")
        rho = detail.get("robustness")
        rho_txt = f" (robustness {rho:.3f})" if isinstance(rho, (int, float)) else ""
        parts.append(
            f"The Phi_mob social-compliance shield fired {leg}: "
            f"the '{conjunct}' bound was violated{rho_txt}."
        )
    elif trigger == "nav2_goal_aborted":
        parts.append(f"Nav2 aborted the navigation goal {leg}; the route is not drivable.")
    elif trigger == "nav2_stuck":
        window = detail.get("stuck_window_s")
        parts.append(
            f"The robot made no progress {leg} for "
            f"{window:.0f} s and is stuck." if window else
            f"The robot made no progress {leg} and is stuck."
        )
    elif trigger == "reactive_budget_exhausted":
        attempts = detail.get("reactive_attempts", "several")
        parts.append(
            f"{attempts} reactive Nav2 replans {leg} all failed to find a "
            "socially acceptable route; local obstacle avoidance is not enough."
        )
    else:
        parts.append(f"Execution failed {leg} (trigger: {trigger}).")

    obstacle = detail.get("closest_obstacle")
    if obstacle:
        parts.append(f"The obstruction was tracked as '{obstacle}'.")

    blocked = detail.get("blocked_regions") or []
    if blocked:
        parts.append(
            "Regions now asserted as blocked and unusable for (move ...): "
            + ", ".join(sorted(blocked))
            + ". Produce a plan that reaches the goal without entering them."
        )
    else:
        parts.append(
            "No region could be marked blocked (the obstruction is at the "
            "robot's current position or at the goal), so find an alternative "
            "route or ordering instead."
        )
    return " ".join(parts)


def filter_executable_actions(actions, executable_prefix="move"):
    """Split actions into those the executor can drive and those it cannot.

    ``plan_to_nav2_goals`` only lowers actions whose name starts with ``move``;
    ``pickup-box``, ``dropoff-box`` and ``inspect-shelf`` have no executor yet.
    Historically those were dropped silently. Returning them explicitly lets the
    caller log a ``plan_action_unexecutable`` event per action, so a plan that
    quietly does less than it claims is visible rather than mysterious.

    Returns ``(executable, dropped)``.
    """
    executable, dropped = [], []
    for action in actions:
        (executable if action.name.startswith(executable_prefix) else dropped).append(action)
    return executable, dropped


MOVE_ACTIONS = frozenset({"move", "move-through-narrow-area"})


def align_plan_start(plan, current_region, robot_name):
    """Repair a plan whose first move starts from where the robot is not.

    A planner works from the problem's ``(at ?r ?l)``, which can lag the robot
    by a region -- most obviously right after a hot-swap. Driving the plan
    verbatim then sends the first leg from a place the robot is not standing,
    which shows up as a first move that doubles back across the map.

    Anchored on the first MOVE rather than on ``plan[0]``. A plan may
    legitimately open with an action the executor never drives: ``search-for``
    records the planner's assumption about where an unlocated object is, and
    nothing orders it after the driving. Keying on ``plan[0]`` made this repair
    silently no-op in exactly that case. Actions before the first move are
    carried through untouched, so the plan text stays faithful to what the
    planner returned.

    Returns ``(plan, note)``; ``note`` is a human-readable string when
    something was changed, else ``None``. Returning it rather than logging
    keeps this module free of ROS types.
    """
    if not plan or current_region is None:
        return plan, None
    idx = next((i for i, a in enumerate(plan)
                if a.name in MOVE_ACTIONS and len(a.args) >= 3), None)
    if idx is None:
        return plan, None
    first = plan[idx]
    if first.args[0] != robot_name:
        return plan, None

    plan_from, plan_to = first.args[1], first.args[2]
    if plan_from == current_region:
        return plan, None
    head = plan[:idx]
    if plan_to == current_region:
        return ([*head, *plan[idx + 1:]],
                f"Skipping {first.text()} because robot is already in {current_region}")

    # Same action class as the plan already uses, so this stays agnostic about
    # which GroundAction type the caller parsed with.
    adjusted = type(first)(first.name, (first.args[0], current_region, *first.args[2:]))
    return ([*head, adjusted, *plan[idx + 1:]],
            f"Using live robot region for first move: {first.text()} -> {adjusted.text()}")


def plan_reaches_target(actions, target_region: str) -> bool:
    """True when the last executable move ends at ``target_region``.

    Guards against applying a plan that validates symbolically but, once the
    non-move actions are stripped, no longer drives the robot to the goal.
    """
    moves = [a for a in actions if a.name.startswith("move")]
    if not moves:
        return False
    return moves[-1].args[-1].lower() == target_region.lower()
