#!/usr/bin/env python3
"""The mined social-compliance envelope, Phi_mob.

These thresholds come from ``mined_constraints.md`` -- the SCAND-mined formula
produced by the ``stl_formula_evolve`` half of the EvoPlan work. The full mined
formula is a 17-conjunct ``G(...)`` conjunction; the eight below are the subset
whose signals this stack actually computes.

This module exists so those numbers live in exactly one place. Both consumers
import from here:

* ``scand_metrics.py`` -- offline scoring against Gazebo ground truth, which
  reports per-conjunct compliance percentages;
* ``evoplan_bridge.phi_mob_shield`` -- the online shield, which computes
  robustness from perception and can veto.

Keeping one table means a "shield vetoed but the ground-truth scorer saw no
violation" disagreement is a real perception finding rather than a transcription
slip between two copies of the constants.

Deliberately dependency-free (stdlib only) so the host-side orchestrator can
import it without a ROS environment.
"""

from __future__ import annotations

import math

__all__ = ["ENVELOPE", "SIGNALS", "conjunct_margin", "normalizer"]

#: ``(signal_name, comparison_operator, threshold)``, straight from
#: ``mined_constraints.md``. Read as ``G(signal op threshold)``.
ENVELOPE = (
    ("speed", "<=", 2.603),
    ("yaw_rate", "<=", 3.293),
    ("accel", "<=", 3.213),
    ("jerk", "<=", 5.862),
    ("lat_accel", "<=", 2.228),
    ("ttc", ">=", 0.452),
    ("ped_ttc", ">=", 0.422),
    ("ped_approach_rate", "<=", 5.0),
)

#: Signal names in envelope order.
SIGNALS = tuple(name for name, _, _ in ENVELOPE)

#: Conjuncts the online shield may VETO on. The rest are still monitored and
#: reported -- they just cannot trigger a replan.
#:
#: Excluded, and why:
#:   speed  a symbolic REROUTE cannot slow the robot down. Sending it to
#:          different regions does nothing about exceeding 2.603 m/s, so the
#:          replan is structurally incapable of fixing the violation -- it just
#:          costs a hold and consumes the replan budget. Observed in the
#:          pedestrian-free objects world: both symbolic replans fired on speed
#:          (rho -0.034, -0.146), exhausted the 2-replan budget, and truncated a
#:          14-region tour at 5 regions. The correct response to a speed
#:          violation is a velocity limit in the controller, not a new plan.
#:   jerk   third derivative of position by finite differences, so it amplifies
#:          odometry noise enormously and Nav2's start-up acceleration transient
#:          blows past the bound within seconds of the robot moving. Observed
#:          firing at rho=-1.8 then -25.8 at t=2.4s on an empty aisle, which
#:          discarded a 28-action mission. It is a ride-quality measure, not a
#:          safety signal.
#:   accel  same transient, one derivative less severe but same failure mode.
#:   lat_accel / yaw_rate
#:          fire on ordinary in-place turns, which Nav2 does at every waypoint.
#:
#: What remains are the signals a symbolic reroute can actually act on: closing
#: too quickly on a person, whose position determines which route is safe. Every
#: excluded signal is a property of HOW the robot drives, not WHERE -- and only
#: the latter is something a new plan can change.
#:
#: Ground-truth compliance over the FULL envelope is still scored by
#: scand_metrics.py, so nothing is lost from the reported results.
VETO_SIGNALS = ("ttc", "ped_ttc", "ped_approach_rate")

#: The subset of ENVELOPE the shield escalates on.
VETO_ENVELOPE = tuple(c for c in ENVELOPE if c[0] in VETO_SIGNALS)


def normalizer(threshold: float) -> float:
    """Scale factor making conjunct margins commensurable.

    Raw margins are in mixed units -- m/s, rad/s, m/s^3, seconds -- so an
    unnormalized ``min`` over conjuncts would just always report whichever
    signal happens to have the largest numeric range. Dividing by the threshold
    turns every margin into a dimensionless fraction of its own bound, so
    ``-0.5`` means "50% past the limit" for any conjunct.
    """
    return 1.0 / abs(threshold) if abs(threshold) > 1e-9 else 1.0


def conjunct_margin(value: float, op: str, threshold: float) -> float:
    """Normalized robustness of one ``G(value op threshold)`` conjunct.

    Positive means satisfied, negative means violated, and the magnitude is the
    fraction of the threshold by which it holds or fails.

    A non-finite value means the signal is undefined right now -- most often an
    infinite time-to-collision because nothing is ahead of the robot. That is
    trivially compliant for a ``>=`` bound, so it returns ``+inf`` rather than
    NaN; ``scand_metrics.py`` makes the same call when it counts an infinite TTC
    as a compliant tick. An undefined ``<=`` signal is treated the same way
    (vacuously satisfied) so a missing signal can never manufacture a veto.
    """
    if not math.isfinite(value):
        return math.inf
    scale = normalizer(threshold)
    if op == "<=":
        return (threshold - value) * scale
    if op == ">=":
        return (value - threshold) * scale
    raise ValueError(f"unsupported envelope operator: {op!r}")
