"""The executor's status blob and scand_metrics' reader must agree on keys.

``evo_plan_deploy`` publishes a JSON blob on ``evoplan/status`` at 1 Hz;
``scand_metrics`` subscribes and gates both SUCCESS and TERMINATION on fields
inside it. Nothing types that interface -- a reader asking for a key the
publisher never emits gets ``None``, which is falsy, so the gate silently never
fires and the run hangs instead of erroring.

That has now happened twice with the same shape:

* ``mission_complete`` -- published only when Tier-2 was enabled, so every
  ``SYMBOLIC_REPLAN=0`` run scored against position alone.
* ``plan_exhausted`` -- read at ``scand_metrics.py:457`` to stop a run whose
  plan finished short of the mission, but never added to the publisher. The
  executor logged ``[PLAN EXHAUSTED]`` and the trial then idled until its
  timeout; one run was found still up, whole sim running, 13.4 hours later.

A static key check is enough to catch it and needs no ROS, no sim and no node.
"""

import ast
import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
EXECUTOR = (REPO / "src" / "planning_ros_pkgs" / "evo_skill_ros" / "evo_skill_ros"
            / "nodes" / "eveo_plan_deploy.py")
METRICS = REPO / "scand_metrics.py"

#: Keys scand_metrics may read without the publisher emitting them. Anything
#: here is a deliberate optional field, not an oversight.
OPTIONAL = frozenset()


@pytest.fixture(scope="module")
def published_keys():
    """String literal keys of the dict built inside publish_evoplan_status."""
    if not EXECUTOR.is_file():
        pytest.skip("executor source not present")
    tree = ast.parse(EXECUTOR.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "publish_evoplan_status":
            for sub in ast.walk(node):
                if isinstance(sub, ast.Dict):
                    return {k.value for k in sub.keys
                            if isinstance(k, ast.Constant) and isinstance(k.value, str)}
    pytest.fail("publish_evoplan_status does not build a dict literal")


@pytest.fixture(scope="module")
def consumed_keys():
    """Keys scand_metrics pulls off the status blob via .get("...")."""
    if not METRICS.is_file():
        pytest.skip("scand_metrics.py not present")
    text = METRICS.read_text()
    keys = set()
    # Both spellings in use: `self.evoplan_status.get("x")` and, after
    # `status = self.evoplan_status or {}`, plain `status.get("x")`.
    for pattern in (r"evoplan_status\s*(?:or\s*\{\})?\)?\s*\.get\(\s*[\"']([\w_]+)[\"']",
                    r"\bstatus\.get\(\s*[\"']([\w_]+)[\"']"):
        keys.update(re.findall(pattern, text))
    return keys


class TestKeysAgree:
    def test_every_consumed_key_is_published(self, published_keys, consumed_keys):
        missing = consumed_keys - published_keys - OPTIONAL
        assert not missing, (
            f"scand_metrics reads {sorted(missing)} off the status blob but "
            f"publish_evoplan_status never emits them -- .get() returns None, "
            f"the gate silently never fires, and the run hangs. Published: "
            f"{sorted(published_keys)}"
        )

    def test_the_two_gating_keys_are_present(self, published_keys):
        """Named explicitly: these are the two that gate success and stopping,
        and both have been the regression already."""
        for key in ("mission_complete", "plan_exhausted"):
            assert key in published_keys, f"{key} missing from the status blob"

    def test_consumer_actually_reads_something(self, consumed_keys):
        """Guards the regex itself -- a silent zero-match would make the
        contract test above pass unconditionally."""
        assert len(consumed_keys) >= 3, f"suspiciously few keys parsed: {consumed_keys}"
