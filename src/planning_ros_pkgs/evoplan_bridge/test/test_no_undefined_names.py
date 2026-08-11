"""No module may reference a name that does not exist.

This exists because of a live failure, not as hygiene. Refactoring
``begin_object_approach`` replaced its ``found = locate_object(...)`` binding
but left one later reference to ``found`` in an event payload. Everything
imported, every unit test passed, the sim brought up cleanly, the robot drove
the whole survey and logged ``[OBJECT SIGHTED]`` -- and then the executor died
with ``NameError: name 'found' is not defined`` at the exact moment the object
approach began, twelve minutes into the run. The one code path that mattered
was the one nothing had executed.

The node cannot be imported off-robot (it needs rclpy), so no import-time or
unit-level check could have caught it. Static analysis can, in milliseconds.

Scope is deliberately narrow: undefined names and redefinitions only, not
style. A test that fails on line length trains people to ignore it.
"""

import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]

TARGETS = [
    REPO / "src" / "planning_ros_pkgs" / "evo_skill_ros" / "evo_skill_ros" / "nodes",
    REPO / "src" / "planning_ros_pkgs" / "evoplan_bridge" / "evoplan_bridge",
    REPO / "pipeline" / "evoplan_bridge_host",
]

#: The pyflakes findings this gate is for -- an allow-list, not a deny-list.
#: Written this way on purpose: a deny-list grows every time an unrelated
#: cosmetic warning appears somewhere in the tree, and a test that fails for
#: reasons nobody cares about is a test people learn to skip. These two are
#: the ones that are always a bug.
FATAL = (
    "undefined name",
    "redefinition of unused",
)


def sources():
    for target in TARGETS:
        yield from sorted(p for p in target.rglob("*.py")
                          if "__pycache__" not in p.parts)


@pytest.fixture(scope="module")
def pyflakes():
    proc = subprocess.run([sys.executable, "-m", "pyflakes", "--version"],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        pytest.skip("pyflakes not installed (pip install pyflakes)")


def test_there_are_sources_to_check(pyflakes):
    """A glob that silently matches nothing would make this pass forever."""
    found = list(sources())
    assert len(found) > 5, f"only found {found}"
    assert any(p.name == "eveo_plan_deploy.py" for p in found), (
        "the executor is the module this test exists for")


def test_no_undefined_names(pyflakes):
    proc = subprocess.run(
        [sys.executable, "-m", "pyflakes", *[str(p) for p in sources()]],
        capture_output=True, text=True, cwd=REPO)
    problems = [line for line in (proc.stdout + proc.stderr).splitlines()
                if any(fatal in line for fatal in FATAL)]
    assert not problems, (
        "an undefined name is a crash on whichever code path first reaches "
        "it:\n  " + "\n  ".join(problems))
