"""The executor and the planner must agree on the domain.

Two copies of the classical factory domain exist by necessity:

* ``evolve_stl_pddl/jackal/in/factory_jackal_domain.pddl`` -- canonical. The
  replan service, Fast Downward and VAL all use this one.
* ``src/planning_ros_pkgs/evo_skill_ros/config/factory_sim_domain.pddl`` -- the
  ROS copy, referenced from eight places and installed into the package share.

They were previously *different languages*: the ROS side held a durative
(PDDL 2.1) model whose ``move`` required ``available``/``localized``/
``battery-ok``/``safe``, none of which any mission problem asserts. The
executor validated planner output against it and would have rejected every
legal plan -- masked only by a second bug that made the validation call throw
and be swallowed. These tests make that class of divergence loud.
"""

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
CANONICAL = REPO / "evolve_stl_pddl" / "jackal" / "in" / "factory_jackal_domain.pddl"
ROS_COPY = (REPO / "src" / "planning_ros_pkgs" / "evo_skill_ros" / "config"
            / "factory_sim_domain.pddl")
MISSIONS = sorted((REPO / "evolve_stl_pddl" / "jackal" / "in").glob("factory_mission_*.pddl"))


def strip_header(text):
    """Drop leading ';;' comment lines so provenance headers do not count."""
    lines = text.splitlines()
    i = 0
    while i < len(lines) and (not lines[i].strip() or lines[i].lstrip().startswith(";")):
        i += 1
    return "\n".join(lines[i:]).strip()


def actions(text):
    return sorted(set(re.findall(r":(?:durative-)?action\s+([a-z0-9-]+)", text)))


def predicates(text):
    block = re.search(r"\(:predicates(.*?)\n\s*\)", text, re.S)
    return sorted(set(re.findall(r"\(([a-z0-9-]+)\s", block.group(1)))) if block else []


@pytest.fixture(scope="module")
def texts():
    if not CANONICAL.is_file() or not ROS_COPY.is_file():
        pytest.skip("domain files not present (submodule not initialised?)")
    return CANONICAL.read_text(), ROS_COPY.read_text()


class TestCopiesAgree:
    def test_bodies_are_identical(self, texts):
        """Byte-identical below the header. Anything else is drift."""
        canon, ros = texts
        assert strip_header(canon) == strip_header(ros), (
            "factory_sim_domain.pddl has drifted from factory_jackal_domain.pddl; "
            "re-copy the canonical file and keep only the provenance header"
        )

    def test_same_actions(self, texts):
        assert actions(texts[0]) == actions(texts[1])

    def test_same_predicates(self, texts):
        assert predicates(texts[0]) == predicates(texts[1])


class TestClassicalNotDurative:
    def test_ros_copy_is_classical(self, texts):
        """Fast Downward rejects :durative-actions with translate exit code 31,
        and FD/VAL score every candidate plan -- so the live domain must be
        classical or the planner cannot use it at all.

        Checked on the stripped body: the provenance header explains the
        durative history in prose and must not trip the check.
        """
        body = strip_header(texts[1])
        assert ":durative-actions" not in body
        assert ":durative-action " not in body

    def test_no_numeric_fluents(self, texts):
        """(:functions ...) needs :fluents, which FD's translator also rejects."""
        assert "(:functions" not in strip_header(texts[1])


class TestProblemsSatisfyTheDomain:
    """Every precondition the domain demands must be assertable by the problems.

    This is the check that would have caught the original divergence: the
    durative domain required four predicates that appear in zero problems.
    """

    DURATIVE_ONLY = ("available", "localized", "battery-ok", "safe",
                     "narrow", "occupied-by-human")

    @pytest.mark.parametrize("predicate", DURATIVE_ONLY)
    def test_live_domain_does_not_require_unassertable_predicates(self, texts, predicate):
        assert f"({predicate} " not in texts[1], (
            f"the live domain references ({predicate} ...), which no "
            f"factory_mission_*.pddl asserts -- plans would be rejected on a "
            f"condition the planner was never asked to satisfy"
        )

    def test_move_preconditions_are_all_assertable(self, texts):
        """Whatever `move` requires must be groundable from a mission problem."""
        if not MISSIONS:
            pytest.skip("no mission problems present")
        block = re.search(r"\(:action move(.*?)\n  \)", strip_header(texts[1]), re.S)
        assert block, "move action not found in the live domain"
        pre = re.search(r":precondition(.*?):effect", block.group(1), re.S)
        assert pre, "move has no :precondition"
        text = pre.group(1)
        # Drop NEGATIVE preconditions: (not (blocked ?to)) is satisfied by
        # absence, and (blocked ?l) is asserted at runtime by execution
        # failures -- that is precisely the replan mechanism, so requiring it
        # in :init would be backwards.
        text = re.sub(r"\(not\s+\([^)]*\)\s*\)", " ", text)
        required = set(re.findall(r"\(([a-z0-9-]+)\s+\?", text))
        required -= {"not", "and"}
        # Effects-only predicates need not appear in :init.
        required -= {"visited"}
        corpus = "\n".join(m.read_text() for m in MISSIONS)
        for pred in sorted(required):
            assert f"({pred} " in corpus, (
                f"move requires ({pred} ...) but no mission problem asserts it"
            )
