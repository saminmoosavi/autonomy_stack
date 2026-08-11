#!/usr/bin/env python3
"""Host-side replan service for the online EvoPlan loop.

Long-lived on purpose. The expensive parts of a replan -- loading the mission
library and graph, locating VAL, importing the evolution stack -- are paid once
at boot, so a request costs only planning. Per-request subprocess startup for
Fast Downward or OpenEvolve is a couple of seconds against LLM round trips, and
buys a hard ``subprocess`` timeout plus crash isolation that in-process code
cannot.

Asynchronous job API, so the ROS client can poll, time out and walk away rather
than blocking a callback on an LLM:

    GET  /health
    POST /replan                  -> 202 {"job_id": ...}
    GET  /replan/<job_id>         -> {"status": running|ok|failed|timeout|unsolvable, ...}
    POST /replan/<job_id>/cancel
    POST /observation             -> record a sighting, rewrite the problem

``/observation`` is what makes a find-an-object mission planable. Its problem
states the object's position as unknown and a goal that names it, which has no
achiever -- the robot drives that plan's executable prefix, a survey. Perception
reports the object's region partway through, this endpoint writes the problem
that names it, and the next replan is planned FROM that file.

Bound to 127.0.0.1. ``docker-compose.yml`` runs the ROS container on
``network_mode: host``, so the container reaches it with no port mapping.

Run:
    pipeline/replan_service.sh start
    python3 pipeline/evoplan_bridge_host/server.py --help
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
# The artifact naming is shared with the executor, which writes index 0 from
# inside the container. It lives in the ROS package because that is the side
# that cannot import from here.
sys.path.insert(0, str(HERE.parent.parent / "src" / "planning_ros_pkgs" / "evoplan_bridge"))

from planners import PlanResult, run_fast_downward, val_binary, validate_with_val  # noqa: E402
from problem_builder import (  # noqa: E402
    MissionLibrary, build_runtime_problem, regions_in_problem)
from evoplan_bridge.replan_artifacts import (  # noqa: E402
    next_index, safe_tag, write_artifacts)

REPO = HERE.parent.parent


class Job:
    """One replan request and its eventual result."""

    def __init__(self, job_id: str, payload: dict):
        self.id = job_id
        self.payload = payload
        self.result = PlanResult(status="running")
        self.cancelled = threading.Event()
        self.created = time.monotonic()
        #: The runtime problem actually planned against, set by _plan and
        #: re-set if the blocked-region relaxation rebuilds it. Kept on the job
        #: so _run can archive it on EVERY exit path, including the exception
        #: one -- a replan that fails is exactly when the problem is worth
        #: reading, and until now a failed fd_only replan left no trace of it.
        self.problem_text: str | None = None
        #: Where the base problem came from -- "mission:<id>" for the authored
        #: file, or the path of the problem /observation wrote once perception
        #: located the object. Archived so an artifact can be traced to what it
        #: was actually planned from.
        self.base_source: str = ""

    def to_dict(self) -> dict:
        r = self.result
        return {
            "job_id": self.id,
            "status": r.status,
            "plan": r.plan,
            "planner": r.planner,
            "valid": r.valid,
            "elapsed_s": round(r.elapsed_s, 3),
            "validator_output": r.validator_output,
            "error": r.error,
            "relaxed": r.relaxed,
        }


class ReplanService:
    """Planning logic, independent of the HTTP layer so it can be unit-tested."""

    def __init__(self, args):
        self.args = args
        self.started = time.monotonic()
        self.jobs: dict[str, Job] = {}
        self.lock = threading.Lock()
        #: (tag, object class) -> the last sighting reported by perception,
        #: recorded mid-drive by /observation. Kept so a replan planned for a
        #: trial whose object has already been seen is built against the real
        #: position even when the request itself does not carry one.
        self.observations: dict[tuple[str, str], dict] = {}

        self.mock_plan = None
        if args.mock_plan:
            self.mock_plan = [
                line for line in Path(args.mock_plan).read_text().splitlines()
                if line.strip() and not line.strip().startswith(";")
            ]

        self.missions = MissionLibrary(Path(args.jackal_in),
                                      extra_dirs=[REPO / 'pipeline' / 'missions'])
        self.fd_path = Path(args.fd_path)

        # Boot-time consistency check. A mission that names a region graph.json
        # does not have produces routes Nav2 cannot follow, and the failure
        # would otherwise surface much later as an unexplained abort.
        graph_file = Path(args.graph_file)
        if graph_file.is_file():
            graph = json.loads(graph_file.read_text())
            names = {str(r.get("name", "")).lower() for r in graph.get("regions", [])}
            for warning in self.missions.check_against_graph(names):
                print(f"[replan-service] WARNING {warning}", flush=True)
        else:
            print(f"[replan-service] WARNING graph file not found: {graph_file}", flush=True)

    # ------------------------------------------------------------------
    def health(self) -> dict:
        return {
            "ok": True,
            "planner_modes": ["fd_only", "fd_first", "evoplan", "evoplan_only"],
            "mock": self.mock_plan is not None,
            "val": val_binary() is not None,
            "fd": self.fd_path.is_file(),
            "missions": sorted(p.stem for p in Path(self.args.jackal_in).glob("factory_mission_*.pddl")),
            "uptime_s": round(time.monotonic() - self.started, 1),
        }

    def _sightings_for(self, payload: dict) -> list[dict]:
        """Every object located so far on this trial, oldest first.

        A list, not one record: a mission can hunt several objects
        (factory_tour_02 wants the cone AND the skateboard) and they are found
        minutes apart. Returning only the latest would make each discovery
        erase the last -- the problem would name whichever object was seen most
        recently and forget the other.
        """
        tag = safe_tag(payload.get("tag"), fallback=payload.get("mission_id") or "unknown")
        with self.lock:
            found = [dict(r) for (seen_tag, _), r in self.observations.items()
                     if seen_tag == tag]
        return sorted(found, key=lambda r: r.get("at", 0))

    def _remembered_sighting(self, payload: dict):
        """``found_object`` for a request that did not carry one.

        A request that does carry one is more current by construction -- it was
        built from observation memory at escalation time -- so this is only
        ever a fallback.
        """
        return [{"class": r["class"], "region": r["region"]}
                for r in self._sightings_for(payload)] or None

    def _base_problem(self, payload: dict, mission_id: str) -> tuple[str, str]:
        """The problem a replan starts from: ``(text, provenance)``.

        Normally the authored mission. But once perception has located the
        object, ``/observation`` has already written the problem that names its
        position -- and THAT file is what the next replan is planned from, so
        the artifact on disk is the real input rather than a parallel record
        the service happens to reproduce. Read back from disk each time, so the
        file is genuinely load-bearing: edit it between replans and the next
        one uses the edit.

        Falls back to the authored mission if the file has gone missing or
        cannot be parsed for its declared regions. A replan against a stale but
        valid problem beats no replan at all -- the robot is holding station.
        """
        records = self._sightings_for(payload)
        path = next((r.get("problem_file") for r in reversed(records)
                     if r.get("problem_file")), None)
        if path:
            try:
                text = Path(path).read_text()
                if regions_in_problem(text):
                    return text, path
                raise ValueError("no declared regions")
            except Exception as exc:  # noqa: BLE001
                print(f"[replan-service] WARNING observed problem {path} unusable "
                      f"({exc}); falling back to the authored mission", flush=True)
        return self.missions.problem_text(mission_id), f"mission:{mission_id}"

    def observe(self, payload: dict) -> dict:
        """Record a perception sighting and rewrite the problem around it.

        Called while the robot is still driving the survey, not after it. The
        mission problem states ``(location-unknown cone)`` and a goal naming
        the cone, which is unsolvable on purpose -- what the robot is executing
        is the plan's executable prefix. The single fact that turns that
        unsolvable problem into a solvable one is the object's region, and YOLO
        produces it mid-drive. Taking it the moment it exists means the
        approach can be planned against the real position rather than
        discovered only once the survey has run out.

        Deliberately does NOT plan. The mission is "survey, then approach", so
        a sighting must not cut the survey short; this updates state and leaves
        the decision to escalate where it already lives, in the executor.

        Idempotent per (tag, object): re-reporting the same region is a no-op,
        so the executor can call it on every tick without guarding.
        """
        tag = safe_tag(payload.get("tag"), fallback=payload.get("mission_id") or "unknown")
        obj = str(payload.get("class") or "").strip().lower()
        region = str(payload.get("region") or "").strip().lower()
        mission_id = payload.get("mission_id") or "factory_mission_01"
        if not obj or not region:
            return {"ok": False, "error": "class and region are required"}

        key = (tag, obj)
        with self.lock:
            previous = self.observations.get(key)
            self.observations[key] = {
                "class": obj, "region": region, "mission_id": mission_id,
                "evidence": payload.get("evidence"), "at": time.time(),
            }
        if previous and previous["region"] == region:
            return {"ok": True, "region": region, "changed": False}

        # Rebuild the mission problem around EVERY object located so far and
        # put it on disk, so the updated problem exists as an artifact at the
        # moment of detection rather than only inside the next replan request.
        #
        # Every object, not just this one. A mission may hunt several
        # (factory_tour_02 wants the cone and the skateboard), they are found
        # minutes apart, and all sightings for a trial write the same file --
        # so rebuilding around the latest alone would drop the earlier one's
        # (object-at ...) and shrink the goal back to a single object.
        known = self._sightings_for(payload)
        written = None
        try:
            base = self.missions.problem_text(mission_id)
            problem = build_runtime_problem(
                base, robot=payload.get("robot") or "jackal_1",
                current_region=payload.get("current_region"),
                blocked_regions=[], visited_regions=payload.get("visited_regions") or [],
                target_region=region, preserve_goal=False,
                found_object=[{"class": r["class"], "region": r["region"]}
                              for r in known])
            out_dir = REPO / "results" / "single_trials"
            out_dir.mkdir(parents=True, exist_ok=True)
            written = out_dir / f"{tag}_problem_object_found.pddl"
            written.write_text(problem)
            with self.lock:
                # On every record for this tag: _base_problem takes the newest
                # path, and each rebuild supersedes the last.
                for (seen_tag, _), record in self.observations.items():
                    if seen_tag == tag:
                        record["problem_file"] = str(written)
        except Exception as exc:  # noqa: BLE001 - never fail the robot's report
            print(f"[replan-service] WARNING could not write observed problem: {exc}",
                  flush=True)
            return {"ok": True, "region": region, "changed": True, "problem_file": None}

        others = [r["class"] for r in known if r["class"] != obj]
        print(f"[replan-service] observation: {obj} in {region} "
              f"({'moved from ' + previous['region'] if previous else 'first sighting'})"
              + (f"; also known: {', '.join(others)}" if others else "")
              + f" -> {written.name}", flush=True)
        return {"ok": True, "region": region, "changed": True,
                "known_objects": [r["class"] for r in known],
                "problem_file": str(written)}

    def submit(self, payload: dict) -> str:
        job_id = f"r-{uuid.uuid4().hex[:8]}"
        job = Job(job_id, payload)
        with self.lock:
            self.jobs[job_id] = job
        threading.Thread(target=self._run, args=(job,), daemon=True,
                         name=f"replan-{job_id}").start()
        return job_id

    def get(self, job_id: str) -> dict | None:
        with self.lock:
            job = self.jobs.get(job_id)
        return job.to_dict() if job else None

    def cancel(self, job_id: str) -> bool:
        with self.lock:
            job = self.jobs.get(job_id)
        if job is None:
            return False
        job.cancelled.set()
        return True

    # ------------------------------------------------------------------
    def _run(self, job: Job) -> None:
        started = time.monotonic()
        try:
            job.result = self._plan(job)
        except Exception as exc:  # noqa: BLE001 - a bad request must not kill the service
            job.result = PlanResult(status="failed", error=f"{type(exc).__name__}: {exc}")
        job.result.elapsed_s = time.monotonic() - started
        print(f"[replan-service] {job.id} {job.result.status} "
              f"planner={job.result.planner} valid={job.result.valid} "
              f"{job.result.elapsed_s:.1f}s", flush=True)
        self._save_replan_artifacts(job)

    # ------------------------------------------------------------------
    def _save_replan_artifacts(self, job: Job) -> None:
        """Archive the problem, the plan and the outcome, once per replan.

        Written on every exit path including failure. That is the whole point:
        the problem is most worth reading when planning DIDN'T work, and until
        now nothing persisted it. An `(at jackal_1 r6)` naming a region the
        problem never declared aborted Fast Downward's translator and killed a
        whole replan; it was only diagnosable because that run happened to be
        in fd_first mode and fell through to EvoPlan, which incidentally
        embedded the problem in its /tmp workspace. A fd_only run leaves
        nothing at all.

        Naming matches the trial's other artifacts (_observations.jsonl,
        _belief.json, _logs/) so everything for a run sorts together:

            <tag>_replan_1.pddl   the runtime problem actually planned against
            <tag>_replan_1.plan   the returned actions (empty file on failure)
            <tag>_replan_1.json   planner, status, elapsed, validity, VAL output

        Index 0 belongs to the executor -- the plan the robot started on, which
        never passes through this service -- so numbering starts here at 1 by
        arithmetic rather than by rule: next_index reads the sequence off disk.

        Never raises: losing a diagnostic artifact must not fail a replan the
        robot is waiting on.
        """
        try:
            # No tag: still archive, keyed by mission and job id, rather than
            # silently discarding the one record of this replan.
            tag = safe_tag(
                job.payload.get("tag"),
                fallback=f"{job.payload.get('mission_id') or 'unknown'}_{job.id}")

            out_dir = REPO / "results" / "single_trials"
            out_dir.mkdir(parents=True, exist_ok=True)
            # minimum=1: index 0 belongs to the executor's initial plan,
            # whether or not this trial happened to archive one.
            stem = write_artifacts(out_dir, tag, next_index(out_dir, tag, minimum=1),
                                   job.problem_text, job.result.plan, {
                "job_id": job.id,                 # ties back to /tmp/evoplan_r-<id>
                "source": "replan_service",
                "base_problem": getattr(job, "base_source", ""),
                "status": job.result.status,
                "planner": job.result.planner,
                "valid": job.result.valid,
                "relaxed": job.result.relaxed,
                "elapsed_s": round(job.result.elapsed_s, 3),
                "error": job.result.error,
                "validator_output": job.result.validator_output,
                "plan": job.result.plan,
                # The request fields that determine the problem, so a bad
                # artifact can be traced to what the executor believed.
                "mission_id": job.payload.get("mission_id"),
                "current_region": job.payload.get("current_region"),
                "target_region": (job.payload.get("goal") or {}).get("target_region"),
                "blocked_regions": job.payload.get("blocked_regions"),
                "preserve_goal": job.payload.get("preserve_goal"),
                "found_object": job.payload.get("found_object"),
                "planner_mode": job.payload.get("planner_mode"),
                "trigger": (job.payload.get("reason") or {}).get("trigger"),
            })
            print(f"[replan-service] {job.id} artifacts -> {stem.name}"
                  f".{{pddl,plan,json}}", flush=True)
        except Exception as exc:  # noqa: BLE001 - diagnostics must never break a replan
            print(f"[replan-service] {job.id} WARNING could not save artifacts: "
                  f"{type(exc).__name__}: {exc}", flush=True)

    def _plan(self, job: Job) -> PlanResult:
        payload = job.payload

        if self.mock_plan is not None:
            # Mock mode exists so the whole ROS-side hold/hot-swap path can be
            # exercised against a live sim without spending a single token.
            delay = float(self.args.mock_delay_s)
            deadline = time.monotonic() + delay
            while time.monotonic() < deadline:
                if job.cancelled.is_set():
                    return PlanResult(status="failed", planner="mock",
                                      error="cancelled during mock delay")
                time.sleep(0.1)
            return PlanResult(status="ok", plan=list(self.mock_plan), planner="mock",
                              valid=True, validator_output="mock service")

        mission_id = payload.get("mission_id") or "factory_mission_01"
        robot = payload.get("robot") or "jackal_1"
        base_problem, base_source = self._base_problem(payload, mission_id)
        job.base_source = base_source
        if not base_source.startswith("mission:"):
            print(f"[replan-service] {job.id} planning from the observed problem "
                  f"{Path(base_source).name}", flush=True)
        blocked = list(payload.get("blocked_regions") or [])
        visited = self._visited_from(payload.get("executed_plan") or [])
        current_region = payload.get("current_region")

        # build_runtime_problem drops an undeclared start rather than emit a
        # problem FD's translator aborts on. Say so: a wrong current_region
        # means the robot's pose is being mis-attributed upstream, and a silent
        # drop hides that behind a plan that merely looks suboptimal.
        if current_region and current_region.lower() not in regions_in_problem(base_problem):
            print(f"[replan-service] WARNING current_region '{current_region}' is not "
                  f"declared in {mission_id}; ignoring it and planning from the "
                  f"problem's authored start", flush=True)

        target_region = (payload.get("goal") or {}).get("target_region")
        # None => decide from the mission goal's own predicates (a coverage tour
        # is preserved, a delivery mission is narrowed). A request may override.
        preserve_goal = payload.get("preserve_goal")
        # {"class": <detector class>, "region": <region>} once perception has
        # localised the mission object. Turns the narrowed goal from a bare
        # (at robot R) into (reached robot <object>) when the mission declares
        # a `target`; absent or unresolvable, the old behaviour stands.
        found_object = payload.get("found_object") or self._remembered_sighting(payload)

        def make_problem(blocked_regions):
            return build_runtime_problem(
                base_problem, robot=robot, current_region=current_region,
                blocked_regions=blocked_regions, visited_regions=visited,
                target_region=target_region, preserve_goal=preserve_goal,
                found_object=found_object,
            )

        problem_text = make_problem(blocked)
        job.problem_text = problem_text
        domain_text = self.missions.domain_text
        mode = payload.get("planner_mode") or "fd_first"
        deadline_s = float(payload.get("deadline_s") or 45.0)
        fd_timeout = min(self.args.fd_timeout_s, deadline_s)
        relaxed = False

        # evoplan_only skips Fast Downward entirely. Everywhere else the
        # unsolvability check runs first because handing an impossible problem
        # to an LLM burns the whole deliberation budget for nothing -- but that
        # check also REJECTS the case this mode exists for. A goal naming an
        # object whose position is unknown is unsolvable by construction, and
        # the useful answer is not "no plan": it is the plan's valid prefix,
        # which surveys the regions the object might be in. FD cannot express
        # that, so in this mode it is not consulted and there is no fallback
        # plan to fall back to.
        if mode == "evoplan_only":
            fd = PlanResult(status="skipped", plan=[], planner="none",
                            error="fast downward not consulted (evoplan_only)")
            return self._run_evoplan(job, domain_text, problem_text, payload,
                                     deadline_s, fd, blocked, relaxed)

        fd = run_fast_downward(str(self.fd_path), domain_text, problem_text,
                               timeout_s=fd_timeout)

        if fd.status == "unsolvable" and blocked:
            # The east corridor of this warehouse (R9-R10-R11-R12-R13) is a bare
            # chain: every region in it is a cut vertex, so blocking the one
            # ahead of the robot disconnects the goal outright. But (blocked ?l)
            # is a *hint* derived from a social or perception event, not a claim
            # that the corridor is physically impassable -- a person standing in
            # R10 makes it costly, not absent. When honouring the hint makes the
            # mission impossible, routing through anyway and letting the shield
            # and the reactive layer manage the encounter beats stranding the
            # robot. The relaxation is reported so it shows up in the metrics
            # rather than looking like an ordinary replan.
            print(f"[replan-service] {job.id} unsolvable with blocked={blocked}; "
                  f"relaxing", flush=True)
            relaxed = True
            problem_text = make_problem([])
            job.problem_text = problem_text   # archive what was really planned
            fd = run_fast_downward(str(self.fd_path), domain_text, problem_text,
                                   timeout_s=fd_timeout)
            if fd.status == "ok":
                fd.error = (
                    f"blocked regions {sorted(blocked)} disconnect the goal; "
                    "replanned without them"
                )

        if fd.status == "unsolvable":
            return fd
        if fd.status == "ok":
            valid, output = validate_with_val(domain_text, problem_text, fd.plan)
            fd.valid = valid
            fd.validator_output = output
            fd.relaxed = relaxed
            if mode in ("fd_only", "fd_first") and valid:
                return fd
        if mode == "fd_only":
            fd.relaxed = relaxed
            return fd

        # Pass the EFFECTIVE blocked list, not the requested one. When the
        # relaxation above fired, the problem handed to EvoPlan no longer
        # asserts those (blocked ...) facts -- telling the evaluator and the
        # LLM otherwise asks them to route around a region the planner has
        # already proved unroutable (r10 is a cut vertex in this warehouse).
        # The LLM then answers in prose instead of code, OpenEvolve reports
        # 'No valid code found in response', and every iteration is wasted.
        effective_blocked = [] if relaxed else blocked
        return self._run_evoplan(job, domain_text, problem_text, payload,
                                 deadline_s, fd, effective_blocked, relaxed)

    def _run_evoplan(self, job, domain_text, problem_text, payload, deadline_s, fd,
                     effective_blocked=None, relaxed=False):
        """Hand off to the EvoPlan evolutionary loop (Phase 4)."""
        try:
            from evoplan_runner import run_evoplan_repair
        except ImportError as exc:
            fd.error = f"evoplan backend unavailable ({exc}); returning FD result"
            return fd
        return run_evoplan_repair(
            effective_blocked=effective_blocked,
            relaxed=relaxed,
            job=job,
            domain_text=domain_text,
            problem_text=problem_text,
            payload=payload,
            deadline_s=deadline_s,
            fallback=fd,
            args=self.args,
        )

    @staticmethod
    def _visited_from(executed_plan) -> list[str]:
        """Destination regions of already-executed moves."""
        visited = []
        for line in executed_plan:
            match = re.search(r"\(([^)]+)\)", line)
            if not match:
                continue
            parts = match.group(1).split()
            if parts and parts[0].startswith("move") and len(parts) >= 2:
                visited.append(parts[-1].lower())
        return visited


class Handler(BaseHTTPRequestHandler):
    service: ReplanService = None  # set on the server instance

    def log_message(self, fmt, *args):
        pass  # the service prints its own, one line per job

    def _send(self, code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            return self._send(200, self.service.health())
        match = re.fullmatch(r"/replan/([\w-]+)", self.path)
        if match:
            state = self.service.get(match.group(1))
            return self._send(200, state) if state else self._send(404, {"error": "no such job"})
        self._send(404, {"error": "not found"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw or b"{}")
        except json.JSONDecodeError as exc:
            return self._send(400, {"error": f"bad JSON: {exc}"})

        match = re.fullmatch(r"/replan/([\w-]+)/cancel", self.path)
        if match:
            ok = self.service.cancel(match.group(1))
            return self._send(200 if ok else 404, {"cancelled": ok})
        if self.path == "/replan":
            job_id = self.service.submit(payload)
            return self._send(202, {"job_id": job_id, "accepted_at": time.time()})
        if self.path == "/observation":
            return self._send(200, self.service.observe(payload))
        self._send(404, {"error": "not found"})


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8077)
    p.add_argument("--jackal-in", default=str(REPO / "evolve_stl_pddl" / "jackal" / "in"),
                   help="directory holding factory_jackal_domain.pddl and the missions")
    p.add_argument("--graph-file",
                   default=str(REPO / "src" / "planning_ros_pkgs" / "evo_skill_ros"
                               / "config" / "graph.json"))
    p.add_argument("--fd-path", default=str(REPO / "fast_downward" / "fast-downward.py"))
    p.add_argument("--fd-timeout-s", type=float, default=10.0)
    p.add_argument("--openevolve-run", default=str(Path.home() / "openevolve" / "openevolve-run.py"))
    p.add_argument("--evoplan-config", default=str(HERE / "config_jackal_openai.yaml"))
    p.add_argument("--evoplan-model", default=os.environ.get("EVOPLAN_MODEL"),
                   help="override llm.primary_model in the config (or set $EVOPLAN_MODEL)")
    p.add_argument("--evoplan-iterations", type=int, default=6,
                   help="online repair budget; the offline benchmark used 15-30")
    p.add_argument("--mock-plan", default=None,
                   help="serve this plan file instead of planning (zero-token testing)")
    p.add_argument("--mock-delay-s", type=float, default=5.0)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    service = ReplanService(args)
    Handler.service = service
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    mode = "MOCK" if service.mock_plan is not None else "live"
    print(f"[replan-service] listening on http://{args.host}:{args.port} ({mode}); "
          f"val={'yes' if val_binary() else 'NO'} fd={'yes' if Path(args.fd_path).is_file() else 'NO'}",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("[replan-service] shutting down", flush=True)
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
