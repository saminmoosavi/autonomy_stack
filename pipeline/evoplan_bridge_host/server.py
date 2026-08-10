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

from planners import PlanResult, run_fast_downward, val_binary, validate_with_val  # noqa: E402
from problem_builder import MissionLibrary, build_runtime_problem  # noqa: E402

REPO = HERE.parent.parent


class Job:
    """One replan request and its eventual result."""

    def __init__(self, job_id: str, payload: dict):
        self.id = job_id
        self.payload = payload
        self.result = PlanResult(status="running")
        self.cancelled = threading.Event()
        self.created = time.monotonic()

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
            "planner_modes": ["fd_only", "fd_first", "evoplan"],
            "mock": self.mock_plan is not None,
            "val": val_binary() is not None,
            "fd": self.fd_path.is_file(),
            "missions": sorted(p.stem for p in Path(self.args.jackal_in).glob("factory_mission_*.pddl")),
            "uptime_s": round(time.monotonic() - self.started, 1),
        }

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
        base_problem = self.missions.problem_text(mission_id)
        blocked = list(payload.get("blocked_regions") or [])
        visited = self._visited_from(payload.get("executed_plan") or [])
        current_region = payload.get("current_region")

        target_region = (payload.get("goal") or {}).get("target_region")
        # None => decide from the mission goal's own predicates (a coverage tour
        # is preserved, a delivery mission is narrowed). A request may override.
        preserve_goal = payload.get("preserve_goal")

        def make_problem(blocked_regions):
            return build_runtime_problem(
                base_problem, robot=robot, current_region=current_region,
                blocked_regions=blocked_regions, visited_regions=visited,
                target_region=target_region, preserve_goal=preserve_goal,
            )

        problem_text = make_problem(blocked)
        domain_text = self.missions.domain_text
        mode = payload.get("planner_mode") or "fd_first"
        deadline_s = float(payload.get("deadline_s") or 45.0)
        fd_timeout = min(self.args.fd_timeout_s, deadline_s)
        relaxed = False

        # Unsolvability check first, in every mode. If (blocked ...) has made
        # the problem impossible, FD proves it in well under a second; handing
        # that to an LLM would burn the whole deliberation budget on a task with
        # no solution.
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
