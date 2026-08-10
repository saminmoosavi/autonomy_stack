#!/usr/bin/env python3
"""Client for the host-side replan service.

The evolution loop runs on the host (Python 3.11+ with openevolve and VAL); the
executor runs in the ROS Humble container. ``docker-compose.yml`` puts the
container on ``network_mode: host``, so the two talk over plain HTTP on
127.0.0.1 with no port plumbing at all.

Threading contract, which is the whole point of this module: ``request()`` runs
on a worker thread that touches **no node state**. It is handed an immutable
snapshot dict and drops a result dict on a ``queue.Queue``. The node drains that
queue from an ordinary timer callback, so every mutation of the executor's
bookkeeping still happens on the single rclpy executor thread. That is why this
does not need a ReentrantCallbackGroup or a MultiThreadedExecutor -- the
existing node mutates a dozen fields from four callbacks with no locking, and
making its executor multithreaded would turn latent races into real ones.

stdlib ``urllib`` only: the container has no requests/httpx and does not need
them for four JSON calls.
"""

from __future__ import annotations

import json
import queue
import threading
import time
import urllib.error
import urllib.request

__all__ = ["ReplanResult", "ReplanClient"]

#: Async-job polling, seconds (wall clock). Backs off from a tight first poll to
#: a relaxed steady state: fd_only plans return in ~0.08s, so a flat 1.0s
#: interval spent 1.12s of a 1.31s hold just waiting to ask again -- 15x the
#: actual work, and it lands in the deliberation-cost column that the fd-vs-
#: evoplan comparison is measured on. EvoPlan runs take seconds to tens of
#: seconds, where the relaxed interval costs nothing.
POLL_INITIAL_S = 0.05
POLL_MAX_S = 1.0
POLL_BACKOFF = 1.6


class ReplanResult(dict):
    """Result of one replan attempt.

    A plain dict so it crosses the queue with no shared mutable state. Keys:
    ``status`` (``ok`` | ``failed`` | ``timeout`` | ``error``), ``plan`` (list of
    ``"(action args)"`` strings), ``planner``, ``valid``, ``elapsed_s``,
    ``job_id``, ``validator_output``, ``error``.
    """

    @property
    def usable(self) -> bool:
        """True when the plan may be applied to the robot.

        A ``timeout`` with a validated best-so-far plan is still usable --
        partial credit beats stranding the robot. An unvalidated plan never is.
        """
        return bool(self.get("plan")) and bool(self.get("valid"))


class ReplanClient:
    """Fire-and-poll client for the replan service.

    One request in flight at a time; the caller enforces that via its state
    machine, and :meth:`busy` is offered so it can assert rather than assume.
    """

    def __init__(self, base_url: str, http_timeout_s: float = 60.0, logger=None):
        self.base_url = base_url.rstrip("/")
        self.http_timeout_s = float(http_timeout_s)
        self.results: queue.Queue = queue.Queue()
        self._log = logger
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # public API (called from the rclpy executor thread)
    # ------------------------------------------------------------------
    def busy(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def health(self, timeout_s: float = 3.0) -> dict | None:
        """Probe ``/health``. Returns the payload, or ``None`` if unreachable.

        Blocking, but only for ``timeout_s`` -- intended for a one-shot check at
        startup, never from a periodic callback.
        """
        try:
            return self._get("/health", timeout_s)
        except Exception:
            return None

    def request_async(self, payload: dict, deadline_s: float) -> bool:
        """Start a replan on a worker thread. Returns False if one is running.

        ``payload`` is deep-copied via a JSON round trip so the worker cannot
        observe later mutations of node state -- the snapshot is frozen at the
        moment of the call.
        """
        if self.busy():
            return False
        frozen = json.loads(json.dumps(payload))
        self._thread = threading.Thread(
            target=self._run,
            args=(frozen, float(deadline_s)),
            daemon=True,
            name="evoplan-replan",
        )
        self._thread.start()
        return True

    def poll(self) -> ReplanResult | None:
        """Non-blocking: return a finished result, or ``None``."""
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    # ------------------------------------------------------------------
    # worker thread -- must not touch node state
    # ------------------------------------------------------------------
    def _run(self, payload: dict, deadline_s: float) -> None:
        started = time.monotonic()
        job_id = None
        try:
            accepted = self._post("/replan", payload, self.http_timeout_s)
            job_id = accepted.get("job_id")
            if not job_id:
                raise RuntimeError(f"service accepted the request without a job_id: {accepted}")

            interval = POLL_INITIAL_S
            while True:
                elapsed = time.monotonic() - started
                if elapsed >= deadline_s:
                    # Ask the service to stop burning tokens, then report what
                    # it had. The service returns a validated best-so-far plan
                    # if it has one.
                    self._safe_cancel(job_id)
                    final = self._safe_get(f"/replan/{job_id}")
                    result = ReplanResult(final or {})
                    result.setdefault("plan", [])
                    result.setdefault("valid", False)
                    result["status"] = "timeout"
                    result["job_id"] = job_id
                    result["elapsed_s"] = elapsed
                    self.results.put(result)
                    return

                state = self._get(f"/replan/{job_id}", self.http_timeout_s)
                if state.get("status") != "running":
                    result = ReplanResult(state)
                    result["job_id"] = job_id
                    result.setdefault("elapsed_s", time.monotonic() - started)
                    self.results.put(result)
                    return

                time.sleep(min(interval, max(0.0, deadline_s - elapsed)))
                interval = min(POLL_MAX_S, interval * POLL_BACKOFF)
        except Exception as exc:  # noqa: BLE001 - any failure must reach the node
            if self._log is not None:
                self._log.warn(f"replan service request failed: {exc}")
            self.results.put(
                ReplanResult(
                    status="error",
                    error=str(exc),
                    plan=[],
                    valid=False,
                    job_id=job_id,
                    elapsed_s=time.monotonic() - started,
                )
            )

    # ------------------------------------------------------------------
    # transport
    # ------------------------------------------------------------------
    def _get(self, path: str, timeout_s: float) -> dict:
        req = urllib.request.Request(self.base_url + path, method="GET")
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def _post(self, path: str, payload: dict, timeout_s: float) -> dict:
        body = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.base_url + path,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def _safe_get(self, path: str) -> dict | None:
        try:
            return self._get(path, self.http_timeout_s)
        except Exception:
            return None

    def _safe_cancel(self, job_id: str) -> None:
        try:
            self._post(f"/replan/{job_id}/cancel", {}, 5.0)
        except Exception:
            pass  # best effort; the deadline is enforced here regardless
