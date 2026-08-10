"""Integration tests for ReplanClient against a stub HTTP server.

Exercises the transport, the async job protocol, the deadline and the failure
paths without ROS, without the real planner and without spending tokens. These
are the paths that strand the robot if they are wrong.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from evoplan_bridge.replan_client import ReplanClient

PLAN = ["(move jackal_1 r9 r7)", "(move jackal_1 r7 r8)"]


class StubHandler(BaseHTTPRequestHandler):
    """Mimics the real service's job API. Behaviour driven by ``server.script``."""

    def log_message(self, *a):
        pass

    def _send(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        script = self.server.script
        if self.path == "/health":
            return self._send(200, {"ok": True})
        if self.path.startswith("/replan/"):
            self.server.polls += 1
            if self.server.polls < script.get("running_polls", 0):
                return self._send(200, {"status": "running"})
            return self._send(200, script["final"])
        self._send(404, {})

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length else b"{}"
        if self.path.endswith("/cancel"):
            self.server.cancelled = True
            return self._send(200, {"cancelled": True})
        if self.path == "/replan":
            self.server.last_payload = json.loads(body)
            if self.server.script.get("reject_post"):
                return self._send(500, {"error": "boom"})
            return self._send(202, {"job_id": "r-test"})
        self._send(404, {})


@pytest.fixture
def stub():
    server = ThreadingHTTPServer(("127.0.0.1", 0), StubHandler)
    server.script = {"running_polls": 0, "final": {"status": "ok"}}
    server.polls = 0
    server.cancelled = False
    server.last_payload = None
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server
    server.shutdown()
    server.server_close()


def url(server):
    return f"http://127.0.0.1:{server.server_address[1]}"


def drain(client, timeout_s=10.0):
    """Block until the worker delivers a result."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = client.poll()
        if result is not None:
            return result
        time.sleep(0.02)
    raise AssertionError("client never produced a result")


class TestHappyPath:
    def test_returns_a_usable_plan(self, stub):
        stub.script["final"] = {"status": "ok", "plan": PLAN, "valid": True,
                                "planner": "fd"}
        client = ReplanClient(url(stub))
        assert client.request_async({"mission_id": "m1"}, deadline_s=10)
        result = drain(client)
        assert result["status"] == "ok"
        assert result["plan"] == PLAN
        assert result.usable
        assert result["job_id"] == "r-test"
        assert result["elapsed_s"] >= 0

    def test_polls_until_the_job_finishes(self, stub):
        stub.script.update(running_polls=3,
                           final={"status": "ok", "plan": PLAN, "valid": True})
        client = ReplanClient(url(stub))
        client.request_async({}, deadline_s=20)
        assert drain(client, timeout_s=20)["plan"] == PLAN
        assert stub.polls >= 3

    def test_payload_reaches_the_service_intact(self, stub):
        stub.script["final"] = {"status": "ok", "plan": PLAN, "valid": True}
        client = ReplanClient(url(stub))
        client.request_async(
            {"mission_id": "factory_mission_07", "blocked_regions": ["r10"]},
            deadline_s=10)
        drain(client)
        assert stub.last_payload["mission_id"] == "factory_mission_07"
        assert stub.last_payload["blocked_regions"] == ["r10"]

    def test_health_probe(self, stub):
        assert ReplanClient(url(stub)).health() == {"ok": True}


class TestUsability:
    """`usable` gates whether the robot is allowed to act on the result."""

    @pytest.mark.parametrize("final,expected", [
        ({"status": "ok", "plan": PLAN, "valid": True}, True),
        ({"status": "ok", "plan": PLAN, "valid": False}, False),   # unvalidated
        ({"status": "ok", "plan": [], "valid": True}, False),      # empty
        ({"status": "failed", "plan": [], "valid": False}, False),
        # A timeout that still carried a validated best-so-far IS usable:
        # partial credit beats stranding the robot.
        ({"status": "timeout", "plan": PLAN, "valid": True}, True),
    ])
    def test_usable_predicate(self, stub, final, expected):
        stub.script["final"] = final
        client = ReplanClient(url(stub))
        client.request_async({}, deadline_s=10)
        assert drain(client).usable is expected


class TestFailurePaths:
    def test_deadline_cancels_and_reports_timeout(self, stub):
        """The client must give up on its own, not wait on the service."""
        stub.script.update(running_polls=10_000,  # never finishes
                           final={"status": "ok", "plan": PLAN, "valid": True})
        client = ReplanClient(url(stub))
        started = time.monotonic()
        client.request_async({}, deadline_s=2.0)
        result = drain(client, timeout_s=15)
        assert result["status"] == "timeout"
        assert time.monotonic() - started < 10, "deadline was not enforced"
        assert stub.cancelled, "client must tell the service to stop working"

    def test_unreachable_service_reports_error_not_hang(self, stub):
        """A dead service must produce a result so the node resumes its route."""
        client = ReplanClient("http://127.0.0.1:1", http_timeout_s=2.0)
        client.request_async({}, deadline_s=5)
        result = drain(client, timeout_s=15)
        assert result["status"] == "error"
        assert not result.usable
        assert result["error"]

    def test_http_error_on_submit_is_reported(self, stub):
        stub.script["reject_post"] = True
        client = ReplanClient(url(stub))
        client.request_async({}, deadline_s=5)
        result = drain(client, timeout_s=15)
        assert result["status"] == "error"
        assert not result.usable

    def test_health_on_dead_service_returns_none(self):
        assert ReplanClient("http://127.0.0.1:1").health(timeout_s=1.0) is None


class TestConcurrency:
    def test_only_one_request_in_flight(self, stub):
        stub.script.update(running_polls=5,
                           final={"status": "ok", "plan": PLAN, "valid": True})
        client = ReplanClient(url(stub))
        assert client.request_async({}, deadline_s=20) is True
        assert client.request_async({}, deadline_s=20) is False, "second must be refused"
        drain(client, timeout_s=20)
        assert client.request_async({}, deadline_s=20) is True, "reusable once idle"
        drain(client, timeout_s=20)

    def test_payload_is_snapshotted_not_referenced(self, stub):
        """The worker must not observe node state mutated after the call."""
        stub.script.update(running_polls=3,
                           final={"status": "ok", "plan": PLAN, "valid": True})
        client = ReplanClient(url(stub))
        payload = {"blocked_regions": ["r10"]}
        client.request_async(payload, deadline_s=20)
        payload["blocked_regions"].append("r11")  # mutate after submitting
        drain(client, timeout_s=20)
        assert stub.last_payload["blocked_regions"] == ["r10"], \
            "worker saw a mutation made after the snapshot"

    def test_busy_reflects_worker_state(self, stub):
        stub.script["final"] = {"status": "ok", "plan": PLAN, "valid": True}
        client = ReplanClient(url(stub))
        assert not client.busy()
        client.request_async({}, deadline_s=10)
        drain(client)
        time.sleep(0.1)
        assert not client.busy()
