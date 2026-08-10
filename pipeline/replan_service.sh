#!/usr/bin/env bash
# Manage the host-side EvoPlan replan service.
#
#   pipeline/replan_service.sh start [--mock-plan FILE --mock-delay-s N ...]
#   pipeline/replan_service.sh stop | status | health | log
#
# Runs on the HOST, not in the ROS container: it needs Python 3.11+, openevolve
# and VAL, none of which belong in the Humble image. The container reaches it on
# 127.0.0.1 because docker-compose.yml sets network_mode: host.
#
# Long-lived on purpose -- mission library, graph and VAL discovery are paid
# once at boot rather than per replan.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${REPLAN_PORT:-8077}"
PIDFILE="${REPLAN_PIDFILE:-/tmp/evoplan_replan.pid}"
LOGFILE="${REPLAN_LOG:-/tmp/evoplan_replan.log}"
PYTHON="${REPLAN_PYTHON:-$REPO/.venv-evoplan/bin/python}"
SERVER="$REPO/pipeline/evoplan_bridge_host/server.py"

usage() { sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 2; }

running() {
  [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null
}

cmd_start() {
  if running; then
    echo "replan service already running (pid $(cat "$PIDFILE"), port $PORT)"
    return 0
  fi
  [ -x "$PYTHON" ] || { echo "ERROR: no interpreter at $PYTHON (create .venv-evoplan)" >&2; exit 1; }
  [ -f "$SERVER" ] || { echo "ERROR: no server at $SERVER" >&2; exit 1; }

  # VAL ships in-repo so no sudo install is needed; evaluator.py and planners.py
  # both find it via PATH.
  export PATH="$REPO/.local/bin:$PATH"

  if [ -z "${OPENAI_API_KEY:-}" ]; then
    echo "WARNING: OPENAI_API_KEY is unset -- planner_mode=evoplan will fall back to fd" >&2
  fi

  nohup "$PYTHON" "$SERVER" --port "$PORT" "$@" >>"$LOGFILE" 2>&1 &
  echo $! > "$PIDFILE"

  for _ in $(seq 1 30); do
    if curl -sf "http://127.0.0.1:$PORT/health" >/dev/null 2>&1; then
      echo "replan service up on port $PORT (pid $(cat "$PIDFILE"), log $LOGFILE)"
      return 0
    fi
    sleep 0.5
  done
  echo "ERROR: service did not become healthy within 15s; last log lines:" >&2
  tail -20 "$LOGFILE" >&2
  exit 1
}

cmd_stop() {
  if ! running; then echo "replan service not running"; rm -f "$PIDFILE"; return 0; fi
  local pid; pid="$(cat "$PIDFILE")"
  kill "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.25; done
  kill -9 "$pid" 2>/dev/null || true
  rm -f "$PIDFILE"
  echo "replan service stopped"
}

cmd_status() {
  if running; then echo "running (pid $(cat "$PIDFILE"), port $PORT)"; else echo "stopped"; return 1; fi
}

cmd_health() {
  curl -sf "http://127.0.0.1:$PORT/health" || { echo "unhealthy" >&2; return 1; }
  echo
}

case "${1:-}" in
  start)  shift; cmd_start "$@" ;;
  stop)   cmd_stop ;;
  status) cmd_status ;;
  health) cmd_health ;;
  log)    tail -f "$LOGFILE" ;;
  *)      usage ;;
esac
