#!/usr/bin/env bash
# Tear down a trial and prove it is gone.
#
# Ctrl-C on run_trial.sh does not do this. Two reasons:
#
#   1. The sim runs INSIDE the container. Killing the host-side `docker exec`
#      client detaches from it, it does not signal anything in the container --
#      Gazebo, Nav2 and the executor keep running, holding the GPU and the DDS
#      ports, and the next trial then races a half-dead stack.
#   2. run_trial.sh starts the container with `docker compose run -d` and traps
#      only EXIT. An interrupt that kills the shell hard enough skips the trap
#      and leaves the container up.
#
# So teardown has to remove the CONTAINER, not the client. Everything inside
# dies with it. This script does that, then re-checks and exits non-zero if
# anything survived, because "I ran the teardown" is not the same claim as
# "nothing is running".
#
# Usage:
#   ./pipeline/teardown.sh                # tear down everything
#   ./pipeline/teardown.sh --keep-service # leave the replan service on :8077
#   ./pipeline/teardown.sh --verify       # check only, change nothing
#
# Exit: 0 clean, 1 something survived, 2 bad usage.

set -uo pipefail   # deliberately NOT -e: teardown must run every step even
                   # when an earlier one finds nothing to kill.

CONT="${CONT:-evoplan-trial}"
PORT="${REPLAN_PORT:-8077}"
PIDFILE="${REPLAN_PIDFILE:-/tmp/evoplan_replan.pid}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

KEEP_SERVICE=0
VERIFY_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --keep-service) KEEP_SERVICE=1 ;;
    --verify)       VERIFY_ONLY=1 ;;
    -h|--help)      sed -n '2,28p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "teardown: unknown argument '$arg'" >&2; exit 2 ;;
  esac
done

say()  { printf '[teardown] %s\n' "$*"; }
ok()   { printf '  \033[32mok\033[0m   %s\n' "$*"; }
bad()  { printf '  \033[31mLEFT\033[0m %s\n' "$*"; }

# pgrep -f matches this script and the shell that launched it whenever the
# pattern appears in our own argv. Excluding both stops teardown killing itself
# half way through and leaving the rest of the stack up.
matching_pids() {
  pgrep -f -- "$1" 2>/dev/null | grep -vx "$$" | grep -vx "${PPID:-0}" || true
}

# TERM first so Python/ROS nodes get to flush their logs; KILL only what ignores
# it. A trial that dies mid-write leaves a truncated observations.jsonl.
kill_pattern() {
  local label="$1" pat="$2" pids
  pids="$(matching_pids "$pat")"
  if [ -z "$pids" ]; then ok "$label: none"; return 0; fi
  say "$label: signalling $(echo "$pids" | tr '\n' ' ')"
  # shellcheck disable=SC2086
  kill -TERM $pids 2>/dev/null
  for _ in $(seq 1 20); do
    pids="$(matching_pids "$pat")"
    [ -z "$pids" ] && break
    sleep 0.25
  done
  pids="$(matching_pids "$pat")"
  if [ -n "$pids" ]; then
    # shellcheck disable=SC2086
    kill -KILL $pids 2>/dev/null
    sleep 1
  fi
  [ -z "$(matching_pids "$pat")" ] && ok "$label: gone" || bad "$label: still $(matching_pids "$pat" | tr '\n' ' ')"
}

remove_container() {
  if ! docker container inspect "$CONT" >/dev/null 2>&1; then
    ok "container '$CONT': not present"
    return 0
  fi
  say "container '$CONT': removing (this kills gazebo/nav2/executor inside)"
  docker rm -f "$CONT" >/dev/null 2>&1
  for _ in $(seq 1 60); do
    docker container inspect "$CONT" >/dev/null 2>&1 || { ok "container '$CONT': gone"; return 0; }
    sleep 0.5
  done
  bad "container '$CONT': still present after 30s"
}

# ---------------------------------------------------------------- verification
# Reused as the post-teardown gate, and as the whole job under --verify.
verify() {
  local fail=0 n pids

  n="$(docker ps -q 2>/dev/null | wc -l | tr -d ' ')"
  if [ "$n" = "0" ]; then ok "containers: 0"; else
    bad "containers: $n"; docker ps --format '         {{.Names}}  {{.Status}}' 2>/dev/null; fail=1
  fi

  # Host-side leftovers. Scoped to this repo / this container so a teardown
  # never reaches into somebody else's unrelated ros2 or python process.
  for spec in \
      "trial driver|run_trial\.sh" \
      "sim driver|run_sim\.sh" \
      "docker exec client|docker exec .*${CONT}" \
      "gazebo|ign gazebo" \
      "openevolve|openevolve-run\.py" \
      "replan service|evoplan_bridge_host/server\.py"; do
    local label="${spec%%|*}" pat="${spec#*|}"
    [ "$KEEP_SERVICE" = "1" ] && [ "$label" = "replan service" ] && continue
    pids="$(matching_pids "$pat")"
    if [ -z "$pids" ]; then ok "$label: 0"; else
      bad "$label: $(echo "$pids" | tr '\n' ' ')"
      # shellcheck disable=SC2086
      ps -o pid=,etime=,args= -p $pids 2>/dev/null | cut -c1-150 | sed 's/^/         /'
      fail=1
    fi
  done

  if [ "$KEEP_SERVICE" = "1" ]; then
    ok "port $PORT: not checked (--keep-service)"
  elif ss -ltn 2>/dev/null | grep -q ":${PORT} "; then
    bad "port $PORT: still listening"; fail=1
  else
    ok "port $PORT: free"
  fi

  return $fail
}

# --------------------------------------------------------------------- run it
if [ "$VERIFY_ONLY" = "1" ]; then
  say "verify only, changing nothing"
  verify && { say "CLEAN"; exit 0; } || { say "NOT CLEAN"; exit 1; }
fi

say "tearing down (container '$CONT', port $PORT)"

# Order matters: stop the drivers before removing the container, or run_trial.sh
# reaches its next step and starts something new against a container we just
# deleted.
kill_pattern "trial driver"      'run_trial\.sh'
kill_pattern "docker exec client" "docker exec .*${CONT}"
remove_container

# Anything that escaped the container boundary, or a bare ./run_sim.sh run
# straight on the host.
kill_pattern "sim driver" 'run_sim\.sh'
kill_pattern "gazebo"     'ign gazebo'

if [ "$KEEP_SERVICE" = "1" ]; then
  say "leaving the replan service up (--keep-service)"
else
  # The pidfile is the service's own record; the pattern sweep catches the
  # openevolve children it spawned, which outlive it and keep burning CPU.
  if [ -f "$PIDFILE" ]; then
    pid="$(cat "$PIDFILE" 2>/dev/null || true)"
    [ -n "${pid:-}" ] && kill -TERM "$pid" 2>/dev/null
    sleep 1
    [ -n "${pid:-}" ] && kill -KILL "$pid" 2>/dev/null
    rm -f "$PIDFILE"
  fi
  kill_pattern "replan service" 'evoplan_bridge_host/server\.py'
  kill_pattern "openevolve"     'openevolve-run\.py'
fi

say "verifying"
sleep 2
if verify; then
  say "CLEAN"
  exit 0
fi
say "NOT CLEAN -- rerun, or kill the PIDs listed above by hand"
exit 1
