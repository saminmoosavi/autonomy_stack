#!/usr/bin/env bash
# Find-an-object trials, run back to back on a fixed clock.
#
#   for each mission in PLAN:
#     for ITERATIONS trials:
#       launch run_trial.sh in the background
#       wait TRIAL_WINDOW (15 min)
#       teardown.sh
#       wait SETTLE (1 min)
#
# Default plan: factory_tour_03 five times, then factory_tour_02 five times.
# Ten trials at 16 minutes each is about two hours forty; see WAIT_FOR_EXIT
# below for the shorter version.
#
# The window is a WALL-CLOCK BUDGET, not an expected duration. A healthy tour_03
# finishes in about eight minutes and tour_02 rather longer -- five regions
# instead of three -- so a trial normally exits on its own well before the
# window closes and teardown is then just a sweep. When a run wedges (bringup
# stalls, the executor dies, Nav2 never converges) the window is what stops the
# batch waiting on it. That is why each trial is backgrounded and timed rather
# than simply run in the foreground: a foreground hang would take the whole
# batch with it.
#
# Teardown between every iteration is not optional. Trials share fixed paths --
# the replan service on :8077, the evoplan-trial container, ~/clearpath, and
# /dev/shm/fastrtps_* -- and a hard-killed run leaks DDS shared-memory segments
# that make the NEXT run's AMCL stall and its robot spawn time out. Those
# failures look like a broken mission rather than dirty state, which is what
# makes them expensive.
#
# Usage:
#   ./pipeline/run_batch.sh                       # 5 x tour_03, then 5 x tour_02
#   ITERATIONS=2 ./pipeline/run_batch.sh          # 2 of each
#   WAIT_FOR_EXIT=1 ./pipeline/run_batch.sh       # move on as soon as a trial exits
#   MISSIONS=factory_tour_02 ./pipeline/run_batch.sh   # just one of them
#
# Ctrl-C stops the batch and tears down the trial that is running.

set -uo pipefail        # NOT -e: one bad trial must not abort the batch

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

ITERATIONS="${ITERATIONS:-5}"
TRIAL_WINDOW="${TRIAL_WINDOW:-900}"     # 15 min
SETTLE="${SETTLE:-60}"                  # 1 min
WAIT_FOR_EXIT="${WAIT_FOR_EXIT:-0}"     # 1 = proceed the moment the trial exits
CROWD="${CROWD:-0}"
PLANNER_MODE="${PLANNER_MODE:-evoplan}"
SYMBOLIC_DEADLINE="${SYMBOLIC_DEADLINE:-150}"

# mission | FIND_OBJECT | YOLO_CLASSES
#
# FIND_OBJECT names the object the EXECUTOR drives to, and it can only drive to
# one: find_object_class is a single parameter and begin_object_approach sets a
# single target_region. tour_02's PDDL hunts both the cone and the skateboard,
# and the replan service builds a problem naming both -- but the robot will
# approach the cone and stop. The skateboard is still detected, localised and
# spliced into the problem file, because YOLO_CLASSES carries it; that is what
# makes the two-object plumbing observable in a run that cannot yet drive it.
#
# The cone is the one that drives because it is a real Fuel asset that YOLO
# reliably labels "traffic cone". skateboard_r3 is built from primitives and
# may not be labelled "skateboard" at all -- check observations.jsonl before
# promoting it to FIND_OBJECT.
PLAN=(
  "factory_tour_03|traffic cone|person,traffic cone,chair,suitcase,banana"
  "factory_tour_02|traffic cone|person,traffic cone,skateboard,chair,suitcase,banana"
)
# MISSIONS=a,b filters the plan to those missions, in the plan's order.
if [ -n "${MISSIONS:-}" ]; then
  FILTERED=()
  for entry in "${PLAN[@]}"; do
    case ",${MISSIONS}," in *",${entry%%|*},"*) FILTERED+=("$entry") ;; esac
  done
  [ "${#FILTERED[@]}" -gt 0 ] || { echo "no plan entry matches MISSIONS=$MISSIONS" >&2; exit 2; }
  PLAN=("${FILTERED[@]}")
fi

BATCH_TAG="batch_$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="$REPO/results/batches/$BATCH_TAG"
mkdir -p "$BATCH_DIR"

export CROWD PLANNER_MODE SYMBOLIC_DEADLINE

say() { printf '\n[batch] %s\n' "$*"; }

TRIAL_PID=""
cleanup() {
  say "interrupted -- tearing down"
  [ -n "$TRIAL_PID" ] && kill "$TRIAL_PID" 2>/dev/null
  ./pipeline/teardown.sh >>"$BATCH_DIR/teardown.log" 2>&1
  exit 130
}
trap cleanup INT TERM

TOTAL=$(( ${#PLAN[@]} * ITERATIONS ))
say "$TOTAL trials: ${#PLAN[@]} mission(s) x $ITERATIONS"
say "window ${TRIAL_WINDOW}s + settle ${SETTLE}s  =>  up to $(( TOTAL * (TRIAL_WINDOW + SETTLE) / 60 )) min"
say "crowd=$CROWD planner=$PLANNER_MODE deadline=$SYMBOLIC_DEADLINE"
for entry in "${PLAN[@]}"; do say "  plan: ${entry%%|*}  find='$(echo "$entry" | cut -d'|' -f2)'"; done
say "logs -> $BATCH_DIR"

N=0
for entry in "${PLAN[@]}"; do
  MISSION="${entry%%|*}"
  FIND_OBJECT="$(echo "$entry" | cut -d'|' -f2)"
  YOLO_CLASSES="$(echo "$entry" | cut -d'|' -f3)"
  export MISSION FIND_OBJECT YOLO_CLASSES

  say "########## $MISSION x $ITERATIONS ##########"
  echo "== $MISSION (find='$FIND_OBJECT')" >>"$BATCH_DIR/summary.txt"

  for i in $(seq 1 "$ITERATIONS"); do
    N=$(( N + 1 ))
    LOG="$BATCH_DIR/${MISSION}_${i}.log"
    say "=== trial $N/$TOTAL  $MISSION $i/$ITERATIONS  $(date '+%F %T') -> $(basename "$LOG")"

    ./pipeline/run_trial.sh >"$LOG" 2>&1 &
    TRIAL_PID=$!

    if [ "$WAIT_FOR_EXIT" = "1" ]; then
      # Poll rather than `wait`, so the window still applies as a hard cap.
      for _ in $(seq 1 "$TRIAL_WINDOW"); do
        kill -0 "$TRIAL_PID" 2>/dev/null || break
        sleep 1
      done
    else
      sleep "$TRIAL_WINDOW"
    fi

    if kill -0 "$TRIAL_PID" 2>/dev/null; then
      say "still running at the window; stopping it"
      kill "$TRIAL_PID" 2>/dev/null
    else
      wait "$TRIAL_PID"; RC=$?
      say "exited rc=$RC"
    fi
    TRIAL_PID=""

    # The metrics file is <mission>_p<crowd>_<mode>_<YYYYmmdd_HHMMSS>.json, and
    # a bare *.json glob also matches its siblings -- _belief.json,
    # _evo_log.json, _replan_N.json -- several of which are written LATER, so
    # `ls -t | head -1` would report a replan artifact as the trial result.
    # Anchor on the timestamp instead.
    RESULT="$(ls -t results/single_trials/${MISSION}_p${CROWD}_${PLANNER_MODE}_*.json 2>/dev/null \
              | grep -E "_[0-9]{8}_[0-9]{6}\.json$" | head -1)"
    if [ -n "$RESULT" ]; then
      python3 - "$RESULT" <<'PY' | tee -a "$BATCH_DIR/summary.txt"
import json, sys, pathlib
d = json.load(open(sys.argv[1]))
keys = ("success", "collisions", "duration_s", "path_length_m",
        "mission_complete", "symbolic_replans")
print("  " + pathlib.Path(sys.argv[1]).name,
      {k: d[k] for k in keys if k in d})
PY
    else
      say "no metrics json"
      echo "  $MISSION $i: NO RESULT" >>"$BATCH_DIR/summary.txt"
    fi

    say "teardown"
    ./pipeline/teardown.sh >>"$BATCH_DIR/teardown.log" 2>&1
    TD=$?
    [ "$TD" -eq 0 ] || say "WARNING teardown exited $TD -- see $BATCH_DIR/teardown.log"

    if [ "$N" -lt "$TOTAL" ]; then
      say "settling ${SETTLE}s"
      sleep "$SETTLE"
    fi
  done
done

say "batch done"
echo
cat "$BATCH_DIR/summary.txt" 2>/dev/null
echo
say "artifacts: results/single_trials/  batch logs: $BATCH_DIR"
