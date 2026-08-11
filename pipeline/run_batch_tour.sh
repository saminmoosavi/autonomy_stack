#!/usr/bin/env bash
# Open-world find-and-inspect trials, run back to back on a fixed clock.
#
#   for ITERATIONS trials:
#     launch run_trial.sh in the background
#     wait for it to exit, or TRIAL_WINDOW, whichever comes first
#     teardown.sh
#     wait SETTLE
#
# Sibling of run_batch.sh, which batches find-AN-OBJECT missions. The mission
# style is what differs, and it changes what a run means: there the object is
# named up front and the question is whether the robot reaches it; here the
# object SET is unknown, and the questions are how many the robot finds, whether
# it finds the same number every time, and whether it inspects all of them. So
# this script reports discovered/inspected counts rather than a bare success
# flag, and aggregates the count DISTRIBUTION across the batch -- a mission that
# finds 2 objects four times and 1 object once has told you something that
# "4/5 succeeded" would hide.
#
# Usage:
#   ./pipeline/run_batch_tour.sh                        # 5 x factory_survey_02
#   ITERATIONS=3 ./pipeline/run_batch_tour.sh           # 3 of them
#   MISSION=factory_survey_01 ./pipeline/run_batch_tour.sh
#   EXPECT_OBJECTS=2 ./pipeline/run_batch_tour.sh       # also grade each trial
#   PLANNER_MODE=evoplan ./pipeline/run_batch_tour.sh   # keep FD as a fallback
#
# Ctrl-C stops the batch and tears down the trial that is running.

set -uo pipefail        # NOT -e: one bad trial must not abort the batch

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

MISSION="${MISSION:-factory_survey_02}"
ITERATIONS="${ITERATIONS:-5}"
CROWD="${CROWD:-0}"
# evoplan_only by default: this mission style exists to be planned by EvoPlan.
# The objects do not exist as PDDL symbols until perception finds them, so the
# inspections can only ever be REPLANNED -- and evoplan_only is the mode that
# measures what EvoPlan does unaided, with no Fast Downward plan to fall back
# on. PLANNER_MODE=evoplan keeps that fallback if you want the comparison.
PLANNER_MODE="${PLANNER_MODE:-evoplan_only}"
SYMBOLIC_DEADLINE="${SYMBOLIC_DEADLINE:-150}"
# The classes to inspect, and the detector vocabulary that must contain them.
# INSPECT_OBJECTS is the ONLY place the target class is named: the mission PDDL
# is deliberately class-agnostic, so the same problem file serves cones, chairs
# or anything else the detector can label.
INSPECT_OBJECTS="${INSPECT_OBJECTS:-traffic cone}"
YOLO_CLASSES="${YOLO_CLASSES:-person,traffic cone,chair,suitcase,banana}"
# Wall-clock budget per trial, not an expected duration. A healthy survey_02 run
# measured ~17 min end to end: ~3 min bringup, ~8 min survey, a ~100 s
# deliberation hold for the inspection replan, then the inspection legs and two
# 5 s dwells. 25 min leaves room for a slower LLM round trip without letting a
# wedged run hold the batch. run_batch.sh's 900 s would have killed the
# validated run outright.
TRIAL_WINDOW="${TRIAL_WINDOW:-1500}"
SETTLE="${SETTLE:-60}"
# 1 = move on the moment the trial exits. Default ON here, unlike run_batch.sh:
# these missions terminate themselves on mission_complete, so waiting out the
# full window would add ~40 min of nothing to a five-trial batch.
WAIT_FOR_EXIT="${WAIT_FOR_EXIT:-1}"
# Sim-time cap inside the run, a backstop for a mission that neither completes
# nor exhausts. TRIAL_WINDOW is the wall-clock backstop for everything else.
MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S:-2700}"
# Optional: how many objects the world actually contains, used ONLY to grade
# the summary. Ground truth is legitimate in the harness -- it is the mission
# PDDL, which is the seed handed to the LLM, that must never state it. Leave
# unset to report the observed distribution without judging it.
EXPECT_OBJECTS="${EXPECT_OBJECTS:-}"

export CROWD PLANNER_MODE SYMBOLIC_DEADLINE MISSION INSPECT_OBJECTS \
       YOLO_CLASSES MISSION_TIMEOUT_S

BATCH_TAG="tour_$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="$REPO/results/batches/$BATCH_TAG"
mkdir -p "$BATCH_DIR"
RESULTS_JSONL="$BATCH_DIR/results.jsonl"

say() { printf '\n[batch] %s\n' "$*"; }
die() { printf '\n[batch] ERROR %s\n' "$*" >&2; exit 2; }

TRIAL_PID=""
cleanup() {
  say "interrupted -- tearing down"
  [ -n "$TRIAL_PID" ] && kill "$TRIAL_PID" 2>/dev/null
  ./pipeline/teardown.sh >>"$BATCH_DIR/teardown.log" 2>&1
  exit 130
}
trap cleanup INT TERM

# ---------------------------------- preflight --------------------------------
# Checked once, here, rather than discovered five times. run_trial.sh makes the
# same checks per trial, but a batch that fails every iteration for one missing
# host-side prerequisite wastes an hour before saying so.
#
# Load the env file FIRST, by the same mechanism and from the same path
# run_trial.sh uses (run_trial.sh:34-36). Without this the checks below run
# against the batch's own environment, which in a plain shell has no key --
# so the batch refused to start while every trial it would have launched would
# have found the key perfectly well. A preflight must not be stricter than the
# thing it is standing in for.
_ENV_FILE="$REPO/pipeline/trial.env"
# shellcheck disable=SC1090
[ -f "$_ENV_FILE" ] && set -a && . "$_ENV_FILE" && set +a
[ -f "pipeline/missions/${MISSION}.pddl" ] \
  || die "no PDDL problem at pipeline/missions/${MISSION}.pddl (MISSION=$MISSION)"
[ -f "factory_mission_plans/${MISSION}.txt" ] \
  || die "no survey plan at factory_mission_plans/${MISSION}.txt (MISSION=$MISSION)"
case ",${YOLO_CLASSES}," in
  *",${INSPECT_OBJECTS},"*) ;;
  *) say "WARNING '$INSPECT_OBJECTS' is not in YOLO_CLASSES; nothing can be detected" ;;
esac
if [ "$PLANNER_MODE" != "fd_only" ]; then
  [ -n "${OPENAI_API_KEY:-}" ] \
    || die "PLANNER_MODE=$PLANNER_MODE needs OPENAI_API_KEY: not exported, and not found in $_ENV_FILE"
  # In evoplan_only the service IS the planner -- there is no FD fallback, so an
  # unreachable service does not degrade the batch, it empties it.
  curl -sf "http://127.0.0.1:${REPLAN_PORT:-8077}/health" >/dev/null \
    || die "replan service is not answering on :${REPLAN_PORT:-8077}; start it with pipeline/replan_service.sh start"
fi

say "$ITERATIONS x $MISSION"
say "planner=$PLANNER_MODE deadline=${SYMBOLIC_DEADLINE}s crowd=$CROWD"
say "inspect='$INSPECT_OBJECTS'  (count is NOT given to the robot)"
say "window ${TRIAL_WINDOW}s + settle ${SETTLE}s  =>  up to $(( ITERATIONS * (TRIAL_WINDOW + SETTLE) / 60 )) min"
say "logs -> $BATCH_DIR"

# ---------------------------------- the batch --------------------------------
for i in $(seq 1 "$ITERATIONS"); do
  LOG="$BATCH_DIR/${MISSION}_${i}.log"
  # Tag carries the iteration so an artifact can be traced back to its trial,
  # while still ending in _YYYYmmdd_HHMMSS so the result-file match below stays
  # anchored on the timestamp.
  TAG="i${i}_$(date +%Y%m%d_%H%M%S)"
  export TAG
  say "=== trial $i/$ITERATIONS  $(date '+%F %T')  tag=$TAG -> $(basename "$LOG")"

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

  # The metrics file is <mission>_p<crowd>_<mode>_<tag>.json, and a bare *.json
  # glob also matches its siblings -- _belief.json, _evo_log.json,
  # _replan_N.json -- several of which are written LATER, so `ls -t | head -1`
  # would report a replan artifact as the trial result. Anchor on the tag.
  RESULT="results/single_trials/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}.json"
  if [ -f "$RESULT" ]; then
    python3 - "$RESULT" "$i" "$RESULTS_JSONL" <<'PY'
import json, pathlib, sys

path, trial, jsonl = sys.argv[1], int(sys.argv[2]), sys.argv[3]
d = json.load(open(path))
evo = d.get("evoplan") or {}
# The inspection summary rides in the status blob, which scand_metrics nests
# whole under "evoplan" -- it is NOT a top-level key of the metrics json.
insp = evo.get("inspection") or {}
objects = insp.get("objects") or []
row = {
    "trial": trial,
    "file": pathlib.Path(path).name,
    "mission_complete": d.get("mission_complete"),
    "discovered": insp.get("total"),
    "inspected": insp.get("inspected"),
    "pending": insp.get("pending") or [],
    "by_class": insp.get("by_class") or {},
    "regions": [o.get("region") for o in objects],
    "phase": evo.get("inspect_phase"),
    "duration_s": d.get("duration_s"),
    "moving_time_s": d.get("moving_time_s"),
    "held_time_s": d.get("held_time_s"),
    "path_length_m": d.get("path_length_m"),
    "collisions": d.get("collisions"),
    "symbolic_replans": d.get("symbolic_replans"),
    "regions_remaining": d.get("regions_remaining"),
}
with open(jsonl, "a") as fh:
    fh.write(json.dumps(row) + "\n")

disc, done = row["discovered"], row["inspected"]
print(f"  trial {trial}: complete={row['mission_complete']} "
      f"discovered={disc} inspected={done} "
      f"regions={row['regions']} "
      f"duration={row['duration_s']}s collisions={row['collisions']} "
      f"replans={row['symbolic_replans']}")
if row["pending"]:
    print(f"    NOT inspected: {row['pending']}")
PY
  else
    say "no metrics json at $RESULT"
    python3 - "$i" "$RESULTS_JSONL" <<'PY'
import json, sys
with open(sys.argv[2], "a") as fh:
    fh.write(json.dumps({"trial": int(sys.argv[1]), "file": None,
                         "mission_complete": None, "discovered": None,
                         "inspected": None, "pending": []}) + "\n")
PY
  fi

  say "teardown"
  ./pipeline/teardown.sh >>"$BATCH_DIR/teardown.log" 2>&1
  TD=$?
  [ "$TD" -eq 0 ] || say "WARNING teardown exited $TD -- see $BATCH_DIR/teardown.log"

  if [ "$i" -lt "$ITERATIONS" ]; then
    say "settling ${SETTLE}s"
    sleep "$SETTLE"
  fi
done

# --------------------------------- aggregate ---------------------------------
say "batch done"
echo
python3 - "$RESULTS_JSONL" "${EXPECT_OBJECTS:-}" <<'PY' | tee "$BATCH_DIR/summary.txt"
import collections, json, statistics, sys

rows = []
try:
    with open(sys.argv[1]) as fh:
        rows = [json.loads(l) for l in fh if l.strip()]
except OSError:
    pass
if not rows:
    print("no results"); raise SystemExit

expect = sys.argv[2]
expect = int(expect) if expect.strip() else None
n = len(rows)
ran = [r for r in rows if r.get("discovered") is not None]

print(f"{n} trial(s), {len(ran)} produced metrics\n")
for r in rows:
    mark = "  "
    if expect is not None and r.get("discovered") is not None:
        mark = "ok" if (r["discovered"] == expect
                        and r["inspected"] == expect) else "XX"
    print(f" {mark} trial {r['trial']}: complete={r.get('mission_complete')} "
          f"discovered={r.get('discovered')} inspected={r.get('inspected')} "
          f"regions={r.get('regions')} "
          f"duration={r.get('duration_s')}s")

if not ran:
    raise SystemExit

# The headline for an unknown-quantity mission: does it find the same number
# every time? A mean would hide a bimodal split, so report the distribution.
counts = collections.Counter(r["discovered"] for r in ran)
print("\ndiscovered-count distribution: "
      + ", ".join(f"{k} object(s) x{v}" for k, v in sorted(counts.items())))
regions = collections.Counter(g for r in ran for g in (r.get("regions") or []))
print("objects found per region:      "
      + ", ".join(f"{k}:{v}" for k, v in sorted(regions.items())))

all_done = [r for r in ran if r["discovered"] and r["discovered"] == r["inspected"]]
print(f"\ninspected everything found:    {len(all_done)}/{len(ran)}")
print(f"mission_complete:              "
      f"{sum(1 for r in ran if r.get('mission_complete'))}/{len(ran)}")
print(f"zero collisions:               "
      f"{sum(1 for r in ran if r.get('collisions') == 0)}/{len(ran)}")
if expect is not None:
    hit = sum(1 for r in ran if r["discovered"] == expect and r["inspected"] == expect)
    print(f"found AND inspected all {expect}:      {hit}/{len(ran)}")

durations = [r["duration_s"] for r in ran if r.get("duration_s")]
if durations:
    print(f"\nduration_s: min={min(durations):.0f} "
          f"median={statistics.median(durations):.0f} max={max(durations):.0f}")
held = [r["held_time_s"] for r in ran if r.get("held_time_s") is not None]
if held:
    print(f"held_time_s (deliberation): min={min(held):.0f} "
          f"median={statistics.median(held):.0f} max={max(held):.0f}")
PY
echo
say "artifacts: results/single_trials/  batch logs: $BATCH_DIR"
