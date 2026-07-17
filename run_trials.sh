#!/usr/bin/env bash
# run_trials.sh — factory-mission batch: run EVERY mission plan under
# factory_mission_plans/ TRIALS times in the populated warehouse
# (worlds/warehouse_people.sdf, 16 actors), following the manual bringup
# sequence of process.md (AMCL on factory_sim_map + nav2_custom + 4-cam YOLO
# + trackers). Each attempt is one full bringup + trial, automated inside the
# ros_humble docker container by density_sweep.py (WORLD_NAME override).
#
# Each attempt spawns the robot at a noisy start position: x ~ 0±1 m,
# y ~ 1±0.5 m (uniform, re-drawn per attempt; recorded in the attempt's
# spawn.txt). The AMCL initial pose is set to the same point.
#
# Logging policy (unlike run_experiments.sh, stuck runs COUNT): an attempt is
# logged whenever scand_metrics wrote a summary — even if the robot got stuck
# and the backstop expired below 100% route, so Route % in the table reflects
# real completion rates. Only attempts with no metrics at all (bringup
# failure/timeout) are discarded and re-run in a fresh container, with one
# exception: a run reporting 100% route but with a skipped checkpoint is
# retried, so a logged 100% always means the full route was executed.
# Resumable: trials with a logged metrics file are skipped on rerun.
#
# Plan files must contain plain-text PDDL lines ("(move ...)" etc.); files
# that don't parse (e.g. a stray JSON log saved as a plan) are SKIPPED with a
# warning — restore the file and rerun to fill in that table row.
#
# Run on the HOST. Results -> results/factory_trials/plan<p>_trial<r>_metrics.json
# (+ _evo_log.json); per-attempt bringup/sim logs under
# results/factory_trials/_runs/plan<p>_trial<r>_attempt<a>/.
# Final table: gen_trials_table.py -> results/factory_trials/results_table.tex
# (one row per plan, mean±std over trials, sample_table.tex format).
#
# Env: TRIALS (5), BACKSTOP (900 s per-attempt sim-time metrics cap),
#      MAX_ATTEMPTS (20 attempts per trial), BRINGUP_BUDGET (1200 s),
#      WALL_FACTOR (2.2 wall/sim ratio), CONT (evo_exp), WORLD (warehouse_people),
#      OUT (results dir, must be under the repo so the container sees it).
set -uo pipefail

HOSTREPO=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CREPO=/home/user/autonomy_stack_ros_humble   # repo mount point inside container
COMPOSE=$HOSTREPO/docker-compose.yml
CONT=${CONT:-evo_exp}

TRIALS=${TRIALS:-5}
WORLD=${WORLD:-warehouse_people}             # worlds/<WORLD>.sdf
BACKSTOP=${BACKSTOP:-900}
MAX_ATTEMPTS=${MAX_ATTEMPTS:-20}
BRINGUP_BUDGET=${BRINGUP_BUDGET:-1200}       # wall-clock allowance beyond BACKSTOP
WALL_FACTOR=${WALL_FACTOR:-2.2}              # sim seconds -> wall seconds (RTF < 1)
OUT=${OUT:-$HOSTREPO/results/factory_trials}

case "$OUT" in                               # container view of $OUT
  "$HOSTREPO"/*) OUT_C=$CREPO${OUT#"$HOSTREPO"} ;;
  *) echo "[trials] ERROR: OUT=$OUT is not under $HOSTREPO (container can't see it)"; exit 1 ;;
esac
RUNS_H=$OUT/_runs
mkdir -p "$RUNS_H"

[ -s "$HOSTREPO/worlds/${WORLD}.sdf" ] \
  || { echo "[trials] ERROR: worlds/${WORLD}.sdf missing"; exit 1; }

# wall-clock cap for one attempt: sim backstop scaled by RTF + bringup budget
TO_WALL=$(python3 -c "print(int($BACKSTOP * $WALL_FACTOR) + $BRINGUP_BUDGET)")

# discover mission plans; a valid plan has plain-text "(move ...)" lines
PLANS=()
for f in "$HOSTREPO"/factory_mission_plans/factory_mission_*.txt; do
  p=$(basename "$f" .txt); p=${p#factory_mission_}
  if grep -qE '^\(move ' "$f"; then
    PLANS+=("$p")
  else
    echo "[trials] WARNING: $(basename "$f") has no plain-text (move ...) lines (corrupted?) -> SKIPPED"
  fi
done
[ "${#PLANS[@]}" -gt 0 ] || { echo "[trials] ERROR: no valid plans in factory_mission_plans/"; exit 1; }

plan_target() {   # $1 = plan file -> destination of the last (move .. RX RY)
  grep -oE '^\(move +[[:alnum:]_]+ +R[0-9]+ +R[0-9]+\)' "$1" \
    | tail -1 | grep -oE 'R[0-9]+' | tail -1
}

# process.md: the 4-camera robot config must be the active clearpath config
cp "$HOSTREPO/robot_4cam.yaml" "$HOME/clearpath/robot.yaml"

# Gazebo must render on the machine-local X server. Over `ssh -Y` the shell's
# DISPLAY points at the SSH-forwarded display, which the container has no X
# cookie for ("X11 connection rejected because of wrong authentication") and
# which would tunnel GPU camera rendering over the network. Likewise blank out
# XAUTHORITY (an ssh session sets it to a host path that shadows local access).
SIM_DISPLAY=${SIM_DISPLAY:-:1}

activate() {   # docker activate sequence (process.md): compose run the service
  [ "$(docker inspect -f '{{.State.Running}}' "$CONT" 2>/dev/null)" = "true" ] && return 0
  docker rm -f "$CONT" >/dev/null 2>&1
  docker compose -f "$COMPOSE" run -d --rm --name "$CONT" \
    -e DISPLAY="$SIM_DISPLAY" -e XAUTHORITY= ros_humble \
    sleep infinity >/dev/null 2>&1
}

wait_ros() {   # ros2 CLI usable inside the container
  local i
  for i in $(seq 1 40); do
    if docker exec "$CONT" bash -lc \
        'source /opt/ros/humble/setup.bash >/dev/null 2>&1; timeout 8 ros2 topic list >/dev/null 2>&1'; then
      return 0
    fi
    sleep 3
  done
  return 1
}

fresh() {      # clean gz/DDS state: recreate the container (kills its whole cgroup)
  docker rm -f "$CONT" >/dev/null 2>&1
  activate && wait_ros
}

progress_pct() {   # $1 = metrics json -> integer route-completion %, -1 if unreadable
  python3 - "$1" <<'PY' 2>/dev/null || echo -1
import json, sys
d = json.load(open(sys.argv[1]))
p = d.get("path_progress_pct")
print(int(p) if p is not None else -1)
PY
}

checkpoints_ok() {   # $1 = evo log json  $2 = attempt dir
  # every checkpoint must be visited: the final nav2_goal_finished must exist
  # with no missed_waypoints, and no waypoint may have been failed-over
  # ("Failed to process waypoint N, moving to next" in the nav2 log, which
  # stop_on_failure:false emits when a goal is canceled/aborted mid-waypoint)
  python3 - "$1" <<'PY' 2>/dev/null || return 1
import json, sys
d = json.load(open(sys.argv[1]))
fin = [e for e in d.get("events", []) if e.get("event") == "nav2_goal_finished"]
sys.exit(0 if fin and not fin[-1].get("data", {}).get("missed_waypoints") else 1)
PY
  ! grep -aqs "Failed to process waypoint" "$2"/_logs/world*/attempt*/nav2_attempt*.log
}

print_summary() {
  python3 - "$OUT" "$TRIALS" "${PLANS[@]}" <<'PY'
import json, sys, pathlib
out, trials = pathlib.Path(sys.argv[1]), int(sys.argv[2])
plans = sys.argv[3:]
hdr = (f"{'plan':>4} {'trial':>5} {'route%':>7} {'succ':>5} {'time[s]':>8} "
       f"{'path[m]':>8} {'coll':>5} {'minclr[m]':>10}")
print(hdr); print("-" * len(hdr))
for p in plans:
    for r in range(1, trials + 1):
        f = out / f"plan{p}_trial{r}_metrics.json"
        if not f.exists():
            print(f"{p:>4} {r:>5}   -- not logged --")
            continue
        d = json.load(open(f))
        fmt = lambda v, pr=1: "--" if v is None else f"{v:.{pr}f}"
        print(f"{p:>4} {r:>5} {fmt(d.get('path_progress_pct')):>7} "
              f"{'yes' if d.get('success') else 'no':>5} {fmt(d.get('duration_s'), 0):>8} "
              f"{fmt(d.get('path_length_m')):>8} {fmt(d.get('collisions'), 0):>5} "
              f"{fmt(d.get('min_human_clearance_m'), 2):>10}")
PY
  # combined LaTeX table (one row per plan, mean±std over trials)
  python3 "$HOSTREPO/gen_trials_table.py" "$OUT" || true
}

CLEANED=0
cleanup() {   # idempotent: removing the container kills its whole cgroup,
              # i.e. every sim/nav/yolo process the attempt exec'd inside it
  [ "$CLEANED" = 1 ] && return
  CLEANED=1
  docker rm -f "$CONT" >/dev/null 2>&1
}
on_interrupt() {
  trap - INT TERM
  echo
  echo "[trials] interrupted -> tearing down container '$CONT' (in-flight attempt discarded; no partial trial is logged)"
  cleanup
  echo "[trials] stopped after $((SECONDS-START))s. Logged trials remain in $OUT — rerun to resume."
  print_summary
  exit 130
}
START=$SECONDS
trap on_interrupt INT TERM HUP   # HUP: ssh session drop must still tear down
trap cleanup EXIT

echo "[trials] plans [${PLANS[*]}] x $TRIALS trials  world=worlds/${WORLD}.sdf"
echo "[trials] per-attempt backstop=${BACKSTOP}s (sim, ~${TO_WALL}s wall)  max_attempts=$MAX_ATTEMPTS  out=$OUT"

for p in "${PLANS[@]}"; do
  PLAN_H=$HOSTREPO/factory_mission_plans/factory_mission_${p}.txt
  PLAN_C=$CREPO/factory_mission_plans/factory_mission_${p}.txt
  TARGET=$(plan_target "$PLAN_H")
  if [ -z "$TARGET" ]; then
    echo "[trials] plan $p: could not derive goal region -> SKIPPED"
    continue
  fi

for r in $(seq 1 "$TRIALS"); do
  final="$OUT/plan${p}_trial${r}_metrics.json"
  prev=$([ -s "$final" ] && progress_pct "$final" || echo -1)
  if [ "$prev" -ge 0 ]; then
    echo "[trials] plan $p trial $r already logged (route ${prev}%) -> skip"
    continue
  fi

  logged=0
  for a in $(seq 1 "$MAX_ATTEMPTS"); do
    echo "[trials] ===== plan $p (target $TARGET) trial $r attempt $a  t+$((SECONDS-START))s ====="
    if ! fresh; then
      echo "[trials]   container/ROS not ready -> retry"
      continue
    fi

    res_h=$RUNS_H/plan${p}_trial${r}_attempt${a}
    res_c=$OUT_C/_runs/plan${p}_trial${r}_attempt${a}
    rm -rf "$res_h"; mkdir -p "$res_h"

    # noisy start: x ~ 0±1 m, y ~ 1±0.5 m (uniform), fresh draw per attempt;
    # density_sweep uses SPAWN_X/Y for both the Gazebo spawn and the AMCL
    # initial pose, so localization stays consistent with the spawn.
    read -r SX SY <<<"$(python3 -c \
      'import random; print(f"{random.uniform(-1,1):.3f} {random.uniform(0.5,1.5):.3f}")')"
    echo "[trials]   spawn: x=$SX y=$SY"
    echo "$SX $SY" > "$res_h/spawn.txt"

    # one full process.md bringup + trial; metrics_stop_on_success inside
    # density_sweep ends the run the moment the route completes collision-free.
    # WORLD_NAME overrides the density world; WORLDS=0 only names the outputs.
    timeout -k 30 "$TO_WALL" docker exec "$CONT" bash -lc "
      cd $CREPO &&
      WORLDS=0 WORLD_NAME=$WORLD TARGET_REGION=$TARGET PLAN_FILE=$PLAN_C \
      SPAWN_X=$SX SPAWN_Y=$SY WALL_FACTOR=$WALL_FACTOR \
      RESULTS_DIR=$res_c TRIAL_TIMEOUT=$BACKSTOP MAX_ATTEMPTS=3 \
      python3 density_sweep.py
    " > "$res_h/sweep.log" 2>&1
    rc=$?

    mj=$res_h/world0_metrics.json
    pct=$(progress_pct "$mj")
    if [ "$pct" -ge 100 ] && ! checkpoints_ok "$res_h/world0_evo_log.json" "$res_h"; then
      echo "[trials]   plan $p trial $r attempt $a: route ${pct}% but a checkpoint was missed/skipped -> NOT logged, retrying"
      continue
    fi
    if [ "$pct" -ge 0 ]; then
      # stuck/partial runs count: log whatever route % the attempt reached
      cp "$mj" "$final"
      cp -f "$res_h/world0_evo_log.json" "$OUT/plan${p}_trial${r}_evo_log.json" 2>/dev/null
      echo "[trials]   plan $p trial $r LOGGED: route ${pct}% on attempt $a -> $(basename "$final")"
      logged=1
      break
    else
      echo "[trials]   plan $p trial $r attempt $a: no metrics summary (bringup failure/timeout, rc=$rc) -> retrying"
    fi
  done
  [ "$logged" = 1 ] || echo "[trials] plan $p trial $r NOT COMPLETED after $MAX_ATTEMPTS attempts (nothing logged)"
done
done

cleanup
echo
echo "[trials] ALL DONE in $((SECONDS-START))s -> $OUT"
print_summary
