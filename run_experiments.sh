#!/usr/bin/env bash
# run_experiments.sh — warehouse density batch: for each env in ENVS (crowd
# densities: worlds/warehouse_people<N>) run the task TRIALS times with the
# mission plan src/planning_ros_pkgs/evo_skill_ros/config/plan.txt (the
# plan.txt used by process.md), following the manual bringup sequence of
# process.md (AMCL on factory_sim_map + nav2_custom + 4-cam YOLO + trackers).
# Each attempt is one full bringup + trial, automated inside the ros_humble
# docker container by density_sweep.py (which encodes process.md terminals 1-8).
#
# Each attempt spawns the robot at a noisy start position: x ~ 0±1 m,
# y ~ 1±0.5 m (uniform, re-drawn per attempt; recorded in the attempt's
# spawn.txt). The AMCL initial pose is set to the same point.
#
# Stuck-robot policy: a trial is logged ONLY when scand_metrics reports 100%
# route completion (path_progress_pct == 100). An attempt that ends short of
# that (robot stuck, bringup failure, backstop expiry) is discarded and the
# same trial re-run in a fresh container until the route completes; only then
# does the loop move on. Resumable: trials already logged at 100% are skipped.
#
# Run on the HOST. Results -> results/warehouse_envs/<N>_<r>.json (process.md
# [num_ppl]_[trial] naming) + <N>_<r>_evo_log.json + <N>_<r>.tex; per-attempt
# bringup/sim logs under results/warehouse_envs/_runs/w<N>_trial<r>_attempt<a>/.
#
# Env: ENVS ("0 10 20 30 40"), TRIALS (10), BACKSTOP (600 s per-attempt sim-time
#      metrics cap), MAX_ATTEMPTS (20 attempts per trial), CONT (evo_exp),
#      OUT (results dir, must be under the repo so the container sees it).
set -uo pipefail

HOSTREPO=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CREPO=/home/user/autonomy_stack_ros_humble   # repo mount point inside container
COMPOSE=$HOSTREPO/docker-compose.yml
CONT=${CONT:-evo_exp}

TRIALS=${TRIALS:-10}
ENVS=$(echo "${ENVS:-0 10 20 30 40}" | tr ',' ' ')   # crowd densities to sweep
BACKSTOP=${BACKSTOP:-600}
MAX_ATTEMPTS=${MAX_ATTEMPTS:-20}
BRINGUP_BUDGET=${BRINGUP_BUDGET:-1200}       # wall-clock allowance beyond BACKSTOP
OUT=${OUT:-$HOSTREPO/results/warehouse_envs}

case "$OUT" in                               # container view of $OUT
  "$HOSTREPO"/*) OUT_C=$CREPO${OUT#"$HOSTREPO"} ;;
  *) echo "[exp] ERROR: OUT=$OUT is not under $HOSTREPO (container can't see it)"; exit 1 ;;
esac
RUNS_H=$OUT/_runs
mkdir -p "$RUNS_H"

PLAN_H=$HOSTREPO/src/planning_ros_pkgs/evo_skill_ros/config/plan.txt
PLAN_C=$CREPO/src/planning_ros_pkgs/evo_skill_ros/config/plan.txt
[ -s "$PLAN_H" ] || { echo "[exp] ERROR: plan file missing: $PLAN_H"; exit 1; }

# goal region = destination of the plan's last (move ...) — R11 for current plan.txt
TARGET=$(grep -oE '\(move +[[:alnum:]_]+ +R[0-9]+ +R[0-9]+\)' "$PLAN_H" \
         | tail -1 | grep -oE 'R[0-9]+' | tail -1)
[ -n "$TARGET" ] || { echo "[exp] ERROR: no (move ...) action in $PLAN_H"; exit 1; }

for n in $ENVS; do
  [ -s "$HOSTREPO/worlds/warehouse_people${n}.sdf" ] \
    || { echo "[exp] ERROR: worlds/warehouse_people${n}.sdf missing"; exit 1; }
done

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
  # shellcheck disable=SC2086
  python3 - "$OUT" "$TRIALS" $ENVS <<'PY'
import json, sys, pathlib
out, trials = pathlib.Path(sys.argv[1]), int(sys.argv[2])
envs = sys.argv[3:]
hdr = (f"{'env':>4} {'trial':>5} {'route%':>7} {'succ':>5} {'time[s]':>8} "
       f"{'path[m]':>8} {'coll':>5} {'minclr[m]':>10}")
print(hdr); print("-" * len(hdr))
for n in envs:
    for r in range(1, trials + 1):
        f = out / f"{n}_{r}.json"
        if not f.exists():
            print(f"{n:>4} {r:>5}   -- not logged --")
            continue
        d = json.load(open(f))
        fmt = lambda v, p=1: "--" if v is None else f"{v:.{p}f}"
        print(f"{n:>4} {r:>5} {fmt(d.get('path_progress_pct')):>7} "
              f"{'yes' if d.get('success') else 'no':>5} {fmt(d.get('duration_s'), 0):>8} "
              f"{fmt(d.get('path_length_m')):>8} {fmt(d.get('collisions'), 0):>5} "
              f"{fmt(d.get('min_human_clearance_m'), 2):>10}")
PY
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
  echo "[exp] interrupted -> tearing down container '$CONT' (in-flight attempt discarded; no partial trial is logged)"
  cleanup
  echo "[exp] stopped after $((SECONDS-START))s. Logged trials remain in $OUT — rerun to resume."
  print_summary
  exit 130
}
START=$SECONDS
trap on_interrupt INT TERM HUP   # HUP: ssh session drop must still tear down
trap cleanup EXIT

echo "[exp] warehouse envs [$ENVS] x $TRIALS trials  plan=$PLAN_C  target=$TARGET"
echo "[exp] per-attempt backstop=${BACKSTOP}s  max_attempts=$MAX_ATTEMPTS  out=$OUT"

for WORLD_N in $ENVS; do
for r in $(seq 1 "$TRIALS"); do
  final="$OUT/${WORLD_N}_${r}.json"
  if [ -s "$final" ] && [ "$(progress_pct "$final")" -ge 100 ]; then
    echo "[exp] env $WORLD_N trial $r already logged at 100% -> skip"
    continue
  fi

  logged=0
  for a in $(seq 1 "$MAX_ATTEMPTS"); do
    echo "[exp] ===== env $WORLD_N trial $r attempt $a  t+$((SECONDS-START))s ====="
    if ! fresh; then
      echo "[exp]   container/ROS not ready -> retry"
      continue
    fi

    res_h=$RUNS_H/w${WORLD_N}_trial${r}_attempt${a}
    res_c=$OUT_C/_runs/w${WORLD_N}_trial${r}_attempt${a}
    rm -rf "$res_h"; mkdir -p "$res_h"

    # noisy start: x ~ 0±1 m, y ~ 1±0.5 m (uniform), fresh draw per attempt;
    # density_sweep uses SPAWN_X/Y for both the Gazebo spawn and the AMCL
    # initial pose, so localization stays consistent with the spawn.
    read -r SX SY <<<"$(python3 -c \
      'import random; print(f"{random.uniform(-1,1):.3f} {random.uniform(0.5,1.5):.3f}")')"
    echo "[exp]   spawn: x=$SX y=$SY"
    echo "$SX $SY" > "$res_h/spawn.txt"

    # one full process.md bringup + trial; metrics_stop_on_success inside
    # density_sweep ends the run the moment the route completes.
    # (2x BACKSTOP: metrics_duration is sim time and the world RTF is capped
    # at 0.5, so a full backstop can take twice its value in wall-clock.)
    timeout -k 30 $((2 * BACKSTOP + BRINGUP_BUDGET)) docker exec "$CONT" bash -lc "
      cd $CREPO &&
      WORLDS=$WORLD_N TARGET_REGION=$TARGET PLAN_FILE=$PLAN_C \
      SPAWN_X=$SX SPAWN_Y=$SY \
      RESULTS_DIR=$res_c TRIAL_TIMEOUT=$BACKSTOP MAX_ATTEMPTS=3 \
      python3 density_sweep.py
    " > "$res_h/sweep.log" 2>&1
    rc=$?

    mj=$res_h/world${WORLD_N}_metrics.json
    pct=$(progress_pct "$mj")
    if [ "$pct" -ge 100 ] && ! checkpoints_ok "$res_h/world${WORLD_N}_evo_log.json" "$res_h"; then
      echo "[exp]   env $WORLD_N trial $r attempt $a: route ${pct}% but a checkpoint was missed/skipped -> NOT logged, retrying"
      continue
    fi
    if [ "$pct" -ge 100 ]; then
      cp "$mj" "$final"
      cp -f "$res_h/world${WORLD_N}_evo_log.json" "$OUT/${WORLD_N}_${r}_evo_log.json" 2>/dev/null
      cp -f "$res_h/density_table.tex" "$OUT/${WORLD_N}_${r}.tex" 2>/dev/null
      echo "[exp]   env $WORLD_N trial $r COMPLETE: 100% route on attempt $a -> $(basename "$final")"
      logged=1
      break
    elif [ "$pct" -ge 0 ]; then
      echo "[exp]   env $WORLD_N trial $r attempt $a: route ${pct}% < 100% (stuck/backstop, rc=$rc) -> NOT logged, retrying"
    else
      echo "[exp]   env $WORLD_N trial $r attempt $a: no metrics summary (bringup failure/timeout, rc=$rc) -> retrying"
    fi
  done
  [ "$logged" = 1 ] || echo "[exp] env $WORLD_N trial $r NOT COMPLETED after $MAX_ATTEMPTS attempts (nothing logged)"
done
done

cleanup
echo
echo "[exp] ALL DONE in $((SECONDS-START))s -> $OUT"
print_summary
