#!/usr/bin/env bash
# run_ablation.sh — warehouse density ABLATION: crowd size N swept 0..20 in
# steps of 1 (finer than run_experiments.sh's 0/10/20/30/40). Instead of a
# prebuilt world per level, every run subsamples N walkers WITHOUT REPLACEMENT
# from a single 100-person pool (worlds/warehouse_people100.sdf), so different
# trials at the same N see different crowds.
#
# For each (N, trial r) the crowd is drawn ONCE by make_density_world.py
# (--keep N, seed = N*1000+r, reproducible) into worlds/_variants/abl_p<N>_r<r>.sdf
# and reused across that trial's retry attempts (the experimental condition is
# constant per trial; only the robot spawn is re-noised per attempt). The same
# variant SDF is handed to Gazebo AND to scand_metrics (metrics_actors_sdf) by
# density_sweep.py via WORLD_NAME, so metrics see exactly the people present.
#
# Each attempt spawns the robot at a noisy start position: x ~ 0±1 m,
# y ~ 1±0.5 m (uniform, re-drawn per attempt; recorded in the attempt's
# spawn.txt). The AMCL initial pose is set to the same point. (Unchanged.)
#
# Stuck-robot policy: a trial is logged ONLY when scand_metrics reports 100%
# route completion (path_progress_pct == 100). An attempt that ends short of
# that is discarded and the same trial re-run in a fresh container until the
# route completes. Resumable: trials already logged at 100% are skipped.
#
# The N=0 baseline (empty crowd) is identical to the old sweep's empty world, so
# its existing valid trials are copied in from $SRC0 and NOT re-run.
#
# Run on the HOST. Results -> results/warehouse_ablation/<N>_<r>.json
# (+ <N>_<r>_evo_log.json + <N>_<r>.tex); per-attempt logs under
# results/warehouse_ablation/_runs/w<N>_trial<r>_attempt<a>/.
#
# Env: LEVELS ("0 1 2 ... 20"), TRIALS (10), BACKSTOP (600 s per-attempt sim-time
#      metrics cap), MAX_ATTEMPTS (20 attempts per trial), CONT (evo_abl),
#      SRC0 (results/warehouse_envs — source of reusable N=0 trials),
#      OUT (results dir, must be under the repo so the container sees it).
set -uo pipefail

HOSTREPO=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CREPO=/home/user/autonomy_stack_ros_humble   # repo mount point inside container
COMPOSE=$HOSTREPO/docker-compose.yml
CONT=${CONT:-evo_abl}

TRIALS=${TRIALS:-5}
LEVELS=$(echo "${LEVELS:-$(seq 2 2 20)}" | tr ',' ' ')   # crowd sizes to sweep: 2,4,..,20
BACKSTOP=${BACKSTOP:-600}
MAX_ATTEMPTS=${MAX_ATTEMPTS:-20}
BRINGUP_BUDGET=${BRINGUP_BUDGET:-1200}       # wall-clock allowance beyond BACKSTOP
OUT=${OUT:-$HOSTREPO/results/warehouse_ablation}
SRC0=${SRC0:-$HOSTREPO/results/warehouse_envs}   # reusable, already-valid N=0 trials

case "$OUT" in                               # container view of $OUT
  "$HOSTREPO"/*) OUT_C=$CREPO${OUT#"$HOSTREPO"} ;;
  *) echo "[abl] ERROR: OUT=$OUT is not under $HOSTREPO (container can't see it)"; exit 1 ;;
esac
RUNS_H=$OUT/_runs
mkdir -p "$RUNS_H"

PLAN_H=$HOSTREPO/src/planning_ros_pkgs/evo_skill_ros/config/plan.txt
PLAN_C=$CREPO/src/planning_ros_pkgs/evo_skill_ros/config/plan.txt
[ -s "$PLAN_H" ] || { echo "[abl] ERROR: plan file missing: $PLAN_H"; exit 1; }

# goal region = destination of the plan's last (move ...) — R11 for current plan.txt
TARGET=$(grep -oE '\(move +[[:alnum:]_]+ +R[0-9]+ +R[0-9]+\)' "$PLAN_H" \
         | tail -1 | grep -oE 'R[0-9]+' | tail -1)
[ -n "$TARGET" ] || { echo "[abl] ERROR: no (move ...) action in $PLAN_H"; exit 1; }

# single 100-person sampling pool (worlds/warehouse_people100.sdf); regenerate
# with:  python3 worlds/gen_density_worlds.py
POOL_H=$HOSTREPO/worlds/warehouse_people100.sdf
[ -s "$POOL_H" ] || { echo "[abl] ERROR: pool missing: $POOL_H (run worlds/gen_density_worlds.py)"; exit 1; }
mkdir -p "$HOSTREPO/worlds/_variants"

# process.md: the 4-camera robot config must be the active clearpath config
cp "$HOSTREPO/robot_4cam.yaml" "$HOME/clearpath/robot.yaml"

# Gazebo must render on the machine-local X server. Over `ssh -Y` the shell's
# DISPLAY points at the SSH-forwarded display, which the container has no X
# cookie for ("X11 connection rejected because of wrong authentication") and
# which would tunnel GPU camera rendering over the network. Likewise blank out
# XAUTHORITY (an ssh session sets it to a host path that shadows local access).
SIM_DISPLAY=${SIM_DISPLAY:-:1}

# Observation-logger env forwarded into the container (docker exec does NOT
# inherit the host environment, so each var must be named explicitly below).
# Only NON-EMPTY vars are forwarded: density_sweep.py reads these with
# os.environ.get(k, <default>), so exporting an empty value would override the
# default with "" and emit a bare `obs_log_cameras:=`, which ros2 launch rejects
# as a malformed argument. Set OBS_LOG=1 to enable.
OBS_ENV=""
for _v in OBS_LOG OBS_LOG_PERIOD OBS_LOG_CAMERAS OBS_LOG_CLASSES OBS_EXCLUDE_FILE; do
  [ -n "${!_v:-}" ] && OBS_ENV="$OBS_ENV $_v=${!_v}"
done
[ -n "$OBS_ENV" ] && echo "[abl] forwarding observation-logger env:$OBS_ENV"

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

checkpoints_ok() {   # $1 = evo log json   ($2 = attempt dir, kept for signature compat)
  # This is a SECONDARY gate; the caller already required path_progress_pct==100
  # (the route was completed to the goal region). Here we reject ONLY a positive
  # genuine-skip signal: a nav2_goal_finished event whose missed_waypoints list is
  # non-empty. Two things we deliberately do NOT reject on (both were shown to
  # discard valid, route-completing runs, ~64% of 100% attempts):
  #   * an ABSENT nav2_goal_finished event — the metrics_stop_on_success race tears
  #     the run down at 100% before the evo node logs the finish; the robot still
  #     reached the goal (pct==100), so this is success, not a miss.
  #   * "Failed to process waypoint" in the nav2 log — an STL replan canceling and
  #     re-sending a waypoint goal to route around a person (legitimate crowd
  #     avoidance) prints this for the superseded goal; the FINAL goal still
  #     finishes with empty missed_waypoints. A genuine skip would appear in
  #     missed_waypoints regardless, which is what we check. Set STRICT_CHECKPOINTS=1
  #     to restore the old all-checkpoints-visited / no-failover behavior.
  if [ "${STRICT_CHECKPOINTS:-0}" = 1 ]; then
    python3 - "$1" <<'PY' 2>/dev/null || return 1
import json, sys
d = json.load(open(sys.argv[1]))
fin = [e for e in d.get("events", []) if e.get("event") == "nav2_goal_finished"]
sys.exit(0 if fin and not fin[-1].get("data", {}).get("missed_waypoints") else 1)
PY
    ! grep -aqs "Failed to process waypoint" "$2"/_logs/world*/attempt*/nav2_attempt*.log
    return
  fi
  python3 - "$1" <<'PY' 2>/dev/null || return 1
import json, sys
d = json.load(open(sys.argv[1]))
fin = [e for e in d.get("events", []) if e.get("event") == "nav2_goal_finished"]
# reject only on a POSITIVE genuine skip; absent finish event => success (race)
sys.exit(1 if (fin and fin[-1].get("data", {}).get("missed_waypoints")) else 0)
PY
}

print_summary() {
  # shellcheck disable=SC2086
  python3 - "$OUT" "$TRIALS" $LEVELS <<'PY'
import json, sys, pathlib
out, trials = pathlib.Path(sys.argv[1]), int(sys.argv[2])
levels = sys.argv[3:]
hdr = (f"{'N':>4} {'trial':>5} {'route%':>7} {'succ':>5} {'time[s]':>8} "
       f"{'path[m]':>8} {'coll':>5} {'minclr[m]':>10}")
print(hdr); print("-" * len(hdr))
for n in levels:
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
  echo "[abl] interrupted -> tearing down container '$CONT' (in-flight attempt discarded; no partial trial is logged)"
  cleanup
  echo "[abl] stopped after $((SECONDS-START))s. Logged trials remain in $OUT — rerun to resume."
  print_summary
  exit 130
}
START=$SECONDS
trap on_interrupt INT TERM HUP   # HUP: ssh session drop must still tear down
trap cleanup EXIT

echo "[abl] warehouse ablation N=[$LEVELS] x $TRIALS trials  pool=$(basename "$POOL_H")  plan=$PLAN_C  target=$TARGET"
echo "[abl] per-attempt backstop=${BACKSTOP}s  max_attempts=$MAX_ATTEMPTS  out=$OUT"

# Reuse existing valid N=0 trials (empty crowd == --keep 0) instead of re-running.
if [ -d "$SRC0" ]; then
  for r in $(seq 1 "$TRIALS"); do
    for suf in ".json" "_evo_log.json" ".tex"; do
      s="$SRC0/0_${r}${suf}"; d="$OUT/0_${r}${suf}"
      [ -s "$s" ] && [ ! -e "$d" ] && cp "$s" "$d" && \
        echo "[abl] reused existing N=0 trial $r ($(basename "$s"))"
    done
  done
fi

for N in $LEVELS; do
for r in $(seq 1 "$TRIALS"); do
  final="$OUT/${N}_${r}.json"
  if [ -s "$final" ] && [ "$(progress_pct "$final")" -ge 100 ]; then
    echo "[abl] N $N trial $r already logged at 100% -> skip"
    continue
  fi

  # crowd for this trial: N walkers sampled WITHOUT REPLACEMENT from the 100-pool,
  # seeded by (N, r) so it is reproducible and constant across this trial's
  # retry attempts. Drawn once here; attempts below reuse the same variant.
  WORLD_NAME_REL="_variants/abl_p${N}_r${r}"          # basename density_sweep expects (under worlds/)
  VAR_H="$HOSTREPO/worlds/${WORLD_NAME_REL}.sdf"
  SEED=$((N * 1000 + r))
  WALKERS=$(python3 "$HOSTREPO/make_density_world.py" \
              --base "$POOL_H" --keep "$N" --seed "$SEED" \
              --out "$VAR_H" 2>/dev/null | grep -oE '[0-9]+')
  if [ ! -s "$VAR_H" ]; then
    echo "[abl] N $N trial $r: FAILED to sample crowd (make_density_world) -> skip"
    continue
  fi
  echo "[abl] N $N trial $r: sampled ${WALKERS:-?} of 100 (seed $SEED) -> $WORLD_NAME_REL.sdf"

  logged=0
  for a in $(seq 1 "$MAX_ATTEMPTS"); do
    echo "[abl] ===== N $N trial $r attempt $a  t+$((SECONDS-START))s ====="
    if ! fresh; then
      echo "[abl]   container/ROS not ready -> retry"
      continue
    fi

    res_h=$RUNS_H/w${N}_trial${r}_attempt${a}
    res_c=$OUT_C/_runs/w${N}_trial${r}_attempt${a}
    rm -rf "$res_h"; mkdir -p "$res_h"
    echo "$WALKERS $SEED" > "$res_h/crowd.txt"   # provenance: walker count + seed

    # noisy start: x ~ 0±1 m, y ~ 1±0.5 m (uniform), fresh draw per attempt;
    # density_sweep uses SPAWN_X/Y for both the Gazebo spawn and the AMCL
    # initial pose, so localization stays consistent with the spawn.
    read -r SX SY <<<"$(python3 -c \
      'import random; print(f"{random.uniform(-1,1):.3f} {random.uniform(0.5,1.5):.3f}")')"
    echo "[abl]   spawn: x=$SX y=$SY"
    echo "$SX $SY" > "$res_h/spawn.txt"

    # one full process.md bringup + trial on this trial's sampled crowd; WORLD_NAME
    # routes the variant into BOTH the Gazebo spawn and metrics_actors_sdf, while
    # WORLDS=$N only labels the outputs (world${N}_metrics.json).
    # metrics_stop_on_success inside density_sweep ends the run the moment the
    # route completes. (2x BACKSTOP: metrics_duration is sim time and the world
    # RTF is capped at 0.5, so a full backstop can take twice its value in wall.)
    timeout -k 30 $((2 * BACKSTOP + BRINGUP_BUDGET)) docker exec "$CONT" bash -lc "
      cd $CREPO &&
      WORLDS=$N WORLD_NAME=$WORLD_NAME_REL TARGET_REGION=$TARGET PLAN_FILE=$PLAN_C \
      SPAWN_X=$SX SPAWN_Y=$SY \
      RESULTS_DIR=$res_c TRIAL_TIMEOUT=$BACKSTOP MAX_ATTEMPTS=3 \
     $OBS_ENV \
      python3 density_sweep.py
    " > "$res_h/sweep.log" 2>&1
    rc=$?

    mj=$res_h/world${N}_metrics.json
    pct=$(progress_pct "$mj")
    if [ "$pct" -ge 100 ] && ! checkpoints_ok "$res_h/world${N}_evo_log.json" "$res_h"; then
      echo "[abl]   N $N trial $r attempt $a: route ${pct}% but a checkpoint was missed/skipped -> NOT logged, retrying"
      continue
    fi
    if [ "$pct" -ge 100 ]; then
      cp "$mj" "$final"
      cp -f "$res_h/world${N}_evo_log.json" "$OUT/${N}_${r}_evo_log.json" 2>/dev/null
      cp -f "$res_h/density_table.tex" "$OUT/${N}_${r}.tex" 2>/dev/null
      echo "[abl]   N $N trial $r COMPLETE: 100% route on attempt $a -> $(basename "$final")"
      logged=1
      break
    elif [ "$pct" -ge 0 ]; then
      echo "[abl]   N $N trial $r attempt $a: route ${pct}% < 100% (stuck/backstop, rc=$rc) -> NOT logged, retrying"
    else
      echo "[abl]   N $N trial $r attempt $a: no metrics summary (bringup failure/timeout, rc=$rc) -> retrying"
    fi
  done
  [ "$logged" = 1 ] || echo "[abl] N $N trial $r NOT COMPLETED after $MAX_ATTEMPTS attempts (nothing logged)"
done
done

cleanup
echo
echo "[abl] ALL DONE in $((SECONDS-START))s -> $OUT"
print_summary
