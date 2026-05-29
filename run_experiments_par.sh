#!/usr/bin/env bash
# Parallel batch SCAND-shield experiments over factory_mission_plans/*.txt.
#
# Spins up N isolated worker containers (distinct ROS_DOMAIN_ID + IGN_PARTITION,
# own PID namespace so a worker's teardown can't kill its siblings) cloned from
# the same image/mounts as evo_dev. The 15 (plan x rep) work items are spread
# round-robin across workers; each worker runs its items sequentially, restarting
# its own container between runs for a clean gz/DDS state. All run logs go to the
# /home mount (root fs is tight). Resumable: items with a captured .summary skip.
#
# Results -> results/factory_missions/plan<NN>_rep<r>.summary  (shared, resumable)
#
# PORTABLE: the host repo path is derived from this script's location, so it runs
# on any machine where (1) the docker image is built (see Dockerfile / build.sh),
# (2) ~/clearpath is generated (clearpath_generator_common), and (3) an X display
# is available for gz rendering. Override defaults via env.
#
# Env: WORKERS (2), REPS (3), BACKSTOP (600s), RUN_TIMEOUT (660s), STAGGER (100s),
#      IMAGE (ubuntu-22-humble:latest), CLEARPATH_DIR ($HOME/clearpath),
#      HF_CACHE ($HOME/.cache/huggingface), DISPLAY.
set -uo pipefail

# repo root = directory containing this script (portable across machines)
HOSTREPO=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CREPO=/home/user/autonomy_stack_ros_humble        # repo mount point inside container (fixed by image)
IMAGE=${IMAGE:-ubuntu-22-humble:latest}
CLEARPATH_DIR=${CLEARPATH_DIR:-$HOME/clearpath}   # host clearpath config (robot.yaml etc.)
HF_CACHE=${HF_CACHE:-$HOME/.cache/huggingface}
OUT=$HOSTREPO/results/factory_missions
LOGROOT_HOST=$OUT/_log                       # host view of per-run logs (on /home)
LOGROOT_CONT=$CREPO/results/factory_missions/_log
mkdir -p "$OUT" "$LOGROOT_HOST"

WORKERS=${WORKERS:-4}
REPS=${REPS:-3}
BACKSTOP=${BACKSTOP:-600}     # per-run budget: metrics stops + prints at 10 min if goal not reached
RUN_TIMEOUT=${RUN_TIMEOUT:-660}
PLANS=(01 02 03 04 05)
# auto-derive each plan's goal region from its last (move ... RX RY) destination,
# so swapping plan files needs no edits here.
declare -A TGT
for _p in "${PLANS[@]}"; do
  _pf="$HOSTREPO/factory_mission_plans/factory_mission_${_p}.txt"
  TGT[$_p]=$(grep -oE '\(move [a-z_0-9]+ R[0-9]+ R[0-9]+\)' "$_pf" 2>/dev/null | tail -1 | grep -oE 'R[0-9]+' | tail -1)
done

NAV2_YAML=/opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml

ensure_worker() {   # $1=worker idx ; create (idempotent) + loosen nav2 tolerance
  local k=$1 name=evo_w$k
  mkdir -p "$LOGROOT_HOST/w$k/roslog"
  if ! docker inspect "$name" >/dev/null 2>&1; then
    # required mounts; optional ones added only if present on this host
    local mounts=( -v "$HOSTREPO":"$CREPO" -v "$HOSTREPO/docker/.bashrc":/home/user/.bashrc )
    [ -d "$CLEARPATH_DIR" ] && mounts+=( -v "$CLEARPATH_DIR":/home/user/clearpath )
    [ -d /tmp/.X11-unix ]   && mounts+=( -v /tmp/.X11-unix:/tmp/.X11-unix )
    [ -d "$HF_CACHE" ]      && mounts+=( -v "$HF_CACHE":/home/.cache/huggingface )
    docker run -d --name "$name" \
      --gpus all --network host --ipc host \
      -e DISPLAY="${DISPLAY:-:1}" \
      -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
      -e HOME=/home/user \
      -e ROS_DOMAIN_ID="$k" -e ROS_LOCALHOST_ONLY=1 \
      -e IGN_IP=127.0.0.1 -e IGN_PARTITION="w$k" -e GZ_PARTITION="w$k" \
      -e ROS_LOG_DIR="$LOGROOT_CONT/w$k/roslog" \
      "${mounts[@]}" \
      "$IMAGE" sleep infinity >/dev/null
  else
    docker start "$name" >/dev/null 2>&1
  fi
  # loosen Nav2 goal tolerance (the running image isn't rebuilt yet)
  docker exec "$name" bash -lc "sed -i \
      -e 's/xy_goal_tolerance: 0.3/xy_goal_tolerance: 0.6/' \
      -e 's/yaw_goal_tolerance: 0.3/yaw_goal_tolerance: 3.15/' \
      -e 's/xy_goal_tolerance: 0.25/xy_goal_tolerance: 0.6/' \
      $NAV2_YAML 2>/dev/null; true"
}

wait_ros() {   # $1=container name
  local name=$1 i
  for i in $(seq 1 60); do
    if docker exec "$name" bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; timeout 8 ros2 topic list >/dev/null 2>&1'; then
      return 0
    fi
    sleep 3
  done
  return 1
}

teardown() {   # $1=container name (own pid ns -> scoped, safe)
  docker exec "$1" bash -lc '
    pkill -f run_sim.sh; pkill -f evo_plan_run.launch;
    pkill -f "simulation.launch|nav2.launch|slam.launch";
    pkill -f "yolo-world.launch|tracker_with_yolo";
    pkill -f scan_relay.py; pkill -f scand_metrics.py;
    pkill -f "ign gazebo|ruby"; true' >/dev/null 2>&1
}

run_one() {    # $1=worker $2=plan $3=rep $4=target
  local k=$1 p=$2 r=$3 tgt=$4 name=evo_w$1
  local sm="$OUT/plan${p}_rep${r}.summary"
  local ld_host="$LOGROOT_HOST/w$k/p${p}_r${r}"
  local ld_cont="$LOGROOT_CONT/w$k/p${p}_r${r}"
  if [ -s "$sm" ] && grep -q "SCAND-shield runtime metrics" "$sm"; then
    echo "[w$k] plan$p rep$r already done -> skip"; return
  fi
  # randomized crowd density for this run: rep -> band (sparse/medium/dense),
  # exact walker count + subset seeded by (plan,rep) for reproducibility.
  # Computed once; retries reuse the same crowd.
  local dmin dmax
  case "$r" in 1) dmin=4; dmax=7;; 2) dmin=8; dmax=11;; *) dmin=12; dmax=16;; esac
  local vbase="worlds/_variants/wh_p${p}_r${r}"
  local nwalk
  nwalk=$(python3 "$HOSTREPO/make_density_world.py" \
            --base "$HOSTREPO/worlds/warehouse_people.sdf" \
            --out "$HOSTREPO/${vbase}.sdf" --seed $((10#$p*100+r)) \
            --min "$dmin" --max "$dmax" 2>/dev/null | grep -oE "[0-9]+")
  echo "${nwalk:-?}" > "$OUT/plan${p}_rep${r}.density"

  # up to 2 attempts: flaky bringups (controller_manager / DDS service timeouts)
  # usually clear on a fresh container restart. A run that produces ANY metrics
  # summary is a real result (kept, even succ=no); only no-summary retries.
  local attempt
  for attempt in 1 2; do
    echo "[w$k] ===== plan$p rep$r (target=$tgt, ${nwalk:-?} walkers, attempt $attempt) ====="
    teardown "$name"
    docker restart "$name" >/dev/null
    ensure_worker "$k"
    if ! wait_ros "$name"; then echo "[w$k] plan$p rep$r: ROS not ready (attempt $attempt)"; continue; fi
    sleep 4
    rm -rf "$ld_host"; mkdir -p "$ld_host"

    docker exec -d "$name" bash -lc "
      source /opt/ros/humble/setup.bash;
      source $CREPO/install/setup.bash;
      cd $CREPO;
      PLAN=$CREPO/factory_mission_plans/factory_mission_${p}.txt \
      TARGET=$tgt WORLD=$CREPO/${vbase} \
      MULTICAM=1 METRICS=1 UNTIL_SUCCESS=1 \
      METRICS_DURATION=$BACKSTOP NAV2_TIMEOUT=300 YOLO_TIMEOUT=120 \
      COSTMAP_EDIT_RADIUS=0.3 STL_REPLAN_COOLDOWN=5.0 \
      LOGDIR=$ld_cont \
      ./run_sim.sh > $ld_cont/runsim.log 2>&1
    "

    local deadline=$((SECONDS + RUN_TIMEOUT)) got=0 bfail=0
    while [ $SECONDS -lt $deadline ]; do
      if grep -aq "SCAND-shield runtime metrics" "$ld_host/evo.log" 2>/dev/null; then got=1; break; fi
      if grep -aq "ERROR: Nav2 did not activate\|ERROR: no robot under namespace\|ERROR: robot did not spawn" "$ld_host"/*.log 2>/dev/null; then
        bfail=1; break; fi
      sleep 10
    done
    sleep 6
    grep -aB2 -A32 "SCAND-shield runtime metrics" "$ld_host/evo.log" 2>/dev/null > "$sm"
    if [ "$got" = 1 ] && grep -q "SCAND-shield runtime metrics" "$sm"; then
      echo "[w$k] plan$p rep$r: captured ($(grep -ao 'succ.*: [a-z]*' "$sm" | head -1))"
      teardown "$name"; return
    fi
    echo "[w$k] plan$p rep$r: attempt $attempt failed ($([ "$bfail" = 1 ] && echo bringup || echo timeout)/no-summary)"
    teardown "$name"
  done
  : > "$sm"   # leave empty so the cleanup pass / resume retries it
  echo "[w$k] plan$p rep$r: NO SUMMARY after retries"
}

worker_loop() {   # $1=worker idx ; remaining args = "p:r:tgt" items
  local k=$1; shift
  ensure_worker "$k"
  local it
  for it in "$@"; do
    IFS=: read -r p r tgt <<<"$it"
    run_one "$k" "$p" "$r" "$tgt"
  done
  echo "[w$k] done."
}

# ---- build work items and distribute round-robin ----
items=()
for p in "${PLANS[@]}"; do for r in $(seq 1 "$REPS"); do items+=("$p:$r:${TGT[$p]}"); done; done
declare -A BUCKET
for i in "${!items[@]}"; do
  k=$(( i % WORKERS + 1 ))
  BUCKET[$k]+="${items[$i]} "
done

if [ "${PAR_NO_MAIN:-0}" != "1" ]; then
  echo "[par] ${#items[@]} runs across $WORKERS workers (backstop=${BACKSTOP}s)"
  START=$SECONDS
  pids=()
  for k in $(seq 1 "$WORKERS"); do
    # stagger initial starts so concurrent bringups don't saturate the CPU
    # (4 simultaneous bringups starve the Nav2 lifecycle handshake).
    [ "$k" -gt 1 ] && sleep "${STAGGER:-100}"
    # shellcheck disable=SC2086
    worker_loop "$k" ${BUCKET[$k]:-} > "/tmp/exp_par_w$k.log" 2>&1 &
    pids+=($!)
  done
  wait "${pids[@]}"
  echo "[par] ALL DONE in $((SECONDS-START))s"
fi
