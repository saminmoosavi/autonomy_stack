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
# Env: WORKERS (4), REPS (3), BACKSTOP (900s), RUN_TIMEOUT (1100s).
set -uo pipefail

HOSTREPO=/home/mvarun/Research/temp/autonomy_stack
CREPO=/home/user/autonomy_stack_ros_humble
IMAGE=ubuntu-22-humble:latest
OUT=$HOSTREPO/results/factory_missions
LOGROOT_HOST=$OUT/_log                       # host view of per-run logs (on /home)
LOGROOT_CONT=$CREPO/results/factory_missions/_log
mkdir -p "$OUT" "$LOGROOT_HOST"

WORKERS=${WORKERS:-4}
REPS=${REPS:-3}
BACKSTOP=${BACKSTOP:-900}
RUN_TIMEOUT=${RUN_TIMEOUT:-1100}
PLANS=(01 02 03 04 05)
declare -A TGT=( [01]=R8 [02]=R7 [03]=R7 [04]=R7 [05]=R12 )

NAV2_YAML=/opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml

ensure_worker() {   # $1=worker idx ; create (idempotent) + loosen nav2 tolerance
  local k=$1 name=evo_w$k
  mkdir -p "$LOGROOT_HOST/w$k/roslog"
  if ! docker inspect "$name" >/dev/null 2>&1; then
    docker run -d --name "$name" \
      --gpus all --network host --ipc host \
      -e DISPLAY="${DISPLAY:-:1}" \
      -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
      -e HOME=/home/user \
      -e ROS_DOMAIN_ID="$k" -e ROS_LOCALHOST_ONLY=1 \
      -e IGN_IP=127.0.0.1 -e IGN_PARTITION="w$k" -e GZ_PARTITION="w$k" \
      -e ROS_LOG_DIR="$LOGROOT_CONT/w$k/roslog" \
      -v "$HOSTREPO":"$CREPO" \
      -v "$HOSTREPO/docker/.bashrc":/home/user/.bashrc \
      -v /home/mvarun/clearpath:/home/user/clearpath \
      -v /home/mvarun/clearpath_2:/home/user/clearpath_2 \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /home/mvarun/.cache/huggingface:/home/.cache/huggingface \
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
  echo "[w$k] ===== plan$p rep$r (target=$tgt) ====="
  teardown "$name"
  docker restart "$name" >/dev/null
  ensure_worker "$k"   # re-apply nav2 sed survives restart, but cheap & safe
  if ! wait_ros "$name"; then echo "[w$k] plan$p rep$r: ROS not ready -> skip"; return; fi
  sleep 4
  rm -rf "$ld_host"; mkdir -p "$ld_host"

  docker exec -d "$name" bash -lc "
    source /opt/ros/humble/setup.bash;
    source $CREPO/install/setup.bash;
    cd $CREPO;
    PLAN=$CREPO/factory_mission_plans/factory_mission_${p}.txt \
    TARGET=$tgt WORLD=$CREPO/worlds/warehouse_people \
    MULTICAM=1 METRICS=1 UNTIL_SUCCESS=1 \
    METRICS_DURATION=$BACKSTOP NAV2_TIMEOUT=300 YOLO_TIMEOUT=120 \
    COSTMAP_EDIT_RADIUS=0.3 STL_REPLAN_COOLDOWN=5.0 \
    LOGDIR=$ld_cont \
    ./run_sim.sh > $ld_cont/runsim.log 2>&1
  "

  local deadline=$((SECONDS + RUN_TIMEOUT)) got=0
  while [ $SECONDS -lt $deadline ]; do
    if grep -aq "SCAND-shield runtime metrics" "$ld_host/evo.log" 2>/dev/null; then got=1; break; fi
    if grep -aq "ERROR: Nav2 did not activate\|ERROR: robot did not spawn" "$ld_host"/*.log 2>/dev/null; then
      echo "[w$k] plan$p rep$r: bringup failed"; break; fi
    sleep 10
  done
  sleep 6
  grep -aB2 -A32 "SCAND-shield runtime metrics" "$ld_host/evo.log" 2>/dev/null > "$sm"
  if [ "$got" = 1 ] && grep -q "SCAND-shield runtime metrics" "$sm"; then
    echo "[w$k] plan$p rep$r: captured ($(grep -ao 'succ.*: [a-z]*' "$sm" | head -1))"
  else
    echo "[w$k] plan$p rep$r: NO SUMMARY (timeout/bringup)"
  fi
  teardown "$name"
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
