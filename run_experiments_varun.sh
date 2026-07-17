#!/usr/bin/env bash
# Batch SCAND-shield experiments over factory_mission_plans/*.txt.
#
# For each plan x repeat: restart the container (clean gz/DDS), run the full
# MULTICAM + warehouse_people pipeline via run_sim.sh until the robot reaches
# the plan goal (UNTIL_SUCCESS) or a backstop fires, capture the scand_metrics
# summary, then tear the run down. Resumable: a run whose .summary already
# exists is skipped.
#
# Results -> results/factory_missions/plan<NN>_rep<r>.{summary,evo.log}
#
# Env overrides: REPS (3), BACKSTOP (900s metrics cap), RUN_TIMEOUT (1100s hard
# wall-clock per run), CONT (evo_dev).
set -uo pipefail

REPO=/home/mvarun/Research/temp/autonomy_stack
CREPO=/home/user/autonomy_stack_ros_humble        # repo path inside container
CONT=${CONT:-evo_dev}
OUT=$REPO/results/factory_missions
mkdir -p "$OUT"

PLANS=(01 02 03 04 05)
declare -A TGT=( [01]=R8 [02]=R7 [03]=R7 [04]=R7 [05]=R12 )
REPS=${REPS:-3}
BACKSTOP=${BACKSTOP:-900}
RUN_TIMEOUT=${RUN_TIMEOUT:-1100}

wait_ros() {   # wait until the ros2 daemon is usable after a container restart
  local i
  for i in $(seq 1 60); do
    if docker exec "$CONT" bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; timeout 8 ros2 topic list >/dev/null 2>&1'; then
      return 0
    fi
    sleep 3
  done
  return 1
}

teardown() {   # kill the sim pipeline inside the container (not the container)
  docker exec "$CONT" bash -lc '
    pkill -f run_sim.sh; pkill -f evo_plan_run.launch;
    pkill -f "simulation.launch|nav2.launch|slam.launch";
    pkill -f "yolo-world.launch|tracker_with_yolo";
    pkill -f scan_relay.py; pkill -f scand_metrics.py;
    pkill -f "ign gazebo|ruby"; true' >/dev/null 2>&1
}

echo "[exp] $((${#PLANS[@]} * REPS)) runs (plans=${PLANS[*]} reps=$REPS backstop=${BACKSTOP}s timeout=${RUN_TIMEOUT}s)"
START=$SECONDS

for p in "${PLANS[@]}"; do
  for r in $(seq 1 "$REPS"); do
    sm="$OUT/plan${p}_rep${r}.summary"
    if [ -s "$sm" ] && grep -q "SCAND-shield runtime metrics" "$sm"; then
      echo "[exp] plan$p rep$r already done -> skip"
      continue
    fi
    echo "[exp] ===== plan$p rep$r (target=${TGT[$p]})  t+$((SECONDS-START))s ====="

    teardown
    echo "[exp]   restarting container..."
    docker restart "$CONT" >/dev/null
    if ! wait_ros; then echo "[exp]   ROS not ready after restart -> skip"; continue; fi
    sleep 5

    LOGDIR=/tmp/exp_p${p}_r${r}
    docker exec "$CONT" bash -lc "rm -rf $LOGDIR; mkdir -p $LOGDIR" >/dev/null 2>&1

    echo "[exp]   launching run_sim.sh ..."
    docker exec -d "$CONT" bash -lc "
      source /opt/ros/humble/setup.bash;
      source $CREPO/install/setup.bash;
      cd $CREPO;
      PLAN=$CREPO/factory_mission_plans/factory_mission_${p}.txt \
      TARGET=${TGT[$p]} \
      WORLD=$CREPO/worlds/warehouse_people \
      MULTICAM=1 METRICS=1 UNTIL_SUCCESS=1 \
      METRICS_DURATION=$BACKSTOP NAV2_TIMEOUT=300 YOLO_TIMEOUT=120 \
      COSTMAP_EDIT_RADIUS=0.3 STL_REPLAN_COOLDOWN=5.0 \
      LOGDIR=$LOGDIR \
      ./run_sim.sh > $LOGDIR/runsim.log 2>&1
    "

    # poll evo.log for the SCAND summary (printed when metrics exits)
    deadline=$((SECONDS + RUN_TIMEOUT))
    got=0
    while [ $SECONDS -lt $deadline ]; do
      if docker exec "$CONT" bash -lc "grep -aq 'SCAND-shield runtime metrics' $LOGDIR/evo.log 2>/dev/null"; then
        got=1; break
      fi
      # bail early if bringup itself failed
      if docker exec "$CONT" bash -lc "grep -aq 'ERROR: Nav2 did not activate\|ERROR: robot did not spawn' $LOGDIR/*.log 2>/dev/null"; then
        echo "[exp]   bringup failed early"; break
      fi
      sleep 10
    done
    sleep 6   # let the summary finish printing

    docker exec "$CONT" bash -lc "grep -aB2 -A32 'SCAND-shield runtime metrics' $LOGDIR/evo.log 2>/dev/null" > "$sm"
    docker cp "$CONT:$LOGDIR/evo.log" "$OUT/plan${p}_rep${r}.evo.log" >/dev/null 2>&1
    docker cp "$CONT:$LOGDIR/runsim.log" "$OUT/plan${p}_rep${r}.runsim.log" >/dev/null 2>&1

    if [ "$got" = 1 ] && grep -q "SCAND-shield runtime metrics" "$sm"; then
      echo "[exp]   plan$p rep$r: summary captured ($(grep -o 'succ.*: .*' "$sm" | head -1))"
    else
      echo "[exp]   plan$p rep$r: NO SUMMARY (timeout/bringup) -> see runsim.log"
    fi
    teardown
    sleep 3
  done
done

echo "[exp] ALL DONE in $((SECONDS-START))s -> $OUT"
