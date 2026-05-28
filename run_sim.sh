#!/usr/bin/env bash
# run_sim.sh — bring up the full simulation pipeline inside the Docker container:
#   Gazebo (Clearpath Jackal) -> Nav2 + SLAM -> scan relay -> evo_skill plan deploy
#
# Run this INSIDE the container (after `docker compose run --rm ros_humble`).
# On the HOST first allow the GUI:  xhost +local:
#
# Env overrides: NS, WORLD, TARGET, WS, LOGDIR, NO_EVO=1 (skip evo), RVIZ=true
set -o pipefail   # NOTE: no 'set -u' — ROS setup.bash references unbound vars

WS=${WS:-$HOME/autonomy_stack_ros_humble}
NS=${NS:-/j100_0000}                       # robot namespace (matches ~/clearpath/robot.yaml)
WORLD=${WORLD:-warehouse}
TARGET=${TARGET:-R10}
RVIZ=${RVIZ:-false}
LIDAR3D=${LIDAR3D:-${NS}/sensors/lidar3d_0/scan}   # sim VLP16 flattened scan (has a publisher)
LIDAR2D=${LIDAR2D:-${NS}/sensors/lidar2d_0/scan}   # topic Clearpath SLAM/Nav2 subscribe to
LOGDIR=${LOGDIR:-/tmp/evo_sim}
mkdir -p "$LOGDIR"

# --- Fixes for this multi-NIC host (see README "Troubleshooting") ---
export ROS_LOCALHOST_ONLY=1     # pin all ROS2 DDS to loopback; else Nav2 lifecycle hangs
export IGN_IP=127.0.0.1         # pin gz-transport to loopback; else the robot never spawns
export DISPLAY=${DISPLAY:-:1}
unset XAUTHORITY 2>/dev/null || true

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

PIDS=()
cleanup() {
  echo
  echo "[run_sim] shutting down pipeline..."
  kill "${PIDS[@]}" 2>/dev/null || true
  # children survive (pid:host); kill the node processes directly
  for p in "ros2 launch clearpath_gz" "ros2 launch clearpath_nav2_demos" \
           "ros2 launch evo_skill_ros" "ign gazebo" "/ros_gz_bridge/" \
           "/ros_gz_image/" nav2_ slam_toolbox "evo_skill_ros/lib" \
           scan_relay.py robot_state_publisher robot_localization/ekf_node; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  echo "[run_sim] done."
}
trap cleanup INT TERM EXIT

wait_for() { # description  test-command...
  local desc=$1; shift
  echo -n "[run_sim] waiting for $desc"
  until "$@" >/dev/null 2>&1; do echo -n "."; sleep 3; done
  echo " OK"
}

echo "[run_sim] NS=$NS WORLD=$WORLD TARGET=$TARGET  (logs -> $LOGDIR)"

# 1) Gazebo + robot ----------------------------------------------------------
ros2 launch clearpath_gz simulation.launch.py world:="$WORLD" rviz:="$RVIZ" \
  > "$LOGDIR/sim.log" 2>&1 &
PIDS+=($!)
wait_for "robot to spawn in Gazebo" \
  bash -c "ign model --list 2>/dev/null | grep -q '${NS#/}/robot'"

# 2) Nav2 + SLAM -------------------------------------------------------------
ros2 launch clearpath_nav2_demos nav2.launch.py \
  use_sim_time:=true setup_path:="$HOME/clearpath/" > "$LOGDIR/nav2.log" 2>&1 &
PIDS+=($!)
ros2 launch clearpath_nav2_demos slam.launch.py \
  use_sim_time:=true setup_path:="$HOME/clearpath/" > "$LOGDIR/slam.log" 2>&1 &
PIDS+=($!)

# 3) Scan relay: feed the 3D lidar scan into the 2D topic SLAM/Nav2 expect ----
python3 "$WS/scan_relay.py" "$LIDAR3D" "$LIDAR2D" > "$LOGDIR/relay.log" 2>&1 &
PIDS+=($!)

# 4) Wait for Nav2 to finish lifecycle activation ----------------------------
wait_for "Nav2 to activate (waypoint_follower)" \
  bash -c "[ \"\$(ros2 lifecycle get ${NS}/waypoint_follower 2>/dev/null)\" = 'active [3]' ]"

if [ "${NO_EVO:-0}" = "1" ]; then
  echo "[run_sim] NO_EVO=1 -> skipping evo_skill. Sim+Nav2+SLAM are up."
else
  # 5) evo_skill plan deploy -------------------------------------------------
  EVO_CFG="$(ros2 pkg prefix evo_skill_ros)/share/evo_skill_ros/config"
  ros2 launch evo_skill_ros evo_plan_run.launch.py \
    namespace:="$NS" robot_name:=jackal_1 target_region:="$TARGET" \
    graph_file:="$EVO_CFG/graph.json" \
    domain_file:="$EVO_CFG/factory_sim_domain.pddl" \
    plan_file:="$EVO_CFG/evoskill_plan_sim.txt" \
    tracks_topic:="$NS/tracks" \
    costmap_edit_max_radius:=1.0 require_map:=false \
    json_log_file:="$WS/evo_plan_deploy_log.json" > "$LOGDIR/evo.log" 2>&1 &
  PIDS+=($!)
  echo "[run_sim] evo_skill plan deploy launched (target_region=$TARGET)."
fi

echo "[run_sim] pipeline is UP. Logs: $LOGDIR/{sim,nav2,slam,relay,evo}.log"
echo "[run_sim] verify motion:  ign model -m ${NS#/}/robot -p   (run twice)"
echo "[run_sim] Ctrl-C to tear everything down."
wait
