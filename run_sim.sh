#!/usr/bin/env bash
# run_sim.sh — bring up the full simulation pipeline inside the Docker container:
#   Gazebo (Clearpath Jackal) -> Nav2 + SLAM -> scan relay -> evo_skill plan deploy
#
# Run this INSIDE the container (after `docker compose run --rm ros_humble`).
# On the HOST first allow the GUI:  xhost +local:
#
# Env overrides: NS (default: auto-detected from ~/clearpath/robot.yaml), WORLD,
#   TARGET, WS, LOGDIR, NO_EVO=1 (skip evo), NO_YOLO=1 (skip YOLO),
#   YOLO_CLASSES (default "person"; comma-list for more), RVIZ=true,
#   SPAWN_TIMEOUT (default 180s), NAV2_TIMEOUT (default 180s), YOLO_TIMEOUT (90s)
set -o pipefail   # NOTE: no 'set -u' — ROS setup.bash references unbound vars

WS=${WS:-$HOME/autonomy_stack_ros_humble}
# robot namespace: use $NS if set, else auto-detect from ~/clearpath/robot.yaml
if [ -z "${NS:-}" ]; then
  NS_DET=$(grep -m1 -E '^[[:space:]]*namespace:' "$HOME/clearpath/robot.yaml" 2>/dev/null | awk '{print $2}')
  NS=${NS_DET:+/$NS_DET}; NS=${NS:-/j100_0000}
fi
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
           "ros2 launch evo_skill_ros" "ros2 launch yolo_bringup" "ign gazebo" \
           "/ros_gz_bridge/" "/ros_gz_image/" nav2_ slam_toolbox "evo_skill_ros/lib" \
           yolo_ros scan_relay.py robot_state_publisher robot_localization/ekf_node; do
    pkill -9 -f "$p" 2>/dev/null || true
  done
  echo "[run_sim] done."
}
trap cleanup INT TERM EXIT

wait_for() { # timeout_s  description  test-command...
  local timeout=$1 desc=$2; shift 2
  local start=$SECONDS
  echo -n "[run_sim] waiting for $desc (timeout ${timeout}s)"
  until "$@" >/dev/null 2>&1; do
    if [ $((SECONDS - start)) -ge "$timeout" ]; then echo " TIMEOUT"; return 1; fi
    echo -n "."; sleep 3
  done
  echo " OK ($((SECONDS - start))s)"
}

# Robust spawn detection: gz model list OR (transport-independent) a ROS
# odometry publisher under the namespace. Either is sufficient.
robot_spawned() {
  # timeout-wrap every CLI call: a hung gz/ros2 discovery call must not stall the loop
  timeout 8 ign model --list 2>/dev/null | grep -q "${NS#/}/robot" && return 0
  local pc
  pc=$(timeout 8 ros2 topic info "${NS}/platform/odom" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')
  [ "${pc:-0}" -ge 1 ]
}

echo "[run_sim] NS=$NS WORLD=$WORLD TARGET=$TARGET  (logs -> $LOGDIR)"

# 1) Gazebo + robot ----------------------------------------------------------
ros2 launch clearpath_gz simulation.launch.py world:="$WORLD" rviz:="$RVIZ" \
  > "$LOGDIR/sim.log" 2>&1 &
PIDS+=($!)
if ! wait_for "${SPAWN_TIMEOUT:-180}" "robot ($NS) to spawn in Gazebo" robot_spawned; then
  echo "[run_sim] ERROR: no robot under namespace '$NS' within timeout."
  echo "[run_sim] Gazebo models actually present:"
  ign model --list 2>/dev/null | sed 's/^/      /'
  echo "[run_sim] If the namespace differs, rerun:  NS=/your_namespace ./run_sim.sh"
  echo "[run_sim] (namespace is the 'namespace:' field in ~/clearpath/robot.yaml,"
  echo "[run_sim]  i.e. the part before '/robot' in the model list above)"
  exit 1
fi

# 2) Nav2 + SLAM -------------------------------------------------------------
ros2 launch clearpath_nav2_demos nav2.launch.py \
  use_sim_time:=true setup_path:="$HOME/clearpath/" > "$LOGDIR/nav2.log" 2>&1 &
PIDS+=($!)
ros2 launch clearpath_nav2_demos slam.launch.py \
  use_sim_time:=true setup_path:="$HOME/clearpath/" > "$LOGDIR/slam.log" 2>&1 &
PIDS+=($!)

# 3) Scan source: SLAM/Nav2 subscribe to lidar2d_0/scan. How that gets fed
#    depends on the robot.yaml sensor block:
#      - lidar2d block (e.g. Hokuyo): publishes lidar2d_0/scan directly -> no relay
#      - lidar3d block (Velodyne):    publishes lidar3d_0/scan          -> relay 3D->2D
#      - neither:                     NO scan at all -> SLAM/Nav2 cannot work
ROBOT_YAML="$HOME/clearpath/robot.yaml"
if grep -qE '^[[:space:]]*lidar2d:' "$ROBOT_YAML" 2>/dev/null; then
  echo "[run_sim] robot.yaml has a 2D lidar -> SLAM/Nav2 use lidar2d_0/scan directly (no relay)."
elif grep -qE '^[[:space:]]*lidar3d:' "$ROBOT_YAML" 2>/dev/null; then
  echo "[run_sim] robot.yaml has a 3D lidar -> relaying $LIDAR3D -> $LIDAR2D"
  python3 "$WS/scan_relay.py" "$LIDAR3D" "$LIDAR2D" > "$LOGDIR/relay.log" 2>&1 &
  PIDS+=($!)
else
  echo "[run_sim] WARNING: no lidar2d/lidar3d sensor in $ROBOT_YAML."
  echo "[run_sim] SLAM cannot build a map and Nav2 will NOT activate. Add a lidar to"
  echo "[run_sim] robot.yaml (see clearpath_config sample a200_dual_laser.yaml or"
  echo "[run_sim] velodyne_lidar.yaml), regenerate setup.bash, and rerun."
fi

# 4) Wait for Nav2 to finish lifecycle activation ----------------------------
if ! wait_for "${NAV2_TIMEOUT:-180}" "Nav2 to activate (waypoint_follower)" \
  bash -c "[ \"\$(timeout 8 ros2 lifecycle get ${NS}/waypoint_follower 2>/dev/null)\" = 'active [3]' ]"; then
  echo "[run_sim] ERROR: Nav2 did not activate. Check $LOGDIR/nav2.log and $LOGDIR/slam.log"
  echo "[run_sim] Common causes: scan relay not feeding ${NS}/sensors/lidar2d_0/scan (no map),"
  echo "[run_sim] or DDS dropping traffic (ensure ROS_LOCALHOST_ONLY=1 everywhere)."
  exit 1
fi

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

# 6) YOLO detection -> /yolo/tracking. tracker_with_yolo turns that into 3D
#    /tracks, which evo's STL layer treats as human obstacles. A person must be
#    in the camera's view to register. Skip with NO_YOLO=1.
if [ "${NO_YOLO:-0}" = "1" ]; then
  echo "[run_sim] NO_YOLO=1 -> skipping YOLO (people won't register as STL obstacles)."
else
  ros2 launch yolo_bringup yolo-world.launch.py \
    input_image_topic:="${NS}/sensors/camera_0/color/image" \
    > "$LOGDIR/yolo.log" 2>&1 &
  PIDS+=($!)
  echo "[run_sim] YOLO launched on ${NS}/sensors/camera_0/color/image -> /yolo/tracking."
  # yolo-world is open-vocabulary: it detects NOTHING until classes are set.
  # Wait for the set_classes service, then prompt it (default: person). Run in
  # the background so it doesn't block bringup.
  YOLO_CLASSES="${YOLO_CLASSES:-person}"
  (
    if wait_for "${YOLO_TIMEOUT:-90}" "YOLO set_classes service" \
        bash -c "timeout 6 ros2 service list 2>/dev/null | grep -q /yolo/set_classes"; then
      timeout 15 ros2 service call /yolo/set_classes yolo_msgs/srv/SetClasses \
        "{classes: [${YOLO_CLASSES}]}" >/dev/null 2>&1 \
        && echo "[run_sim] YOLO classes set: [${YOLO_CLASSES}]" \
        || echo "[run_sim] WARNING: failed to set YOLO classes; people won't be detected."
    else
      echo "[run_sim] WARNING: /yolo/set_classes never appeared; people won't be detected."
    fi
  ) &
  PIDS+=($!)
fi

echo "[run_sim] pipeline is UP. Logs: $LOGDIR/{sim,nav2,slam,relay,evo,yolo}.log"
echo "[run_sim] verify motion:  ign model -m ${NS#/}/robot -p   (run twice)"
echo "[run_sim] Ctrl-C to tear everything down."
wait
