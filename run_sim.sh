#!/usr/bin/env bash
# run_sim.sh — bring up the full simulation pipeline inside the Docker container:
#   Gazebo (Clearpath Jackal) -> Nav2 + SLAM -> scan relay -> evo_skill plan deploy
#
# Run this INSIDE the container (after `docker compose run --rm ros_humble`).
# On the HOST first allow the GUI:  xhost +local:
#
# Env overrides: NS (default: auto-detected from ~/clearpath/robot.yaml), WORLD,
#   PLAN (mission plan file; default the packaged config/plan.txt),
#   TARGET (default: derived from PLAN's last (move ...) action),
#   COSTMAP_EDIT_RADIUS (0.3) / STL_REPLAN_COOLDOWN (5.0) — the tuned
#     batch-harness values; the launch defaults 1.0/2.0 make phantom-person
#     costmap discs corridor-wide and cancel/replan cycles rapid,
#   WS, LOGDIR, NO_EVO=1 (skip evo), NO_YOLO=1 (skip YOLO),
#   YOLO_CLASSES (default "person"; comma-list for more), RVIZ=true,
#   MULTICAM=1 (YOLO+tracker on 4 cameras for ~360deg detection),
#   UNTIL_SUCCESS=1 (stop+report when goal reached; set METRICS_DURATION as backstop),
#   SPAWN_X / SPAWN_Y (robot Gazebo spawn, default -0.2 / 1.0 — 1 m from nearest shelf),
#   SPAWN_TIMEOUT (default 180s), NAV2_TIMEOUT (default 180s), YOLO_TIMEOUT (90s),
#   OBS_LOG=true (JSONL snapshots of robot pose + detected objects, plus a
#     cumulative per-region expected-vs-observed belief map, for plan/perception
#     desync), OBS_LOG_FILE, OBS_BELIEF_FILE, OBS_LOG_PERIOD (SIM s, default 1.0),
#     OBS_LOG_CAMERAS (default 0,1,2,3 when MULTICAM=1), OBS_LOG_CLASSES (default: all)
set -o pipefail   # NOTE: no 'set -u' — ROS setup.bash references unbound vars

# Workspace = directory containing this script, like run_ablation.sh:37 and
# run_experiments_par.sh:24. This used to be $HOME/autonomy_stack_ros_humble,
# which only happens to be right inside the container (HOME=/home/user); on the
# host it pointed at a nonexistent path and every $WS-relative lookup failed.
WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}

# This script drives a bringup that only exists inside the container. Fail here
# with one clear message instead of letting `source` of a missing setup.bash
# fall through into a pile of "ros2: command not found".
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[run_sim] ERROR: /opt/ros/humble/setup.bash not found."
  echo "[run_sim] run_sim.sh must run INSIDE the ROS 2 Humble container, not on the host:"
  echo "[run_sim]   docker compose run --rm ros_humble"
  echo "[run_sim]   cd /home/user/autonomy_stack_ros_humble && OBS_LOG=true ./run_sim.sh"
  echo "[run_sim] (To drive trials from the host instead, use ./run_ablation.sh.)"
  exit 1
fi

# robot namespace: use $NS if set, else auto-detect from ~/clearpath/robot.yaml
if [ -z "${NS:-}" ]; then
  NS_DET=$(grep -m1 -E '^[[:space:]]*namespace:' "$HOME/clearpath/robot.yaml" 2>/dev/null | awk '{print $2}')
  NS=${NS_DET:+/$NS_DET}; NS=${NS:-/j100_0000}
fi
WORLD="${WORLD:-$WS/worlds/warehouse_people}"
# Mission plan. run_experiments_par.sh passes PLAN=<factory_mission_N.txt>, which
# this script used to IGNORE: plan_file was hardcoded to /home/user/plan.txt,
# which does not exist, so evo_plan_deploy's resolve_path() silently fell back to
# the packaged config/plan.txt by basename and every mission ran the same plan.
PLAN=${PLAN:-$WS/src/planning_ros_pkgs/evo_skill_ros/config/plan.txt}
[ -s "$PLAN" ] || { echo "[run_sim] ERROR: plan file missing: $PLAN"; exit 1; }
# Goal region = destination of the plan's last (move ...), derived the same way
# as run_ablation.sh:62-64. A hardcoded default drifts from the plan: this was
# R10 while the packaged plan.txt ends at R11.
if [ -z "${TARGET:-}" ]; then
  # Case-INSENSITIVE: the hand-written mission plans use "R5", but Fast Downward
  # emits lowercase "r5", so an FD-generated plan (e.g. the region-coverage
  # tour) matched nothing and died with "no (move ...) action" despite being a
  # perfectly good plan. The executor lowercases region names anyway.
  TARGET=$(grep -oEi '\(move +[[:alnum:]_]+ +r[0-9]+ +r[0-9]+\)' "$PLAN" 2>/dev/null \
           | tail -1 | grep -oEi 'r[0-9]+' | tail -1)
  [ -n "$TARGET" ] || { echo "[run_sim] ERROR: no (move ...) action in $PLAN"; exit 1; }
  echo "[run_sim] target region derived from $(basename "$PLAN"): $TARGET"
fi
RVIZ=${RVIZ:-false}
# Default spawn is 1 m away from shelf_7 (nearest shelf, centred at 0.4,-2.0).
SPAWN_X=${SPAWN_X:--0.2}
SPAWN_Y=${SPAWN_Y:-1.0}
MULTICAM=${MULTICAM:-1}
# Observation logger (plan-vs-perception desync). Off by default so existing
# runs are untouched; each logged camera adds a PointCloud2 subscription.
OBS_LOG=${OBS_LOG:-false}
if [ "${MULTICAM:-0}" = "1" ]; then OBS_LOG_CAMERAS=${OBS_LOG_CAMERAS:-0,1,2,3}
else                                OBS_LOG_CAMERAS=${OBS_LOG_CAMERAS:-0}; fi
LIDAR3D=${LIDAR3D:-${NS}/sensors/lidar3d_0/scan}   # sim VLP16 flattened scan (has a publisher)
LIDAR2D=${LIDAR2D:-${NS}/sensors/lidar2d_0/scan}   # topic Clearpath SLAM/Nav2 subscribe to
LOGDIR=${LOGDIR:-/tmp/evo_sim}
mkdir -p "$LOGDIR"

# --- Fixes for this multi-NIC host (see README "Troubleshooting") ---
export ROS_LOCALHOST_ONLY=1     # pin all ROS2 DDS to loopback; else Nav2 lifecycle hangs
export IGN_IP=127.0.0.1         # pin gz-transport to loopback; else the robot never spawns
# Gazebo must render on the machine-local X server (same reasoning as
# run_ablation.sh:75-80). Over `ssh -Y` the shell's DISPLAY is a FORWARDED
# display like "localhost:10.0", which docker-compose.yml:21 passes straight into
# the container; the container has no X cookie for it -> "X11 connection rejected
# because of wrong authentication", and it would tunnel GPU camera rendering over
# the network. `${DISPLAY:-:1}` could not catch this because DISPLAY is set, just
# set to the wrong thing. A local display starts with ':'; anything else has a
# host part and is remote.
SIM_DISPLAY=${SIM_DISPLAY:-:1}
case "${DISPLAY:-}" in
  :*) ;;
  "") DISPLAY=$SIM_DISPLAY ;;
  *)  echo "[run_sim] DISPLAY=$DISPLAY is a forwarded display (ssh -X/-Y);"
      echo "[run_sim] using $SIM_DISPLAY so Gazebo renders on the local GPU."
      DISPLAY=$SIM_DISPLAY ;;
esac
export DISPLAY
# An ssh session points XAUTHORITY at a host path that shadows local access.
unset XAUTHORITY 2>/dev/null || true

_xsock=${DISPLAY#:}; _xsock=/tmp/.X11-unix/X${_xsock%%.*}
if [ ! -S "$_xsock" ]; then
  echo "[run_sim] WARNING: $_xsock missing -- no X server on $DISPLAY."
  echo "[run_sim] Gazebo will fail to render. On the HOST run: sudo bash start_x1.sh"
fi

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
  x:="$SPAWN_X" y:="$SPAWN_Y" \
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

# 2) Nav2 + localization ------------------------------------------------------
# nav2_custom.launch.py, not clearpath_nav2_demos/nav2.launch.py: it loads the
# tuned j100_nav2.yaml. This is what density_sweep.py:510 launches, i.e. the
# configuration every result in results/ was produced with.
launch_nav2() {
  # Rotate the log instead of appending to it. nav2_active() decides readiness
  # by grepping this file for "Creating bond timer", so an appended log lets a
  # marker from a PREVIOUS attempt -- or, worse, a previous RUN that reused this
  # log directory -- satisfy the check instantly. Observed: the grep matched a
  # line written 8 minutes earlier by an unrelated run, run_sim.sh declared Nav2
  # up, the executor launched and published its waypoints 5.7 s BEFORE
  # waypoint_follower activated, the FollowWaypoints goal was rejected, and the
  # robot never moved while every other part of the stack looked healthy.
  # Rotating rather than truncating keeps the failed attempt for post-mortem.
  if [ -s "$LOGDIR/nav2.log" ]; then
    mv "$LOGDIR/nav2.log" "$LOGDIR/nav2.log.$(date +%s)" 2>/dev/null || true
  fi
  : > "$LOGDIR/nav2.log"
  ros2 launch "$WS/nav2_custom.launch.py" \
    use_sim_time:=true setup_path:="$HOME/clearpath/" >> "$LOGDIR/nav2.log" 2>&1 &
  NAV2_PID=$!
  PIDS+=("$NAV2_PID")
}
# Kill a half-activated Nav2 hard: SIGTERM on the launch pid leaves the servers
# behind, and a second lifecycle_manager talking to orphaned servers never bonds.
# Same node list density_sweep.py:512-520 pkills.
kill_nav2() {
  kill -TERM "$NAV2_PID" 2>/dev/null || true
  for pat in nav2_custom.launch nav2_bringup bt_navigator planner_server \
             controller_server waypoint_follower behavior_server \
             smoother_server velocity_smoother lifecycle_manager_navigation; do
    # NOTE: lifecycle_manager_navigation, NOT bare "lifecycle_manager".
    # pkill -f matches substrings, so the bare name also killed
    # lifecycle_manager_LOCALIZATION -- taking down map_server and AMCL with
    # it. Attempt 1 would fail on the ordinary bond flake, this retry would
    # then destroy localization, and every later attempt was doomed:
    #   "Can't update static costmap layer, no map received"
    #   "Timed out waiting for transform from base_link to map"
    # i.e. the retry made the failure permanent instead of recovering from it.
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 3
}
launch_nav2
# AMCL against the prebuilt map, NOT slam_toolbox. This matches process.md:29
# and density_sweep.py:443 (the path run_ablation.sh drives, and the only one
# actually exercised). With slam.launch.py, slam_toolbox comes up but never
# publishes a map->odom transform here, so the global costmap fails with
# 'Invalid frame ID "map" passed to canTransform' and Nav2 never reaches
# active -- surfacing as a bringup timeout with no obvious cause.
# Set LOCALIZATION_MAP to use a different map (e.g. the real lab).
LOCALIZATION_MAP=${LOCALIZATION_MAP:-$WS/factory_sim_map.yaml}
if [ ! -f "$LOCALIZATION_MAP" ]; then
  echo "[run_sim] ERROR: localization map not found: $LOCALIZATION_MAP" >&2
  exit 1
fi
echo "[run_sim] localizing against $LOCALIZATION_MAP (AMCL, not SLAM)"
ros2 launch clearpath_nav2_demos localization.launch.py \
  map:="$LOCALIZATION_MAP" \
  use_sim_time:=true setup_path:="$HOME/clearpath/" > "$LOGDIR/slam.log" 2>&1 &
PIDS+=($!)

# AMCL publishes no map->odom transform until it is seeded with a pose, so
# without this Nav2 stalls exactly as it does under SLAM. Wait for AMCL to ask
# for one, then seed it at the spawn point (density_sweep.py:471-490).
publish_initial_pose() {
  local cov="0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, \
0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685"
  timeout 20 ros2 topic pub --once "$NS/initialpose" \
    geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $SPAWN_X, y: $SPAWN_Y, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [$cov]}}" >/dev/null 2>&1
}
(
  # AMCL logs this once it is active and still unlocalized.
  for _ in $(seq 1 60); do
    grep -q "Please set the initial pose" "$LOGDIR/slam.log" 2>/dev/null && break
    sleep 1
  done
  # Publish until AMCL confirms with initialPoseReceived, not just once.
  # `ros2 topic pub --once` returns as soon as it has sent, which is routinely
  # before discovery has connected it to AMCL's subscription -- the send
  # "succeeds" and the message goes nowhere. density_sweep.py:503 verifies the
  # same way and treats a missing confirmation as a setup failure.
  for attempt in $(seq 1 10); do
    publish_initial_pose
    for _ in $(seq 1 6); do
      if grep -q "initialPoseReceived" "$LOGDIR/slam.log" 2>/dev/null; then
        echo "[run_sim] AMCL accepted initial pose at ($SPAWN_X, $SPAWN_Y) (map->odom TF valid)"
        exit 0
      fi
      sleep 1
    done
    echo "[run_sim] AMCL has not confirmed the initial pose; retrying ($attempt/10)"
  done
  echo "[run_sim] WARNING: AMCL never confirmed an initial pose -- Nav2 will not activate." >&2
) &
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
# Detect activation from nav2.log, not `ros2 lifecycle get`. The lifecycle CLI
# needs its own DDS discovery round trip and does not reliably answer inside the
# container -- it kept timing out while nav2.log plainly showed every server
# bonded, so a fully-active Nav2 read as a bringup failure. The bond timer is
# logged by lifecycle_manager_navigation only after ALL nav nodes have bonded,
# which is exactly the condition we want; density_sweep.py:537 uses the same
# marker (PAT_NAV2_BONDED).
nav2_active() {
  grep -q "Creating bond timer" "$LOGDIR/nav2.log" 2>/dev/null && return 0
  # Fallback for anyone running with a pre-existing Nav2 that logs elsewhere.
  [ "$(timeout 8 ros2 lifecycle get "${NS}/waypoint_follower" 2>/dev/null)" = 'active [3]' ]
}
# Retry Nav2, don't just time out. Its lifecycle activation has a well-known
# race where the change_state RESPONSE is dropped by DDS -- the server logs
# "failed to send response to .../change_state (timeout)" having actually
# configured fine, and lifecycle_manager then waits forever for a transition
# that already happened, so nothing ever bonds. It is intermittent and clears on
# a clean relaunch; density_sweep.py:70 calls it "the Nav2 bond flake" and
# allows NAV2_RETRIES=3. Without this a trial fails on a coin flip.
NAV2_RETRIES=${NAV2_RETRIES:-3}
NAV2_OK=0
for nav2_try in $(seq 1 "$NAV2_RETRIES"); do
  if wait_for "${NAV2_TIMEOUT:-180}" "Nav2 to activate (attempt $nav2_try/$NAV2_RETRIES)" nav2_active; then
    NAV2_OK=1; break
  fi
  if grep -q "failed to send response.*change_state" "$LOGDIR/nav2.log" 2>/dev/null; then
    echo "[run_sim] Nav2 hit the change_state bond flake; relaunching."
  else
    echo "[run_sim] Nav2 did not activate; relaunching."
  fi
  [ "$nav2_try" -lt "$NAV2_RETRIES" ] || break
  kill_nav2
  launch_nav2
done
if [ "$NAV2_OK" != "1" ]; then
  echo "[run_sim] ERROR: Nav2 did not activate after $NAV2_RETRIES attempts."
  echo "[run_sim] Check $LOGDIR/nav2.log and $LOGDIR/slam.log"
  echo "[run_sim] Common causes: scan relay not feeding ${NS}/sensors/lidar2d_0/scan (no map),"
  echo "[run_sim] AMCL never seeded (no map->odom TF), or DDS dropping traffic."
  exit 1
fi

if [ "${NO_EVO:-0}" = "1" ]; then
  echo "[run_sim] NO_EVO=1 -> skipping evo_skill. Sim+Nav2+SLAM are up."
else
  # 5) evo_skill plan deploy -------------------------------------------------
  EVO_CFG="$(ros2 pkg prefix evo_skill_ros)/share/evo_skill_ros/config"
  EVO_ARGS=()
  # MULTICAM: evo's built-in tracker covers camera_0 via yolo_0 (cams 1-3 below)
  [ "${MULTICAM:-0}" = "1" ] && EVO_ARGS+=("tracking_topic:=/yolo_0/tracking")
  # ros2 launch rejects a bare 'name:=' (malformed argument), so an empty class
  # allowlist must be omitted entirely and left to the launch default ("" = all).
  [ -n "${OBS_LOG_CLASSES:-}" ] && EVO_ARGS+=("obs_log_classes:=${OBS_LOG_CLASSES}")
  [ -n "${OBS_EXCLUDE_FILE:-}" ] && EVO_ARGS+=("obs_exclude_file:=${OBS_EXCLUDE_FILE}")
  # Run-termination knobs. Unset leaves the launch defaults (1800s / 10s / off).
  [ -n "${MISSION_TIMEOUT_S:-}" ] && EVO_ARGS+=("mission_timeout_s:=${MISSION_TIMEOUT_S}")
  [ -n "${PLAN_EXHAUSTION_GRACE_S:-}" ] && EVO_ARGS+=("plan_exhaustion_grace_s:=${PLAN_EXHAUSTION_GRACE_S}")
  [ -n "${END_ON_REPLANS_EXHAUSTED:-}" ] && EVO_ARGS+=("end_on_replans_exhausted:=${END_ON_REPLANS_EXHAUSTED}")
  # FIND_OBJECT: two-phase mission (tour, then approach whatever region the
  # observation log says holds this class). Needs OBS_LOG=true to have a log
  # to search, and the class must be in YOLO_CLASSES to ever be detected.
  if [ -n "${FIND_OBJECT:-}" ]; then
    EVO_ARGS+=("find_object_class:=${FIND_OBJECT}")
    [ -n "${FIND_OBJECT_MIN_HITS:-}" ] && EVO_ARGS+=("find_object_min_hits:=${FIND_OBJECT_MIN_HITS}")
    [ -n "${FIND_OBJECT_MIN_SCORE:-}" ] && EVO_ARGS+=("find_object_min_score:=${FIND_OBJECT_MIN_SCORE}")
    # OBS_LOG arrives as either a boolean word or 1/0 -- run_trial.sh passes the
    # numeric form and ros2 launch's IfCondition accepts both, so this guard has
    # to as well or it rejects a perfectly valid run.
    case "$(printf '%s' "${OBS_LOG}" | tr '[:upper:]' '[:lower:]')" in
      1|true|yes|on) ;;
      *) echo "[run_sim] ERROR: FIND_OBJECT=${FIND_OBJECT} needs OBS_LOG enabled (got '${OBS_LOG}'); nothing would be logged to search" >&2
         exit 1 ;;
    esac
    case ",${YOLO_CLASSES:-}," in
      *",${FIND_OBJECT},"*) ;;
      *) echo "[run_sim] WARNING: '${FIND_OBJECT}' is not in YOLO_CLASSES='${YOLO_CLASSES:-}'; it can never be detected" >&2 ;;
    esac
  fi

  # --- Phi_mob shield + online symbolic replan ------------------------------
  # SHIELD=1 turns on Phi_mob monitoring (reporting only).
  # SYMBOLIC_REPLAN=1 additionally lets it escalate to the host replan service.
  [ "${SHIELD:-0}" = "1" ] && EVO_ARGS+=("enable_phi_mob_shield:=true")
  if [ "${SYMBOLIC_REPLAN:-0}" = "1" ]; then
    REPLAN_URL="${REPLAN_SERVICE_URL:-http://127.0.0.1:${REPLAN_PORT:-8077}}"
    # The service runs on the HOST; this container reaches it on localhost
    # because docker-compose.yml sets network_mode: host.
    if wait_for 20 "replan service at $REPLAN_URL" curl -sf "$REPLAN_URL/health"; then
      # mission_id selects the base PDDL problem, and must match the plan being
      # executed or the replan solves a different mission. Derived from the plan
      # filename, the same way TARGET is derived from its last (move ...).
      MISSION_ID="${MISSION_ID:-$(basename "$PLAN" .txt)}"
      EVO_ARGS+=("enable_phi_mob_shield:=true")
      EVO_ARGS+=("enable_symbolic_replan:=true")
      EVO_ARGS+=("replan_service_url:=$REPLAN_URL")
      EVO_ARGS+=("mission_id:=$MISSION_ID")
      EVO_ARGS+=("planner_mode:=${PLANNER_MODE:-evoplan}")
      EVO_ARGS+=("max_symbolic_replans:=${MAX_SYMBOLIC_REPLANS:-2}")
      EVO_ARGS+=("symbolic_replan_deadline_s:=${SYMBOLIC_DEADLINE:-45.0}")
      echo "[run_sim] symbolic replan ON (mission=$MISSION_ID mode=${PLANNER_MODE:-evoplan})"
    else
      # Deliberately not fatal. Aborting a 20-minute sim bringup because a host
      # helper is down is the wrong trade -- run without Tier 2 and say so.
      echo "[run_sim] WARNING replan service unreachable at $REPLAN_URL;" \
           "continuing with symbolic replan DISABLED" >&2
      EVO_ARGS+=("enable_symbolic_replan:=false")
    fi
  fi
  ros2 launch evo_skill_ros evo_plan_run.launch.py \
    namespace:="$NS" robot_name:=jackal_1 target_region:="$TARGET" \
    graph_file:="$EVO_CFG/graph.json" \
    domain_file:="$EVO_CFG/factory_sim_domain.pddl" \
    plan_file:="$PLAN" \
    tracks_topic:="$NS/tracks" \
    costmap_edit_max_radius:="${COSTMAP_EDIT_RADIUS:-0.3}" \
    stl_replan_cooldown_s:="${STL_REPLAN_COOLDOWN:-5.0}" \
    require_map:=false \
    enable_metrics:="${METRICS:-false}" \
    metrics_world:="${METRICS_WORLD:-warehouse}" \
    metrics_duration:="${METRICS_DURATION:-0}" \
    metrics_stop_on_success:="${UNTIL_SUCCESS:-false}" \
    metrics_actors_sdf:="${WORLD}.sdf" \
    metrics_json_out:="${METRICS_JSON_OUT:-$WS/scand_metrics_out.json}" \
    "${EVO_ARGS[@]}" \
    enable_observation_log:="${OBS_LOG}" \
    obs_log_file:="${OBS_LOG_FILE:-$WS/observations.jsonl}" \
    obs_belief_file:="${OBS_BELIEF_FILE:-$WS/belief.json}" \
    obs_log_period_s:="${OBS_LOG_PERIOD:-1.0}" \
    obs_log_cameras:="${OBS_LOG_CAMERAS}" \
    json_log_file:="${JSON_LOG_FILE:-$WS/evo_plan_deploy_log.json}" > "$LOGDIR/evo.log" 2>&1 &
  PIDS+=($!)
  echo "[run_sim] evo_skill plan deploy launched (target_region=$TARGET, metrics=${METRICS:-false})."
fi

# 6) YOLO detection -> tracker_with_yolo -> /tracks (evo's STL human obstacles).
#    Single front camera by default; MULTICAM=1 runs YOLO + a tracker on all 4
#    cameras (camera_0..3) for ~360 deg detection (heavy: 4x YOLO-world on GPU).
#    yolo-world is open-vocabulary: it detects NOTHING until classes are set.
# Default vocabulary matches the graph.json object types (see
# factory_graph.object_type_from_name), so observation_logger can compare what
# the plan EXPECTS in a region against what was actually seen there.
#
# These do NOT become STL obstacles: tracker_with_yolo forwards everything to
# <NS>/tracks, but evo_plan_deploy discards non-human tracks
# (build_tracked_obstacles, eveo_plan_deploy.py:891) and only inflates the
# costmap for human obstacles (:1114). So extra classes add no new obstacles and
# no new replan triggers. What they DO add is cost: track_callback (:575) re-runs
# the STL monitor and logs an INFO line for every incoming detection, so more
# classes mean more monitor invocations and a noisier evo.log.
# Set YOLO_CLASSES=person for person-only runs; to drop classes from the
# observation log without changing detection, use config/exclude.json instead.
YOLO_CLASSES="${YOLO_CLASSES:-person,chair,table,shelf,column,box,pallet}"
YOLO_DEVICE="${YOLO_DEVICE:-cuda:0}"
# yolo-world.launch.py defaults to yolov8s-worldv2.pt -- the SMALLEST World
# checkpoint. A run that drove to within 1.42 m of a confirmed-spawned
# testobj_bookshelf_r5 logged 83 'chair' and 1 'person' (both COCO-core) and
# ZERO 'bookshelf' at any score, which is what open-vocab underfitting looks
# like at the 's' scale. Medium is the next step up.
#
# This MUST stay a *-worldv2 (or other open-vocab) checkpoint. yolo_node.py:148
# creates the set_classes service only `if isinstance(self.yolo, YOLOWorld)`, so
# a plain yolov8m.pt silently ignores YOLO_CLASSES, runs COCO's 80 classes --
# which contain no 'bookshelf' -- and makes find-object missions unsatisfiable.
YOLO_MODEL="${YOLO_MODEL:-yolov8m-worldv2.pt}"
case "$YOLO_MODEL" in
  *world*|*yoloe*) ;;
  *) echo "[run_sim] WARNING: YOLO_MODEL='$YOLO_MODEL' is not an open-vocabulary" \
          "checkpoint; set_classes will not exist and YOLO_CLASSES will be IGNORED" >&2 ;;
esac
set_classes_bg() {   # <yolo_namespace> <log_file>  (prompt yolo-world with the target classes)
  local yns="$1" log="$2"
  (
    # Call the service directly instead of first waiting for it to appear in
    # `ros2 service list`. Listing enumerates the ENTIRE DDS graph, which with
    # Gazebo + Nav2 + 4 YOLO stacks + trackers + metrics takes longer than the
    # 6s the old probe allowed -- so the probe timed out, found nothing, and
    # after 90s reported "/yolo_N/set_classes never appeared" while the service
    # was in fact advertised the whole time (verified: model_type=World and
    # /set_classes present). yolo-world detects NOTHING until prompted, so this
    # silently left every camera running the default COCO vocabulary: the
    # backpack in the objects world came back labelled "suitcase", and
    # shelf/box/pallet were never detectable at all.
    #
    # `ros2 service call` blocks until that one service resolves, which needs
    # only its own discovery, not the whole graph.
    local deadline=$(( SECONDS + ${YOLO_TIMEOUT:-90} ))
    local ok=0
    while [ $SECONDS -lt $deadline ]; do
      if timeout 20 ros2 service call "/${yns}/set_classes" yolo_msgs/srv/SetClasses \
           "{classes: [${YOLO_CLASSES}]}" >/dev/null 2>&1; then
        ok=1; break
      fi
      # The response is frequently lost over DDS even though the node applied
      # the classes, so treat the node's own log as the source of truth.
      if grep -qaE "Setting classes|New classes" "$log" 2>/dev/null; then
        ok=1; break
      fi
      sleep 3
    done
    sleep 2
    if grep -qaE "Setting classes|New classes" "$log" 2>/dev/null; then
      echo "[run_sim] ${yns} classes set: [${YOLO_CLASSES}]"
    elif [ "$ok" = 1 ]; then
      echo "[run_sim] ${yns} set_classes call returned but the node did not log it; check $log"
    else
      echo "[run_sim] WARNING: ${yns} set_classes never succeeded -- detector is running" \
           "its DEFAULT vocabulary, not [${YOLO_CLASSES}]. check $log"
    fi
  ) &
  PIDS+=($!)
}
if [ "${NO_YOLO:-0}" = "1" ]; then
  echo "[run_sim] NO_YOLO=1 -> skipping YOLO (people won't register as STL obstacles)."
elif [ "${MULTICAM:-0}" = "1" ]; then
  echo "[run_sim] MULTICAM=1 -> YOLO + tracker on camera_0..3 (~360 deg). Heavy GPU load."
  for i in 0 1 2 3; do
    ros2 launch yolo_bringup yolo-world.launch.py \
      model:="$YOLO_MODEL" \
      input_image_topic:="${NS}/sensors/camera_${i}/color/image" namespace:="yolo_${i}" \
      > "$LOGDIR/yolo_${i}.log" 2>&1 &
    PIDS+=($!)
    set_classes_bg "yolo_${i}" "$LOGDIR/yolo_${i}.log"
  done
  # camera_0's tracker is the one in evo_plan_run (tracking_topic:=/yolo_0/tracking);
  # add trackers for camera_1..3, all publishing to ${NS}/tracks.
  for i in 1 2 3; do
    ros2 run evo_skill_ros tracker_with_yolo --ros-args \
      -r __node:="tracker_with_yolo_cam${i}" -r __ns:="$NS" \
      -p namespace:="$NS" -p tracking_topic:="/yolo_${i}/tracking" \
      -p points_topic:="/sensors/camera_${i}/points" -p out_topic:="/tracks" \
      -p target_frame:=map -r /tf:=tf -r /tf_static:=tf_static \
      > "$LOGDIR/tracker_${i}.log" 2>&1 &
    PIDS+=($!)
  done
  echo "[run_sim] MULTICAM: 4 YOLO + 4 trackers (cam0 via evo) -> ${NS}/tracks."
else
  ros2 launch yolo_bringup yolo-world.launch.py \
    model:="$YOLO_MODEL" \
    input_image_topic:="${NS}/sensors/camera_0/color/image" \
    > "$LOGDIR/yolo.log" 2>&1 &
  PIDS+=($!)
  echo "[run_sim] YOLO launched on ${NS}/sensors/camera_0/color/image -> /yolo/tracking."
  set_classes_bg "yolo" "$LOGDIR/yolo.log"
fi

echo "[run_sim] pipeline is UP. Logs: $LOGDIR/{sim,nav2,slam,relay,evo,yolo}.log"
echo "[run_sim] verify motion:  ign model -m ${NS#/}/robot -p   (run twice)"

# When metrics are collecting with early-stop armed, scand_metrics decides when
# the mission is over: it stops on arrival-and-settled, or on a collision, and
# writes its JSON on the way out. Exit with it instead of blocking on `wait`,
# which held the whole pipeline up until the caller's timeout -- observed idling
# ~9 minutes after a mission that finished in 31 seconds. Batch runs pay that
# per trial.
if [ "${METRICS:-false}" = "true" ] && [ "${UNTIL_SUCCESS:-false}" != "false" ]; then
  METRICS_OUT="${METRICS_JSON_OUT:-$WS/scand_metrics_out.json}"
  echo "[run_sim] running until the mission ends (watching $METRICS_OUT); Ctrl-C to stop early."
  # scand_metrics is launched by evo_plan_run.launch.py, so we cannot wait on
  # its PID; watch for the results file it writes on clean exit.
  START_MTIME=$(stat -c %Y "$METRICS_OUT" 2>/dev/null || echo 0)
  while :; do
    NOW_MTIME=$(stat -c %Y "$METRICS_OUT" 2>/dev/null || echo 0)
    if [ "$NOW_MTIME" -gt "$START_MTIME" ]; then
      echo "[run_sim] mission ended; metrics written to $METRICS_OUT"
      break
    fi
    # If every background job has exited, there is nothing left to wait for.
    kill -0 ${PIDS[0]} 2>/dev/null || { echo "[run_sim] pipeline exited"; break; }
    sleep 2
  done
else
  echo "[run_sim] Ctrl-C to tear everything down."
  wait
fi
