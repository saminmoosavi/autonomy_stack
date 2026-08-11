#!/usr/bin/env bash
# =============================================================================
# Single online-EvoPlan trial: host replan service + Gazebo sim, one command.
#
#   ./pipeline/run_trial.sh
#
# Run this from the HOST. It starts the replan service here (it needs the
# venv, openevolve and VAL, none of which are in the Humble image), then runs
# the sim inside the ROS container. They talk over 127.0.0.1 because
# docker-compose.yml puts the container on network_mode: host.
#
# Every knob is at the top. Override any of them per-run from the environment:
#   PLANNER_MODE=evoplan CROWD=10 ./pipeline/run_trial.sh
# =============================================================================
set -euo pipefail

# ------------------------------- WHAT TO RUN ---------------------------------
MISSION="${MISSION:-factory_mission_08}"   # factory_mission_01 .. _10
CROWD="${CROWD:-30}"                       # 0 10 20 30 40 50 100, or "" for warehouse_people
# fd_only     Fast Downward alone, no LLM, no token spend.
# fd_first    FD, and EvoPlan only if FD's plan fails VAL.
# evoplan     FD FIRST (as a solvability guard, a fallback plan and the
#             length reference EvoPlan is scored against), then evolve.
# evoplan_only  FD is never consulted. The LLM is the only planner, so there is
#             no fallback if it returns nothing -- which is the point when the
#             comparison being run is "what can EvoPlan do on its own".
PLANNER_MODE="${PLANNER_MODE:-fd_first}"    # fd_only | fd_first | evoplan | evoplan_only

# --------------------------------- THE LLM -----------------------------------
# Only consulted when PLANNER_MODE reaches EvoPlan. fd_only never calls out.
#
# The key is NOT stored here -- this file is committed. Put it in
# pipeline/trial.env (gitignored; see trial.env.example), or export it.
_ENV_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/trial.env"
# shellcheck disable=SC1090
[ -f "$_ENV_FILE" ] && set -a && . "$_ENV_FILE" && set +a
OPENAI_API_KEY="${OPENAI_API_KEY:-}"
EVOPLAN_MODEL="${EVOPLAN_MODEL:-gpt-5-mini}"
EVOPLAN_ITERATIONS="${EVOPLAN_ITERATIONS:-6}"

# ------------------------------ ONLINE REPLAN --------------------------------
SYMBOLIC_REPLAN="${SYMBOLIC_REPLAN:-1}"    # 1 = Tier 2 on (implies the shield)
SHIELD="${SHIELD:-1}"                      # Phi_mob monitoring; 1 = on
MAX_SYMBOLIC_REPLANS="${MAX_SYMBOLIC_REPLANS:-2}"
SYMBOLIC_DEADLINE="${SYMBOLIC_DEADLINE:-45.0}"   # WALL seconds per replan
# 45 s is sized for the modes that keep a Fast Downward plan in hand: EvoPlan
# overrunning it costs nothing there, because the FD plan is what gets driven.
# evoplan_only has no such floor. OpenEvolve needs roughly 15-25 s per
# iteration (an LLM round trip plus a VAL evaluation) and the config runs 6, so
# 45 s expires mid-run and the service returns an EMPTY plan -- observed twice
# in one trial, both replans reporting "evoplan produced no plan (timeout)"
# after exactly 45.0 s, leaving the robot to abandon the replan and carry on
# with a route that had just been aborted.
if [ "$PLANNER_MODE" = "evoplan_only" ] && [ "${SYMBOLIC_DEADLINE%.*}" -lt 120 ]; then
  echo "[trial] PLANNER_MODE=evoplan_only has no Fast Downward fallback and" \
       "${SYMBOLIC_DEADLINE}s is not enough for 6 OpenEvolve iterations -- raising to 150.0"
  echo "[trial]   (set SYMBOLIC_DEADLINE explicitly to something >= 120 to silence this)"
  SYMBOLIC_DEADLINE=150.0
fi
# Whole-mission deliberation budget, against which every replan's WALL time is
# charged. It must exceed the per-replan deadline or the first replan spends the
# lot and every later escalation is refused with "mission deliberation budget
# exhausted" -- which is not a graceful degradation: a Nav2 abort then has no
# replan available, the executor is left with no active goal, and the run idles
# to its timeout. Nobody hit this while the default deadline was 45 s (2 x 45 <
# 120); raising it for evoplan_only walks straight into it.
#
# Sized as deadline x replans so the budget is exactly what the other two knobs
# imply, with the 120 s default as a floor so no run gets a SMALLER budget than
# it had before.
if [ -z "${MISSION_DELIBERATION_BUDGET_S:-}" ]; then
  MISSION_DELIBERATION_BUDGET_S="$(awk -v d="$SYMBOLIC_DEADLINE" -v n="$MAX_SYMBOLIC_REPLANS" \
    'BEGIN { b = d * n; if (b < 120) b = 120; printf "%.1f", b }')"
fi
REPLAN_PORT="${REPLAN_PORT:-8077}"

# ROS 2 parameters are strictly typed. evo_plan_deploy declares these as DOUBLE,
# so passing "180" instead of "180.0" kills the node at startup with
# InvalidParameterTypeException -- while run_sim.sh still reports "pipeline is
# UP", so the robot simply never moves and the trial looks healthy until the
# metrics come back empty. Force a decimal point on every float-typed knob.
as_float() { case "$1" in *.*) printf '%s' "$1";; *) printf '%s.0' "$1";; esac; }
SYMBOLIC_DEADLINE="$(as_float "$SYMBOLIC_DEADLINE")"
# Every DOUBLE-typed node parameter must go through as_float. rclpy rejects an
# INTEGER override on a DOUBLE parameter outright -- MISSION_TIMEOUT_S=3600
# killed evo_plan_deploy on startup with InvalidParameterTypeException, and the
# bringup still reported "pipeline is UP" because the other nodes were fine.
[ -n "${MISSION_TIMEOUT_S:-}" ] && MISSION_TIMEOUT_S="$(as_float "$MISSION_TIMEOUT_S")"
[ -n "${PLAN_EXHAUSTION_GRACE_S:-}" ] && PLAN_EXHAUSTION_GRACE_S="$(as_float "$PLAN_EXHAUSTION_GRACE_S")"
MISSION_DELIBERATION_BUDGET_S="$(as_float "$MISSION_DELIBERATION_BUDGET_S")"

# ---------------------------------- THE SIM ----------------------------------
NS="${NS:-/j100_0000}"
ROBOT_YAML="${ROBOT_YAML:-robot_4cam.yaml}"      # 4 cams => ~360deg detection
MULTICAM="${MULTICAM:-1}"                        # must match ROBOT_YAML
RVIZ="${RVIZ:-false}"
OBS_LOG="${OBS_LOG:-0}"                      # 1 = log perceived objects to observations.jsonl
OBS_LOG_PERIOD="${OBS_LOG_PERIOD:-1.0}"      # seconds between observation records
# FIND_OBJECT turns the run into a two-phase find-an-object mission: drive the
# whole tour logging what is seen where, then replan an approach to whichever
# region actually held this class. Forces OBS_LOG on (there would be nothing to
# search otherwise) and must name a class present in YOLO_CLASSES.
FIND_OBJECT="${FIND_OBJECT:-}"
FIND_OBJECT_MIN_HITS="${FIND_OBJECT_MIN_HITS:-3}"       # INTEGER param
FIND_OBJECT_MIN_SCORE="$(as_float "${FIND_OBJECT_MIN_SCORE:-0.5}")"   # DOUBLE param
if [ -n "$FIND_OBJECT" ] && [ "$OBS_LOG" != "1" ]; then
  echo "[trial] FIND_OBJECT=$FIND_OBJECT -- forcing OBS_LOG=1"
  OBS_LOG=1
fi
# INSPECT_OBJECTS turns the run into an OPEN-WORLD find-and-inspect mission:
# survey the regions, cluster what perception saw into individual objects --
# however many there turn out to be -- then drive to each one, face it and hold
# still for INSPECT_DWELL_S. Comma-separated classes, all of which should be in
# YOLO_CLASSES. Use with MISSION=factory_survey_01, whose problem declares no
# objects at all; they are spliced in as they are discovered.
INSPECT_OBJECTS="${INSPECT_OBJECTS:-}"
INSPECT_DWELL_S="$(as_float "${INSPECT_DWELL_S:-5.0}")"                # DOUBLE param
INSPECT_STANDOFF_M="$(as_float "${INSPECT_STANDOFF_M:-0.0}")"          # DOUBLE param
INSPECT_CLUSTER_RADIUS_M="$(as_float "${INSPECT_CLUSTER_RADIUS_M:-5.0}")"  # DOUBLE param
INSPECT_MAX_OBJECTS="${INSPECT_MAX_OBJECTS:-12}"                       # INTEGER param
if [ -n "$INSPECT_OBJECTS" ] && [ "$OBS_LOG" != "1" ]; then
  echo "[trial] INSPECT_OBJECTS=$INSPECT_OBJECTS -- forcing OBS_LOG=1"
  OBS_LOG=1
fi
if [ -n "$INSPECT_OBJECTS" ] && [ -n "$FIND_OBJECT" ]; then
  die "FIND_OBJECT and INSPECT_OBJECTS are different missions (approach one known object vs inspect every object found); set one"
fi
# X display Gazebo/RViz render on. A LOCAL display starts with ':'; anything
# else ("localhost:10.0") is an ssh -X/-Y forwarded display, which the container
# has no cookie for and which would tunnel GPU camera rendering over the
# network. run_sim.sh:91-99 makes the same substitution; without it here the
# preflight dies on "no X server on localhost:10.0" whenever the trial is
# launched from a forwarded session.
case "${SIM_DISPLAY:-${DISPLAY:-}}" in
  :*) SIM_DISPLAY="${SIM_DISPLAY:-$DISPLAY}" ;;
  "") SIM_DISPLAY=":1" ;;
  *)  echo "[trial] DISPLAY=${SIM_DISPLAY:-$DISPLAY} is forwarded; using :1 for local GPU rendering" >&2
      SIM_DISPLAY=":1" ;;
esac
XHOST_FIX="${XHOST_FIX:-1}"                  # 1 = grant the container local X access
SPAWN_X="${SPAWN_X:--0.2}"
SPAWN_Y="${SPAWN_Y:-1.0}"
UNTIL_SUCCESS="${UNTIL_SUCCESS:-1}"              # stop as soon as the goal is reached
METRICS_DURATION="${METRICS_DURATION:-0}"        # 0 = no time cap
TRIAL_TIMEOUT="${TRIAL_TIMEOUT:-1800}"           # hard wall cap on the whole trial

# --------------------------------- RESULTS -----------------------------------
RESULTS_DIR="${RESULTS_DIR:-results/single_trials}"
TAG="${TAG:-$(date +%Y%m%d_%H%M%S)}"
KEEP_SERVICE="${KEEP_SERVICE:-1}"    # 1 = leave the service up for the next trial

# =============================================================================
# Nothing below here needs editing for a normal run.
# =============================================================================
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

# The repo is bind-mounted at this path inside the container, so WORLD and PLAN
# must be CONTAINER paths -- passing $PWD from the host silently points at
# nothing and run_sim.sh falls back to its defaults.
WS_IN_CONTAINER="/home/user/autonomy_stack_ros_humble"
# WORLD_NAME overrides the CROWD-derived name, so worlds outside the pedestrian
# series can be run -- e.g. WORLD_NAME=_variants/warehouse_objects_test for the
# perception scenario. TAG_SUFFIX keeps their results distinguishable from the
# p<CROWD> naming.
WORLD_NAME="${WORLD_NAME:-warehouse_people${CROWD}}"
WORLD_SDF="$REPO/worlds/${WORLD_NAME}.sdf"
PLAN_HOST="$REPO/factory_mission_plans/${MISSION}.txt"

say() { printf '\n\033[1m[trial]\033[0m %s\n' "$*"; }
die() { printf '\n\033[1;31m[trial] ERROR\033[0m %s\n' "$*" >&2; exit 1; }

# ----------------------------- preflight -------------------------------------
say "preflight"
[ -f "$WORLD_SDF" ] || die "no world at $WORLD_SDF (WORLD_NAME=$WORLD_NAME)"
[ -f "$PLAN_HOST" ] || die "no plan at $PLAN_HOST (MISSION=$MISSION)"
[ -f "$REPO/evolve_stl_pddl/jackal/in/${MISSION}.pddl" ] \
  || [ -f "$REPO/pipeline/missions/${MISSION}.pddl" ] \
  || die "no PDDL problem for $MISSION (searched evolve_stl_pddl/jackal/in/ and pipeline/missions/)"
[ -f "$REPO/$ROBOT_YAML" ] || die "no robot config at $REPO/$ROBOT_YAML"
[ -x "$REPO/.venv-evoplan/bin/python" ] || die "no host venv; see pipeline/README or Phase 0 of the plan"
docker compose ps >/dev/null 2>&1 || die "docker compose unavailable from $REPO"

# TARGET comes from the plan's last (move ...), exactly as run_sim.sh derives it,
# and is echoed here so a mission/plan mismatch is visible before the 3-minute
# Gazebo bringup rather than after it.
TARGET="$(grep -oE '\(move[^)]*\)' "$PLAN_HOST" | tail -1 | awk '{print $NF}' | tr -d ')')"
[ -n "$TARGET" ] || die "could not derive a target region from $PLAN_HOST"

if [ "$PLANNER_MODE" != "fd_only" ] && [ -z "$OPENAI_API_KEY" ]; then
  die "PLANNER_MODE=$PLANNER_MODE needs OPENAI_API_KEY. Export it, or use PLANNER_MODE=fd_only."
fi

# --- X access ----------------------------------------------------------------
# Gazebo and RViz are Qt apps that must open the host's X display from inside the
# container. The socket is bind-mounted, but X access control is per-user via a
# MIT-MAGIC-COOKIE the container does not have -- and run_sim.sh deliberately
# unsets XAUTHORITY (an ssh session's cookie path shadows local access). Without
# a grant, `ign gazebo` aborts with "could not connect to display", the server
# never comes up, and the robot-spawn wait burns its full 180s before failing
# with a misleading "no robot under namespace" message.
# Strip the leading ':' and any '.screen' suffix BEFORE building the path --
# the path itself contains dots, so trimming afterwards eats it.
X_NUM="${SIM_DISPLAY#:}"; X_NUM="${X_NUM%%.*}"
X_SOCKET="/tmp/.X11-unix/X${X_NUM}"
[ -S "$X_SOCKET" ] || die "no X server on $SIM_DISPLAY (missing $X_SOCKET). \
Start one with: sudo bash start_x1.sh"

if [ "$XHOST_FIX" = "1" ] && command -v xhost >/dev/null 2>&1; then
  # Local, non-network connections only, and reversible with: xhost -local:
  xhost +local: >/dev/null 2>&1 || true
fi
# The actual "can the container open the display" probe runs inside the
# detached container further down, via docker exec. Spinning a separate
# `compose run` container just to check costs a container start and, without
# -T, hangs on TTY allocation when launched from an interactive shell.

cat <<EOF

  mission     : $MISSION   (target region: $TARGET)
  world       : $WORLD_NAME
  obs log     : $([ "$OBS_LOG" = 1 ] && echo ON || echo off)$([ -n "$FIND_OBJECT" ] && echo "
  find object : $FIND_OBJECT")$([ -n "$INSPECT_OBJECTS" ] && echo "
  inspect     : $INSPECT_OBJECTS   (${INSPECT_DWELL_S}s dwell, max $INSPECT_MAX_OBJECTS objects)")
  planner     : $PLANNER_MODE$([ "$PLANNER_MODE" != fd_only ] && echo "   model: $EVOPLAN_MODEL")
  tier 2      : $([ "$SYMBOLIC_REPLAN" = 1 ] && echo "ON  (max $MAX_SYMBOLIC_REPLANS replans, ${SYMBOLIC_DEADLINE}s deadline)" || echo off)
  shield      : $([ "$SHIELD" = 1 ] && echo ON || echo off)
  display     : $SIM_DISPLAY   (rviz: $RVIZ)
  results     : $RESULTS_DIR/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}.json
EOF

# ------------------------- host: the replan service ---------------------------
# Env is read at boot, so a running service started without the key (or with a
# different model) will not pick up this run's settings -- restart when they
# differ rather than silently using the old ones.
say "replan service on port $REPLAN_PORT"
NEED_RESTART=0
if REPLAN_PORT="$REPLAN_PORT" ./pipeline/replan_service.sh status >/dev/null 2>&1; then
  RUNNING_PID="$(cat "${REPLAN_PIDFILE:-/tmp/evoplan_replan.pid}")"
  RUNNING_KEY="$(tr '\0' '\n' < "/proc/$RUNNING_PID/environ" 2>/dev/null | sed -n 's/^OPENAI_API_KEY=//p')"
  RUNNING_MODEL="$(tr '\0' '\n' < "/proc/$RUNNING_PID/environ" 2>/dev/null | sed -n 's/^EVOPLAN_MODEL=//p')"
  [ "$RUNNING_KEY" = "$OPENAI_API_KEY" ] || { NEED_RESTART=1; echo "  running service has a different OPENAI_API_KEY"; }
  [ "$RUNNING_MODEL" = "$EVOPLAN_MODEL" ] || { NEED_RESTART=1; echo "  running service has EVOPLAN_MODEL='$RUNNING_MODEL', want '$EVOPLAN_MODEL'"; }
  if [ "$NEED_RESTART" = 1 ]; then
    echo "  restarting to apply this run's settings"
    REPLAN_PORT="$REPLAN_PORT" ./pipeline/replan_service.sh stop
  else
    echo "  reusing the running service (settings match)"
  fi
fi
export OPENAI_API_KEY EVOPLAN_MODEL
REPLAN_PORT="$REPLAN_PORT" ./pipeline/replan_service.sh start \
  --evoplan-iterations "$EVOPLAN_ITERATIONS"

REPLAN_URL="http://127.0.0.1:${REPLAN_PORT}"
curl -sf "$REPLAN_URL/health" > /tmp/evoplan_health.json \
  || die "service came up but /health is unreachable"
python3 - /tmp/evoplan_health.json <<'PY'
import json, sys
h = json.load(open(sys.argv[1]))
print(f"  health: val={h['val']} fd={h['fd']} mock={h['mock']} missions={len(h['missions'])}")
if h["mock"]:
    print("  NOTE: service is in MOCK mode -- it will not really plan")
if not h["fd"]:
    sys.exit("  fast-downward missing; fd_only and fd_first cannot work")
PY

# ------------------------- container: the sim run -----------------------------
mkdir -p "$RESULTS_DIR"
RESULT_JSON="$RESULTS_DIR/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}.json"
EVO_LOG_JSON="$RESULTS_DIR/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}_evo_log.json"

# run_sim.sh logs to /tmp/evo_sim by default, which is INSIDE the container and
# is destroyed by --rm -- so a failed bringup leaves nothing to diagnose. Point
# it at the bind-mounted repo instead, where the logs survive the container.
LOGDIR_HOST="$REPO/$RESULTS_DIR/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}_logs"
mkdir -p "$LOGDIR_HOST"

# Container lifecycle copied from run_ablation.sh's activate(), which is the
# known-good pattern here: bring up ONE detached container holding `sleep
# infinity`, then docker exec into it. Doing `compose run <cmd>` per step ties
# the container's life to the command, and tearing it down is what leaves
# gz-transport and DDS state behind between attempts.
#
# -e XAUTHORITY= is load-bearing: docker-compose.yml forwards the host's
# XAUTHORITY, which points at an ssh/gdm cookie path that does not resolve
# inside the container and shadows local access.
CONT="${CONT:-evoplan-trial}"
# `docker rm -f` returns before the daemon has finished removing the container,
# so an immediate `create` with the same name races it and dies with
# "Conflict. The container name is already in use". Wait for the name to
# actually free up.
cleanup_container() {
  docker rm -f "$CONT" >/dev/null 2>&1 || true
  for _ in $(seq 1 30); do
    docker container inspect "$CONT" >/dev/null 2>&1 || return 0
    sleep 0.5
  done
  echo "[trial] WARNING container '$CONT' still present after removal" >&2
}
# INT/TERM as well as EXIT. With EXIT alone, Ctrl-C killed the shell without
# ever removing the container, and because the sim runs INSIDE it -- the
# host-side `docker exec` is only a client -- gazebo, nav2 and the executor
# survived the interrupt, held the GPU and DDS ports, and poisoned the next
# trial. Re-raising after cleanup preserves the 130/143 exit status.
cleanup_and_exit() {
  local sig="$1"
  trap - EXIT INT TERM
  echo "[trial] interrupted ($sig) -- removing container '$CONT'" >&2
  cleanup_container
  trap - "$sig"
  kill -"$sig" $$
}
trap cleanup_container EXIT
trap 'cleanup_and_exit INT'  INT
trap 'cleanup_and_exit TERM' TERM

say "container '$CONT'"
cleanup_container
docker compose run -d --rm --name "$CONT" \
  -e DISPLAY="$SIM_DISPLAY" -e XAUTHORITY= ros_humble sleep infinity >/dev/null \
  || die "could not start the container"

for i in $(seq 1 40); do
  docker exec "$CONT" bash -lc \
    'source /opt/ros/humble/setup.bash >/dev/null 2>&1; timeout 8 ros2 topic list >/dev/null 2>&1' \
    && break
  [ "$i" = 40 ] && die "ros2 never became usable inside the container"
  sleep 3
done
echo "  ros2 ready"

# X probe, now that we have a container anyway. Gazebo and RViz are Qt apps: if
# they cannot open the display they abort, the gz server never comes up, and the
# robot-spawn wait burns its full 180s before failing with a misleading
# "no robot under namespace" message. Catch it here in a second instead.
if ! docker exec "$CONT" bash -lc "xdpyinfo -display $SIM_DISPLAY >/dev/null 2>&1"; then
  die "the container cannot open X display $SIM_DISPLAY -- Gazebo and RViz would
  abort and this would surface as a spurious robot-spawn timeout.
  Fix on the HOST:  xhost +local:      (undo with: xhost -local:)"
fi
echo "  X display $SIM_DISPLAY reachable"

# The planner node imports evoplan_bridge. If that package is not in the
# workspace's install tree, evo_plan_deploy dies at startup with
# ModuleNotFoundError -- and because run_sim.sh launches it in the background
# and only the sim/nav2 waits are gated, the bringup still reports "pipeline is
# UP" while the robot silently never moves. Fail here instead.
if ! docker exec "$CONT" bash -lc \
     "source $WS_IN_CONTAINER/install/setup.bash 2>/dev/null; \
      python3 -c 'import evoplan_bridge' 2>/dev/null"; then
  die "evoplan_bridge is not in the workspace install tree, so evo_plan_deploy
  would die on import and the robot would never move. Build it:
    docker compose run --rm -T ros_humble bash -lc \\
      'source install/setup.bash; colcon build --packages-select evoplan_bridge evo_skill_ros'"
fi
echo "  evoplan_bridge importable"

# ~/clearpath is bind-mounted from the host, so the active robot description is
# staged host-side (same as run_ablation.sh does before its first trial).
cp "$REPO/$ROBOT_YAML" "$HOME/clearpath/robot.yaml"

# docker exec does NOT inherit the host environment -- every variable must be
# named explicitly. YOLO_CLASSES was set on the command line and silently
# dropped, so the cameras ran run_sim.sh's default warehouse-furniture
# vocabulary instead of the requested one and the tour logged no backpacks.
# Forward anything in this pass-through set that the caller actually set,
# rather than adding -e flags one bug at a time.
PASSTHROUGH_ENV=()
for _v in YOLO_CLASSES YOLO_MODEL YOLO_TIMEOUT YOLO_DEVICE NAV2_TIMEOUT NAV2_RETRIES \
          SPAWN_TIMEOUT LOCALIZATION_MAP STL_REPLAN_COOLDOWN COSTMAP_EDIT_RADIUS \
          NO_YOLO NO_EVO OBS_LOG_CLASSES OBS_EXCLUDE_FILE; do
  if [ -n "${!_v:-}" ]; then
    PASSTHROUGH_ENV+=(-e "${_v}=${!_v}")
    echo "[trial] forwarding ${_v}=${!_v}"
  fi
done

say "sim (timeout ${TRIAL_TIMEOUT}s) -- logs: $LOGDIR_HOST"
# Timestamp the start so stale artefacts from a PREVIOUS trial can be told apart
# from this one's. scand_metrics_out.json and evo_plan_deploy_log.json live at
# fixed paths in the repo and are only rewritten on a successful run, so a trial
# that dies during bringup leaves the last good run's files sitting there.
# Copying them unconditionally reported a 30-minute-old success=true for a run
# whose robot never moved -- in a batch that silently corrupts every failed row.
TRIAL_START_EPOCH=$(date +%s)
set +e
# docker exec does NOT inherit the host environment, so every var is named here.
timeout -k 30 "$TRIAL_TIMEOUT" docker exec \
  -e "LOGDIR=$WS_IN_CONTAINER/$RESULTS_DIR/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}_logs" \
  -e "SIM_DISPLAY=$SIM_DISPLAY" \
  -e "WS=$WS_IN_CONTAINER" \
  -e "NS=$NS" \
  -e "WORLD=$WS_IN_CONTAINER/worlds/$WORLD_NAME" \
  -e "PLAN=$WS_IN_CONTAINER/factory_mission_plans/${MISSION}.txt" \
  -e "MISSION_ID=$MISSION" \
  -e "SYMBOLIC_REPLAN=$SYMBOLIC_REPLAN" \
  -e "SHIELD=$SHIELD" \
  -e "PLANNER_MODE=$PLANNER_MODE" \
  -e "REPLAN_SERVICE_URL=$REPLAN_URL" \
  -e "MAX_SYMBOLIC_REPLANS=$MAX_SYMBOLIC_REPLANS" \
  -e "SYMBOLIC_DEADLINE=$SYMBOLIC_DEADLINE" \
  -e "METRICS=true" \
  -e "METRICS_WORLD=warehouse" \
  -e "METRICS_DURATION=$METRICS_DURATION" \
  -e "UNTIL_SUCCESS=$UNTIL_SUCCESS" \
  -e "METRICS_JSON_OUT=$WS_IN_CONTAINER/scand_metrics_out.json" \
  -e "JSON_LOG_FILE=$WS_IN_CONTAINER/evo_plan_deploy_log.json" \
  -e "MULTICAM=$MULTICAM" \
  -e "RVIZ=$RVIZ" \
  -e "SPAWN_X=$SPAWN_X" \
  -e "SPAWN_Y=$SPAWN_Y" \
  -e "OBS_LOG=$OBS_LOG" \
  -e "OBS_LOG_PERIOD=$OBS_LOG_PERIOD" \
  -e "OBS_LOG_FILE=$WS_IN_CONTAINER/$RESULTS_DIR/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}_observations.jsonl" \
  -e "OBS_BELIEF_FILE=$WS_IN_CONTAINER/$RESULTS_DIR/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}_belief.json" \
  -e "MISSION_TIMEOUT_S=${MISSION_TIMEOUT_S:-}" \
  -e "MISSION_DELIBERATION_BUDGET_S=$MISSION_DELIBERATION_BUDGET_S" \
  -e "PLAN_EXHAUSTION_GRACE_S=${PLAN_EXHAUSTION_GRACE_S:-}" \
  -e "END_ON_REPLANS_EXHAUSTED=${END_ON_REPLANS_EXHAUSTED:-}" \
  -e "FIND_OBJECT=$FIND_OBJECT" \
  -e "FIND_OBJECT_MIN_HITS=$FIND_OBJECT_MIN_HITS" \
  -e "FIND_OBJECT_MIN_SCORE=$FIND_OBJECT_MIN_SCORE" \
  -e "INSPECT_OBJECTS=$INSPECT_OBJECTS" \
  -e "INSPECT_DWELL_S=$INSPECT_DWELL_S" \
  -e "INSPECT_STANDOFF_M=$INSPECT_STANDOFF_M" \
  -e "INSPECT_CLUSTER_RADIUS_M=$INSPECT_CLUSTER_RADIUS_M" \
  -e "INSPECT_MAX_OBJECTS=$INSPECT_MAX_OBJECTS" \
  -e "ROS_LOCALHOST_ONLY=1" \
  "${PASSTHROUGH_ENV[@]}" \
  "$CONT" bash -lc "cd $WS_IN_CONTAINER && ./run_sim.sh"
SIM_RC=$?
set -e
[ "$SIM_RC" = 124 ] && echo "[trial] WARNING trial hit the ${TRIAL_TIMEOUT}s cap" >&2

# Did the planner node survive? run_sim.sh launches it in the background and
# only gates on the sim and Nav2 waits, so a dead evo_plan_deploy leaves the
# bringup reporting "pipeline is UP" while nothing ever commands the robot.
# This has now bitten twice: once on a missing evoplan_bridge install, once on
# an INTEGER passed to a DOUBLE parameter.
if grep -qE "evo_plan_deploy.*(process has died)|InvalidParameterType|ModuleNotFoundError" \
     "$LOGDIR_HOST/evo.log" 2>/dev/null; then
  echo "[trial] ERROR evo_plan_deploy died -- the robot was never commanded." >&2
  grep -m3 -E "InvalidParameterType|ModuleNotFoundError|process has died" \
    "$LOGDIR_HOST/evo.log" 2>/dev/null | sed 's/^/    /' >&2
fi

# --------------------------------- results ------------------------------------
say "results"
# Only accept artefacts this trial actually produced.
fresh() { [ -f "$1" ] && [ "$(stat -c %Y "$1" 2>/dev/null || echo 0)" -ge "$TRIAL_START_EPOCH" ]; }

if fresh scand_metrics_out.json; then
  cp scand_metrics_out.json "$RESULT_JSON"
else
  echo "  TRIAL FAILED: no metrics were produced by this run." >&2
  if [ -f scand_metrics_out.json ]; then
    echo "  (scand_metrics_out.json on disk is from $(stat -c %y scand_metrics_out.json | cut -c1-19)," >&2
    echo "   before this trial started -- NOT copied, it belongs to an earlier run.)" >&2
  fi
fi
fresh evo_plan_deploy_log.json && cp evo_plan_deploy_log.json "$EVO_LOG_JSON"

if [ -f "$RESULT_JSON" ]; then
  python3 - "$RESULT_JSON" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
g = lambda k, dflt="-": d.get(k, dflt)
print(f"  success            : {g('success')}")
print(f"  route progress     : {g('path_progress_pct')} %")
print(f"  collisions         : {g('collisions')}")
print(f"  duration           : {g('duration_s')} s   (moving {g('moving_time_s')}, held {g('held_time_s')})")
print(f"  symbolic replans   : {g('symbolic_replans')}")
print(f"  deliberation       : {g('deliberation_time_s')} s wall")
print(f"  shield vetoes      : {g('shield_vetoes')}")
print(f"  min human clearance: {g('min_human_clearance_m')} m")
PY
else
  echo "  no metrics JSON produced. Bringup log tails:" >&2
  for f in "$LOGDIR_HOST"/sim.log "$LOGDIR_HOST"/nav2.log "$LOGDIR_HOST"/evo.log; do
    [ -f "$f" ] || continue
    echo "  --- $(basename "$f") ---" >&2
    tail -15 "$f" | sed 's/^/    /' >&2
  done
fi

if [ -f "$EVO_LOG_JSON" ]; then
  echo "  online-replan timeline:"
  python3 - "$EVO_LOG_JSON" <<'PY'
import json, sys
want = ("phi_mob_violation", "symbolic_replan_escalated", "robot_hold_started",
        "symbolic_replan_received", "symbolic_replan_applied", "symbolic_replan_rejected",
        "symbolic_replan_timeout", "symbolic_replan_giveup", "robot_hold_ended",
        "nav2_goal_aborted", "nav2_stuck_detected")
ev = [e for e in json.load(open(sys.argv[1])).get("events", []) if e["event"] in want]
if not ev:
    print("    (none -- Tier 2 never fired this run)")
for e in ev:
    d = e.get("data", {})
    extra = d.get("trigger") or d.get("worst_conjunct") or d.get("status") or ""
    print(f"    t={e['ros_time_sec']:9.2f}s  {e['event']:28s} {extra}")
PY
fi

echo
echo "  metrics : $RESULT_JSON"
echo "  events  : $EVO_LOG_JSON"
echo "  service : $(REPLAN_PORT=$REPLAN_PORT ./pipeline/replan_service.sh status)"

if [ "$KEEP_SERVICE" != "1" ]; then
  REPLAN_PORT="$REPLAN_PORT" ./pipeline/replan_service.sh stop
fi
exit "$SIM_RC"
