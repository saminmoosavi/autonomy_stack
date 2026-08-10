#!/usr/bin/env python3
"""
density_sweep.py — one trial per density world (warehouse_people{0,10,20,30,40,50}),
collecting SCAND metrics and building per-run + combined LaTeX tables.

Run INSIDE the Docker container:
  cd /home/user/autonomy_stack_ros_humble
  python3 density_sweep.py

Env overrides:
  TARGET_REGION   default: R11
  PLAN_FILE       default: $WS/src/planning_ros_pkgs/evo_skill_ros/config/plan.txt
  RESULTS_DIR     default: $WS/results/density_sweep
  TRIAL_TIMEOUT   default: 300   (seconds from evo launch; also passed as metrics_duration)
  SPAWN_TIMEOUT   default: 120   (seconds to wait for sim.log jackal-ready pattern)
  NAV2_TIMEOUT    default: 150   (seconds to wait for nav2.log bond-timer pattern)
  YOLO_TIMEOUT    default: 120   (seconds to wait for all 4 YOLO set_classes responses)
  WORLDS          default: 0,10,20,30,40,50
  WORLD_NAME      default: ""    (worlds/<WORLD_NAME>.sdf overrides warehouse_people{N};
                                  N then only names the outputs world{N}_*)
  SPAWN_X         default: -0.2  (robot Gazebo spawn X — 1 m from nearest shelf)
  SPAWN_Y         default: 1.0   (robot Gazebo spawn Y)
  NS              default: /j100_0000
  OBS_LOG         default: 0     (1 = write world{N}_observations.jsonl +
                                  world{N}_belief.json for plan/perception desync)
  OBS_LOG_PERIOD  default: 1.0   (snapshot interval, SIM seconds)
  OBS_LOG_CAMERAS default: 0,1,2,3
  OBS_LOG_CLASSES default: ""    (empty = log all YOLO classes)
  OBS_EXCLUDE_FILE default: ""   (empty = evo_skill_ros/config/exclude.json;
                                  JSON class denylist, logging only)
"""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────
_HOME = os.environ.get("HOME", "/home/user")
WS    = Path(os.environ.get("WS", f"{_HOME}/autonomy_stack_ros_humble"))
NS    = os.environ.get("NS", "/j100_0000")

TARGET_REGION = os.environ.get("TARGET_REGION", "R11")
PLAN_FILE     = os.environ.get("PLAN_FILE",
                    str(WS / "src/planning_ros_pkgs/evo_skill_ros/config/plan.txt"))
RESULTS_DIR   = Path(os.environ.get("RESULTS_DIR", str(WS / "results/density_sweep")))

TRIAL_TIMEOUT = int(os.environ.get("TRIAL_TIMEOUT", "300"))
SPAWN_TIMEOUT = int(os.environ.get("SPAWN_TIMEOUT", "120"))
NAV2_TIMEOUT  = int(os.environ.get("NAV2_TIMEOUT",  "150"))
YOLO_TIMEOUT  = int(os.environ.get("YOLO_TIMEOUT",  "120"))

WORLDS_RAW = os.environ.get("WORLDS", "0,10,20,30,40,50")
WORLDS     = [int(x.strip()) for x in WORLDS_RAW.split(",")]

# Explicit world basename (under worlds/, no .sdf). When set it overrides the
# warehouse_people{N} naming so any world file can be run; the WORLDS number
# then only labels the output files (world{N}_metrics.json etc.).
WORLD_NAME = os.environ.get("WORLD_NAME", "")

SPAWN_X = os.environ.get("SPAWN_X", "-0.2")
SPAWN_Y = os.environ.get("SPAWN_Y", "1.0")

MAX_ATTEMPTS  = int(os.environ.get("MAX_ATTEMPTS",  "5"))
NAV2_RETRIES  = int(os.environ.get("NAV2_RETRIES",  "3"))  # Nav2-only restarts before giving up
LOC_RETRIES   = int(os.environ.get("LOC_RETRIES",   "3"))  # localization-only restarts before giving up

# STL knobs (tuned batch-harness values; the launch defaults 1.0/2.0 make
# phantom-person costmap discs corridor-wide and cancel/replan cycles rapid)
COSTMAP_EDIT_RADIUS = os.environ.get("COSTMAP_EDIT_RADIUS", "0.3")
STL_REPLAN_COOLDOWN = os.environ.get("STL_REPLAN_COOLDOWN", "5.0")

# Observation logger (plan-vs-perception desync). Default off: each logged
# camera adds a PointCloud2 subscription to a sim already RTF-capped at 0.5, so
# enabling it must be a deliberate choice measured against a baseline run.
OBS_LOG         = os.environ.get("OBS_LOG", "0")            # "1" to enable
OBS_LOG_PERIOD  = os.environ.get("OBS_LOG_PERIOD", "1.0")   # SIM seconds
OBS_LOG_CAMERAS = os.environ.get("OBS_LOG_CAMERAS", "0,1,2,3")
OBS_LOG_CLASSES = os.environ.get("OBS_LOG_CLASSES", "")     # empty = all
OBS_EXCLUDE_FILE = os.environ.get("OBS_EXCLUDE_FILE", "")   # empty = package default

# metrics_duration counts SIM seconds; with the world RTF capped at 0.5 a
# trial can need ~2x that in wall-clock, so scale the poll deadline.
WALL_FACTOR = float(os.environ.get("WALL_FACTOR", "2.2"))

EVO_CFG = WS / "src/planning_ros_pkgs/evo_skill_ros/config"

USABLE_AREA_M2 = 1214.0   # warehouse navigable floor area (30×47 m minus shelves/walls)

# ─────────────────────────────────────────────────────────────────────────────
# Log-based synchronization patterns
# Tuned to observed log output — adjust here if your ROS/clearpath version differs.
# ─────────────────────────────────────────────────────────────────────────────
# sim.log: gz_ros2_control spawner confirms velocity controller active → robot ready to move.
# NOTE: the spawner writes ANSI colour codes around the controller name ([1m before it),
# so the full "...platform_velocity_controller" string never matches.  Matching on the
# clean prefix "Configured and activated" (which falls before any ANSI escape insertion)
# is sufficient and robust.
PAT_JACKAL_READY    = "Configured and activated"

# localization.log: AMCL is active but has no pose yet → safe to publish /initialpose
PAT_AMCL_NEEDS_POSE = "Please set the initial pose"

# localization.log: AMCL processed the /initialpose message → map→odom TF now valid
PAT_POSE_RECEIVED   = "initialPoseReceived"

# nav2.log: lifecycle_manager_navigation finished bonding all nav nodes → Nav2 fully active
PAT_NAV2_BONDED     = "Creating bond timer"

# yolo.log: each YOLO camera responds to set_classes; appears once per camera (0-3)
# Match the prefix only: start_yolo.sh requests person plus the graph.json object
# types (chair/table/shelf/...) so observation_logger can compare expected vs
# observed, so the node prints "New classes: {0: 'person', 1: 'chair', ...}".
# Anchoring on the closing brace (the old person-only string) never matched and
# failed every bringup at the 120 s YOLO wait.
PAT_YOLO_CLASSES    = "New classes: {0: 'person'"
YOLO_CAMERAS        = 4      # wait for this many occurrences before launching trackers

# ─────────────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────────────
def log(msg: str) -> None:
    print(f"[density_sweep] {msg}", flush=True)

def log_sub(msg: str) -> None:
    print(f"  {msg}", flush=True)

# ─────────────────────────────────────────────────────────────────────────────
# ROS environment capture
# ─────────────────────────────────────────────────────────────────────────────
def capture_ros_env() -> dict:
    log("Capturing ROS environment (sourcing setup.bash files)...")
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {WS}/install/setup.bash 2>/dev/null; "
        f"env -0"
    )
    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
    if result.returncode != 0:
        log(f"  WARNING: env capture returned {result.returncode}; stderr: {result.stderr[:200]}")
    env = {}
    for entry in result.stdout.split("\0"):
        if "=" in entry:
            k, _, v = entry.partition("=")
            env[k] = v
    env.setdefault("ROS_LOCALHOST_ONLY", "1")
    env.setdefault("IGN_IP", "127.0.0.1")
    env.setdefault("DISPLAY", os.environ.get("DISPLAY", ":1"))
    log(f"  Captured {len(env)} vars. ROS_DISTRO={env.get('ROS_DISTRO', '?')}")
    return env

# ─────────────────────────────────────────────────────────────────────────────
# Process management
# ─────────────────────────────────────────────────────────────────────────────
def launch(cmd: str, ros_env: dict, log_file: Path) -> subprocess.Popen:
    """Launch a bash command as a background subprocess."""
    wrapped = (f"source /opt/ros/humble/setup.bash && "
               f"source {WS}/install/setup.bash 2>/dev/null; {cmd}")
    fh = open(log_file, "w")
    proc = subprocess.Popen(
        ["bash", "-c", wrapped],
        env=ros_env,
        stdout=fh,
        stderr=fh,
        start_new_session=True,
    )
    return proc


def kill_all(procs: list, label: str = "") -> None:
    if not procs:
        return
    tag = f" ({label})" if label else ""
    log(f"Shutting down {len(procs)} processes{tag}...")
    for p in procs:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except (ProcessLookupError, OSError):
            pass
    deadline = time.time() + 4
    for p in procs:
        try:
            p.wait(timeout=max(0.1, deadline - time.time()))
        except subprocess.TimeoutExpired:
            pass
    for p in procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
    log("  All processes terminated.")


def kill_stale_ros() -> None:
    log("Killing any stale sim processes...")
    patterns = [
        "ign gazebo", "simulation.launch", "clearpath_nav2_demos",
        "nav2_custom.launch", "evo_plan_run.launch", "yolo-world.launch",
        "yolo_bringup", "tracker_with_yolo", "observation_logger", "scan_relay.py",
        "scand_metrics.py", "ruby",
    ]
    for pat in patterns:
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

# ─────────────────────────────────────────────────────────────────────────────
# Log-based wait helpers
# ─────────────────────────────────────────────────────────────────────────────
def wait_for_log(log_file: Path, pattern: str, timeout_s: int, label: str = "") -> bool:
    """Poll log_file every second until pattern appears or timeout expires."""
    label_str = label or f"'{pattern[:60]}'"
    log(f"  Waiting for {label_str}")
    log_sub(f"  (file: {log_file.name}, timeout: {timeout_s}s)")
    start = time.time()
    while time.time() - start < timeout_s:
        if log_file.exists():
            try:
                if pattern in log_file.read_text(errors="replace"):
                    elapsed = time.time() - start
                    log(f"  [{elapsed:.1f}s] OK — {label_str}")
                    return True
            except OSError:
                pass
        elapsed = int(time.time() - start)
        log_sub(f"  [{elapsed:>4}s] not yet...")
        time.sleep(1)
    log(f"  TIMEOUT ({timeout_s}s) — {label_str} never appeared.")
    return False


def wait_for_log_count(log_file: Path, pattern: str, count: int,
                       timeout_s: int, label: str = "") -> bool:
    """Poll log_file until pattern appears at least `count` times."""
    label_str = label or f"'{pattern[:50]}' ×{count}"
    log(f"  Waiting for {label_str}")
    log_sub(f"  (file: {log_file.name}, timeout: {timeout_s}s)")
    start = time.time()
    last_n = 0
    while time.time() - start < timeout_s:
        if log_file.exists():
            try:
                n = log_file.read_text(errors="replace").count(pattern)
                if n != last_n:
                    log_sub(f"  [{int(time.time()-start):>4}s] {n}/{count} occurrences")
                    last_n = n
                if n >= count:
                    elapsed = time.time() - start
                    log(f"  [{elapsed:.1f}s] OK — {label_str}")
                    return True
            except OSError:
                pass
        time.sleep(1)
    log(f"  TIMEOUT ({timeout_s}s) — only {last_n}/{count} occurrences found.")
    return False

# ─────────────────────────────────────────────────────────────────────────────
# PDDL completion check
# ─────────────────────────────────────────────────────────────────────────────
def check_pddl_success(evo_log: Path) -> bool:
    if not evo_log.exists():
        return False
    try:
        data = json.loads(evo_log.read_text())
        for ev in data.get("events", []):
            if ev.get("event") == "nav2_goal_finished":
                if not ev.get("data", {}).get("missed_waypoints"):
                    return True
    except (json.JSONDecodeError, OSError):
        pass
    return False

# ─────────────────────────────────────────────────────────────────────────────
# Table generation
# ─────────────────────────────────────────────────────────────────────────────
def _fmt(val, prec: int = 1) -> str:
    return "--" if val is None else f"{val:.{prec}f}"


def write_density_table(results_dir: Path, worlds_done: list) -> None:
    rows = []
    for N in worlds_done:
        jf = results_dir / f"world{N}_metrics.json"
        if not jf.exists():
            continue
        try:
            d = json.loads(jf.read_text())
        except (json.JSONDecodeError, OSError):
            continue
        rows.append({
            "N":       N,
            "density": N / USABLE_AREA_M2,
            "succ":    bool(d.get("success")),
            "dur":     d.get("duration_s"),
            "path":    d.get("path_length_m"),
            "coll":    d.get("collisions"),
            "intim":   d.get("intim_pct"),
            "pers":    d.get("pers_pct"),
            "social":  d.get("social_pct"),
            "p_int":   d.get("p_int"),
            "min_clr": d.get("min_human_clearance_m"),
        })
    if not rows:
        log("  No rows to write; skipping table.")
        return
    lines = [
        "% Auto-generated by density_sweep.py",
        "\\begin{table}[t]",
        "\\centering",
        "\\caption{SCAND social-compliance metrics vs.~crowd density "
        "(warehouse, one trial per density level, MULTICAM 360$^\\circ$ detection). "
        "Density = actors / 1214\\,m$^2$ navigable area.}",
        "\\label{tab:density_sweep}",
        "\\begin{tabular}{ccccccccccc}",
        "\\toprule",
        "Actors & Density & Succ & Time\\,[s] & Path\\,[m] & Coll & "
        "Intim\\,\\% & Pers\\,\\% & Social\\,\\% & $P_{\\mathrm{int}}$ & MinClr\\,[m] \\\\",
        "\\midrule",
    ]
    for r in rows:
        row = " & ".join([
            str(r["N"]),
            f"{r['density']:.4f}",
            "\\checkmark" if r["succ"] else "\\texttimes",
            _fmt(r["dur"],  0),
            _fmt(r["path"], 1),
            _fmt(r["coll"], 0),
            _fmt(r["intim"],  1),
            _fmt(r["pers"],   1),
            _fmt(r["social"], 1),
            _fmt(r["p_int"],  1),
            _fmt(r["min_clr"], 2),
        ])
        lines.append(row + " \\\\")
    lines += ["\\bottomrule", "\\end{tabular}", "\\end{table}", ""]
    out = results_dir / "density_table.tex"
    out.write_text("\n".join(lines))
    log(f"  Wrote {out} ({len(rows)} rows).")


def print_summary_table(results_dir: Path, worlds: list) -> None:
    header = (f"{'World':<10} {'Actors':>6} {'Density':>9} {'Succ':>5} "
              f"{'Time[s]':>8} {'Path[m]':>8} {'Coll':>5} {'Social%':>8} {'MinClr':>8}")
    print()
    print(header)
    print("-" * len(header))
    for N in worlds:
        jf = results_dir / f"world{N}_metrics.json"
        if not jf.exists():
            print(f"  world{N:<4}  -- no results --")
            continue
        try:
            d = json.loads(jf.read_text())
        except Exception:
            print(f"  world{N:<4}  -- parse error --")
            continue
        print(
            f"  world{N:<4}  {N:>6}  {N/USABLE_AREA_M2:>9.4f}  "
            f"{'yes' if d.get('success') else 'no':>5}  "
            f"{_fmt(d.get('duration_s'), 0):>8}  {_fmt(d.get('path_length_m'), 1):>8}  "
            f"{_fmt(d.get('collisions'), 0):>5}  "
            f"{_fmt(d.get('social_pct'), 1):>8}  {_fmt(d.get('min_human_clearance_m'), 2):>8}"
        )

# ─────────────────────────────────────────────────────────────────────────────
# Per-trial orchestration
# ─────────────────────────────────────────────────────────────────────────────
class SetupFailed(Exception):
    """Raised when a pre-step-8 launch fails to reach its ready state."""


def run_trial(density: int, ros_env: dict, trial_num: int, total: int, attempt: int = 1) -> bool:
    world_base = WORLD_NAME or f"warehouse_people{density}"
    log(f"{'='*60}")
    log(f"WORLD {world_base}  ({trial_num}/{total})  [attempt {attempt}/{MAX_ATTEMPTS}]")
    log(f"{'='*60}")

    world_path = str(WS / f"worlds/{world_base}")
    world_sdf  = world_path + ".sdf"
    out_json   = RESULTS_DIR / f"world{density}_metrics.json"
    evo_log    = RESULTS_DIR / f"world{density}_evo_log.json"
    obs_log    = RESULTS_DIR / f"world{density}_observations.jsonl"
    obs_belief = RESULTS_DIR / f"world{density}_belief.json"
    trace_out  = RESULTS_DIR / f"world{density}_trace.jsonl"
    log_dir    = RESULTS_DIR / f"_logs/world{density}/attempt{attempt}"
    log_dir.mkdir(parents=True, exist_ok=True)

    # obs_log is APPENDED and obs_belief ACCUMULATES, so a stale file from a
    # previous attempt would silently merge two trials into one.
    for stale in [out_json, evo_log, obs_log, obs_belief, trace_out]:
        if stale.exists():
            stale.unlink()
            log(f"  Removed stale {stale.name}")

    kill_stale_ros()

    # Named handles to each log file — referenced by wait_for_log calls below
    sim_log  = log_dir / "sim.log"
    loc_log  = log_dir / "localization.log"
    nav2_log = log_dir / "nav2.log"
    yolo_log = log_dir / "yolo.log"

    procs: list = []

    try:
        # ── Step 1: Gazebo ────────────────────────────────────────────────────
        log("[1/8] Launching Gazebo simulation...")
        gz_cmd = (
            f"ros2 launch clearpath_gz simulation.launch.py "
            f"world:={world_path} x:={SPAWN_X} y:={SPAWN_Y} rviz:=false"
        )
        procs.append(launch(gz_cmd, ros_env, sim_log))
        log(f"  PID {procs[-1].pid} → {sim_log.name}")

        ok = wait_for_log(sim_log, PAT_JACKAL_READY, SPAWN_TIMEOUT,
                          "platform_velocity_controller active (jackal ready)")
        if not ok:
            raise SetupFailed("jackal-ready pattern not seen — Gazebo/gz_ros2_control did not finish spawning")
        log("  Sleeping 2s before scan relay...")
        time.sleep(2)

        # ── Step 2: Scan relay ────────────────────────────────────────────────
        log("[2/8] Launching scan relay (lidar3d → lidar2d)...")
        relay_cmd = (
            f"python3 {WS}/scan_relay.py "
            f"{NS}/sensors/lidar3d_0/scan {NS}/sensors/lidar2d_0/scan "
            f"--ros-args -p use_sim_time:=true"
        )
        procs.append(launch(relay_cmd, ros_env, log_dir / "relay.log"))
        log(f"  PID {procs[-1].pid} → relay.log  (fire-and-forget)")

        # ── Step 3: Localization (AMCL + static map, with per-step retries) ──
        # The amcl/map_server change_state response is sometimes lost over DDS
        # (same race as the Nav2 bond flake). Relaunching just localization
        # keeps Gazebo up — far cheaper than failing the whole bringup attempt.
        loc_cmd = (
            f"ros2 launch clearpath_nav2_demos localization.launch.py "
            f"map:={WS}/factory_sim_map.yaml use_sim_time:=true "
            f"setup_path:={_HOME}/clearpath/"
        )
        loc_proc = None
        loc_ok   = False
        for loc_try in range(1, LOC_RETRIES + 1):
            if loc_proc is not None:
                log(f"  [loc attempt {loc_try}/{LOC_RETRIES}] Killing previous localization instance...")
                try:
                    os.killpg(os.getpgid(loc_proc.pid), signal.SIGTERM)
                except (ProcessLookupError, OSError):
                    pass
                for pat in ["localization.launch", "nav2_amcl", "nav2_map_server"]:
                    subprocess.run(["pkill", "-9", "-f", pat],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if loc_proc in procs:
                    procs.remove(loc_proc)
                time.sleep(3)

            loc_log_cur = log_dir / (
                "localization.log" if loc_try == 1 else f"localization_attempt{loc_try}.log")
            log(f"[3/8] Launching localization (AMCL, factory_sim_map.yaml) "
                f"(attempt {loc_try}/{LOC_RETRIES})...")
            loc_proc = launch(loc_cmd, ros_env, loc_log_cur)
            procs.append(loc_proc)
            log(f"  PID {loc_proc.pid} → {loc_log_cur.name}")

            loc_ok = wait_for_log(loc_log_cur, PAT_AMCL_NEEDS_POSE, 60,
                                  "AMCL active and requesting initial pose")
            if loc_ok:
                loc_log = loc_log_cur   # keep loc_log pointing to the live log
                break
            log(f"  Localization attempt {loc_try}/{LOC_RETRIES} timed out.")

        if not loc_ok:
            raise SetupFailed(f"AMCL 'needs initial pose' not seen after {LOC_RETRIES} attempts")

        # ── Step 4: Publish AMCL initial pose ────────────────────────────────
        log(f"[4/8] Publishing AMCL initial pose at ({SPAWN_X}, {SPAWN_Y})...")
        cov = ("0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, "
               "0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685")
        init_pose_cmd = (
            f"ros2 topic pub --once {NS}/initialpose "
            f"geometry_msgs/msg/PoseWithCovarianceStamped "
            f"\"{{header: {{frame_id: 'map'}}, "
            f"pose: {{pose: {{position: {{x: {SPAWN_X}, y: {SPAWN_Y}, z: 0.0}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, "
            f"covariance: [{cov}]}}}}\""
        )
        r = subprocess.run(
            ["bash", "-c",
             f"source /opt/ros/humble/setup.bash 2>/dev/null; {init_pose_cmd}"],
            env=ros_env, capture_output=True, text=True, timeout=15
        )
        if r.returncode == 0:
            log("  Initial pose published OK.")
        else:
            log(f"  WARNING: publish returned {r.returncode}: {r.stderr[:120]}")

        ok = wait_for_log(loc_log, PAT_POSE_RECEIVED, 30,
                          "AMCL accepted initial pose (map→odom TF valid)")
        if not ok:
            raise SetupFailed("AMCL did not confirm initial pose — map→odom TF not established")

        # ── Step 5: Nav2 (with per-step retries) ─────────────────────────────
        nav2_cmd = (
            f"ros2 launch {WS}/nav2_custom.launch.py use_sim_time:=true "
            f"setup_path:={_HOME}/clearpath/"
        )
        nav2_proc = None
        nav2_ok   = False
        for nav2_try in range(1, NAV2_RETRIES + 1):
            if nav2_proc is not None:
                log(f"  [Nav2 attempt {nav2_try}/{NAV2_RETRIES}] Killing previous Nav2 instance...")
                try:
                    os.killpg(os.getpgid(nav2_proc.pid), signal.SIGTERM)
                except (ProcessLookupError, OSError):
                    pass
                for pat in ["nav2_custom.launch", "nav2_bringup", "bt_navigator",
                            "planner_server", "controller_server", "waypoint_follower",
                            "behavior_server", "smoother_server", "velocity_smoother"]:
                    subprocess.run(["pkill", "-9", "-f", pat],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                if nav2_proc in procs:
                    procs.remove(nav2_proc)
                time.sleep(3)

            nav2_log_cur = log_dir / f"nav2_attempt{nav2_try}.log"
            log(f"[5/8] Launching Nav2 (attempt {nav2_try}/{NAV2_RETRIES})...")
            nav2_proc = launch(nav2_cmd, ros_env, nav2_log_cur)
            procs.append(nav2_proc)
            log(f"  PID {nav2_proc.pid} → {nav2_log_cur.name}")

            nav2_ok = wait_for_log(nav2_log_cur, PAT_NAV2_BONDED, NAV2_TIMEOUT,
                                   "Nav2 lifecycle_manager_navigation bond timer (all nodes active)")
            if nav2_ok:
                nav2_log = nav2_log_cur   # keep nav2_log pointing to the live log
                break
            log(f"  Nav2 attempt {nav2_try}/{NAV2_RETRIES} timed out.")

        if not nav2_ok:
            raise SetupFailed(f"Nav2 bond timer not seen after {NAV2_RETRIES} attempts")

        # ── Step 6: YOLO (all 4 cameras) ─────────────────────────────────────
        log("[6/8] Launching YOLO (start_yolo.sh, cameras 0-3)...")
        procs.append(launch(f"NS={NS} {WS}/start_yolo.sh", ros_env, yolo_log))
        log(f"  PID {procs[-1].pid} → {yolo_log.name}")

        ok = wait_for_log_count(yolo_log, PAT_YOLO_CLASSES, YOLO_CAMERAS,
                                YOLO_TIMEOUT,
                                f"set_classes confirmed for all {YOLO_CAMERAS} cameras")
        if not ok:
            raise SetupFailed(f"YOLO set_classes not confirmed for all {YOLO_CAMERAS} cameras")

        # ── Step 7: Trackers cam1-3 (+3 s after YOLO done) ───────────────────
        log("[7/8] Sleeping 3s then launching tracker_with_yolo (cam1, cam2, cam3)...")
        log("      (cam0 tracker is embedded inside evo_plan_run.launch.py)")
        time.sleep(3)
        for cam in [1, 2, 3]:
            tracker_cmd = (
                f"ros2 run evo_skill_ros tracker_with_yolo --ros-args "
                f"-r __node:=tracker_with_yolo_cam{cam} -r __ns:={NS} "
                f"-p namespace:={NS} -p tracking_topic:=/yolo_{cam}/tracking "
                f"-p points_topic:=/sensors/camera_{cam}/points "
                f"-p out_topic:=/tracks -p target_frame:=map -p use_sim_time:=true "
                f"-r /tf:=tf -r /tf_static:=tf_static"
            )
            procs.append(launch(tracker_cmd, ros_env, log_dir / f"tracker_cam{cam}.log"))
            log(f"  cam{cam} tracker PID {procs[-1].pid}")

        # ── Step 8: evo_plan_run (+3 s after trackers) ───────────────────────
        log("[8/8] Sleeping 3s then launching evo_plan_run (PDDL + scand_metrics)...")
        time.sleep(3)
        log(f"  target={TARGET_REGION}  metrics_duration={TRIAL_TIMEOUT}  "
            f"stop_on_success=true")
        evo_cmd = (
            f"ros2 launch evo_skill_ros evo_plan_run.launch.py "
            f"namespace:={NS} robot_name:=jackal_1 "
            f"target_region:={TARGET_REGION} "
            f"graph_file:={EVO_CFG}/graph.json "
            f"domain_file:={EVO_CFG}/factory_sim_domain.pddl "
            f"plan_file:={PLAN_FILE} "
            f"tracks_topic:={NS}/tracks "
            f"tracking_topic:=/yolo_0/tracking "
            f"costmap_edit_max_radius:={COSTMAP_EDIT_RADIUS} "
            f"stl_replan_cooldown_s:={STL_REPLAN_COOLDOWN} "
            f"require_map:=false "
            f"enable_metrics:=true "
            f"metrics_world:=warehouse "
            f"metrics_duration:={TRIAL_TIMEOUT} "
            f"metrics_stop_on_success:=true "
            f"metrics_actors_sdf:={world_sdf} "
            f"metrics_json_out:={out_json} "
            f"metrics_trace_out:={trace_out} "
            f"json_log_file:={evo_log} "
            f"enable_observation_log:={'true' if OBS_LOG == '1' else 'false'} "
            f"obs_log_file:={obs_log} "
            f"obs_belief_file:={obs_belief} "
            f"obs_log_period_s:={OBS_LOG_PERIOD} "
            f"obs_log_cameras:={OBS_LOG_CAMERAS}"
            # ros2 launch rejects a bare 'name:=' as malformed, so an empty
            # allowlist is omitted and the launch default ("" = all) applies.
            + (f" obs_log_classes:={OBS_LOG_CLASSES}" if OBS_LOG_CLASSES else "")
            + (f" obs_exclude_file:={OBS_EXCLUDE_FILE}" if OBS_EXCLUDE_FILE else "")
        )
        procs.append(launch(evo_cmd, ros_env, log_dir / "evo.log"))
        log(f"  evo_plan_run PID {procs[-1].pid} → evo.log")

        # ── Poll for metrics JSON ─────────────────────────────────────────────
        log(f"  TRIAL RUNNING — polling for {out_json.name}...")
        t0 = time.time()
        # metrics_duration is sim time; WALL_FACTOR covers RTF < 1 (capped 0.5)
        deadline = t0 + TRIAL_TIMEOUT * WALL_FACTOR + 30
        pddl_logged = False
        while time.time() < deadline:
            if out_json.exists():
                log(f"  [{time.time()-t0:.0f}s] Metrics JSON written — trial done.")
                break
            if not pddl_logged and check_pddl_success(evo_log):
                pddl_logged = True
                log(f"  [{time.time()-t0:.0f}s] [PDDL] all waypoints completed.")
            elapsed = int(time.time() - t0)
            remaining = int(deadline - time.time())
            log_sub(f"[{elapsed:>4}s] waiting... ({remaining}s left)")
            time.sleep(2)
        else:
            log(f"  TIMEOUT — no metrics JSON after {int(TRIAL_TIMEOUT*WALL_FACTOR)+30}s.")

    finally:
        kill_all(procs, label=f"world{density}")

    # ── Post-trial results ────────────────────────────────────────────────────
    if not out_json.exists():
        log(f"FAIL — {out_json.name} was not written.")
        return False
    try:
        d = json.loads(out_json.read_text())
    except Exception as e:
        log(f"FAIL — could not parse {out_json.name}: {e}")
        return False

    log(
        f"world{density}: "
        f"success={'yes' if d.get('success') else 'no'}  "
        f"dur={_fmt(d.get('duration_s'), 1)}s  "
        f"path={_fmt(d.get('path_length_m'), 1)}m  "
        f"coll={d.get('collisions', '?')}  "
        f"social={_fmt(d.get('social_pct'), 1)}%  "
        f"min_clr={_fmt(d.get('min_human_clearance_m'), 2)}m"
    )
    return True

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
_worlds_done: list = []


def _sigint_handler(sig, frame):
    log("\nCtrl-C — writing partial tables and exiting.")
    if _worlds_done:
        write_density_table(RESULTS_DIR, _worlds_done)
        print_summary_table(RESULTS_DIR, _worlds_done)
    sys.exit(0)


def main():
    global _worlds_done
    signal.signal(signal.SIGINT, _sigint_handler)

    log("density_sweep starting.")
    log(f"  Worlds:        {WORLDS}")
    log(f"  Target region: {TARGET_REGION}")
    log(f"  Plan file:     {PLAN_FILE}")
    log(f"  Results dir:   {RESULTS_DIR}")
    log(f"  Trial timeout: {TRIAL_TIMEOUT}s")
    log(f"  Spawn:         x={SPAWN_X}, y={SPAWN_Y}")

    for p, label in [
        (WS / "factory_sim_map.yaml",               "factory_sim_map.yaml"),
        (Path(PLAN_FILE),                            f"PLAN_FILE ({PLAN_FILE})"),
        (EVO_CFG / "graph.json",                     "graph.json"),
        (EVO_CFG / "factory_sim_domain.pddl",        "factory_sim_domain.pddl"),
        (WS / "scan_relay.py",                       "scan_relay.py"),
        (WS / "start_yolo.sh",                       "start_yolo.sh"),
    ]:
        status = "OK " if p.exists() else "MISSING"
        log(f"  {status}: {label}")
        if not p.exists():
            sys.exit(1)

    for N in WORLDS:
        base = WORLD_NAME or f"warehouse_people{N}"
        sdf = WS / f"worlds/{base}.sdf"
        status = "OK " if sdf.exists() else "MISSING"
        log(f"  {status}: {base}.sdf")
        if not sdf.exists():
            sys.exit(1)

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    ros_env = capture_ros_env()

    n_ok = 0
    for i, density in enumerate(WORLDS, start=1):
        ok = False
        for attempt in range(1, MAX_ATTEMPTS + 1):
            try:
                ok = run_trial(density, ros_env, trial_num=i, total=len(WORLDS), attempt=attempt)
                break  # completed (success or step-8 timeout) — don't retry
            except SetupFailed as exc:
                log(f"  [attempt {attempt}/{MAX_ATTEMPTS}] Setup failed: {exc}")
                kill_stale_ros()
                if attempt < MAX_ATTEMPTS:
                    log(f"  Retrying world{density} in 5s...")
                    time.sleep(5)
                else:
                    log(f"  All {MAX_ATTEMPTS} attempts exhausted for world{density} — skipping world.")
        if ok:
            _worlds_done.append(density)
            n_ok += 1
            write_density_table(RESULTS_DIR, _worlds_done)
            # per-run copy
            single = RESULTS_DIR / f"world{density}_table.tex"
            combined = RESULTS_DIR / "density_table.tex"
            if combined.exists():
                single.write_text(combined.read_text())
                log(f"  Wrote per-run table: {single.name}")
        else:
            log(f"Trial world{density} FAILED — no table row.")
        log("")

    log("=" * 60)
    log(f"DENSITY SWEEP COMPLETE — {n_ok}/{len(WORLDS)} trials successful")
    log("=" * 60)
    print_summary_table(RESULTS_DIR, WORLDS)
    if _worlds_done:
        write_density_table(RESULTS_DIR, _worlds_done)
        log(f"Final table: {RESULTS_DIR}/density_table.tex")


if __name__ == "__main__":
    main()
