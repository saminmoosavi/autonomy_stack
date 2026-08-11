# `pipeline/` — online-EvoPlan trials

`pipeline/run_trial.sh` runs **one** online-EvoPlan trial end to end;
`pipeline/run_batch.sh` runs a sweep of them. This file is the setup and
debugging guide for the **host** side. It exists because `run_trial.sh`'s
preflight points here when something host-local is missing:

```
[trial] ERROR no host venv; see pipeline/README or Phase 0 of the plan
```

If you are seeing that (or any other preflight error) on a fresh machine, work
through [Phase 0](#phase-0-host-bootstrap) in order, then run
[the smoke test](#smoke-test-no-sim). The [error index](#error-index) maps every
message the trial can print to its cause.

---

## The host/container split (read this first)

A trial is **two processes on two sides of a Docker boundary**, and almost every
setup failure is a piece landing on the wrong side.

| | runs on | needs |
|---|---|---|
| **replan service** (`pipeline/evoplan_bridge_host/server.py`) | **HOST**, port 8077 | Python 3.11+, `openevolve`, VAL, Fast Downward |
| **sim + ROS stack** (`run_sim.sh`, Gazebo, Nav2, `evo_plan_deploy`) | **CONTAINER** (`ubuntu-22-humble:latest`) | built colcon workspace, X display, GPU |

They talk over `127.0.0.1` only because `docker-compose.yml` sets
`network_mode: host`. The service is deliberately **not** in the Humble image:
it needs a modern Python plus `openevolve` and VAL, none of which belong there.

`run_trial.sh` orchestrates from the host: preflight → start/reuse the replan
service → bring up one detached container → `docker exec ./run_sim.sh` inside it
→ collect metrics.

Two consequences worth internalising before debugging:

- **`docker exec` inherits nothing from your shell.** Every variable the sim
  needs is named explicitly in `run_trial.sh` (see the `-e` block around
  `run_trial.sh:332`, plus the `PASSTHROUGH_ENV` loop at `run_trial.sh:312`). A
  variable you set on the command line that is in neither list is silently
  dropped.
- **The repo is bind-mounted at `/home/user/autonomy_stack_ros_humble`.** Paths
  handed to the container must be *container* paths (`WS_IN_CONTAINER`), not
  `$PWD`.

---

## Phase 0: host bootstrap

Everything here is **machine-local and git-ignored** — that is exactly why a
fresh clone fails. From `.gitignore`: `build/`, `install/`, `.venv-evoplan/`,
`.local/`, `results/`, `pipeline/trial.env`. A clone gives you source only.

All commands run from the repo root (`$REPO` below).

### 0.1 — Submodules

```bash
git submodule update --init --recursive
```

Provides `fast_downward/`, `evolve_stl_pddl/` (the `pddl_evolve` evaluator the
host bridge imports), `src/detection_ros_pkgs/yolo_ros` and
`src/planning_ros_pkgs/SPINE`.

`src/planning_ros_pkgs/evo_skill` is an **SSH-URL** submodule
(`git@github.com:saminmoosavi/evo_skill_ros.git`) and will fail without a key —
it is not needed by the trial pipeline. The package the pipeline actually uses,
`src/planning_ros_pkgs/evo_skill_ros`, is tracked directly in this repo.

### 0.2 — The host venv (`.venv-evoplan/`) ← *the error you hit*

Checked by `run_trial.sh:126`; also required by `replan_service.sh:33`
(`REPLAN_PYTHON` overrides the path if you must put it elsewhere).

**Python 3.11+.** The reference machine uses the system `python3.12`:

```bash
python3 -m venv "$REPO/.venv-evoplan"          # -> .venv-evoplan/bin/python
"$REPO/.venv-evoplan/bin/pip" install -U pip
```

The only real dependency is **openevolve**, installed *editable from a clone*,
which drags in `openai`, `pyyaml`, `numpy`, `flask` and friends:

```bash
git clone https://github.com/codelion/openevolve.git "$HOME/openevolve"
"$REPO/.venv-evoplan/bin/pip" install -e "$HOME/openevolve"
```

**Clone it to `$HOME/openevolve`.** `server.py` defaults
`--openevolve-run` to `$HOME/openevolve/openevolve-run.py`
(`server.py:602`) and `evoplan_runner.py` shells out to that script with
`sys.executable`. Elsewhere is fine only if you pass `--openevolve-run` through
`replan_service.sh start`. Reference commit on the working machine:
`411fb59c886c18704caaffb611e17cf9e7d824d2`.

Nothing else needs a `PYTHONPATH`: the bridge inserts the in-repo paths it needs
(`evolve_stl_pddl/pddl_evolve`, `src/planning_ros_pkgs/evo_skill_ros`,
`src/planning_ros_pkgs/evoplan_bridge`) itself.

Verify:

```bash
.venv-evoplan/bin/python -V                        # 3.11+
.venv-evoplan/bin/python -c "import openevolve, openai; print('ok')"
```

### 0.3 — VAL (`.local/`)

VAL validates every candidate plan, and its "Plan Repair Advice" is the text fed
back to the LLM as an OpenEvolve artifact — without it EvoPlan degrades badly.
`replan_service.sh:39` puts `$REPO/.local/bin` on `PATH`, and `planners.py`
finds it via `shutil.which("validate")` → `$VAL_BIN` →
`/usr/local/bin/validate`.

Build from source and stage it into `.local` (no sudo, machine-local):

```bash
git clone https://github.com/KCL-Planning/VAL.git /tmp/VAL
cmake -S /tmp/VAL -B /tmp/VAL/build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/VAL/build -j"$(nproc)"

mkdir -p "$REPO/.local/bin" "$REPO/.local/lib"
cp /tmp/VAL/build/bin/Validate "$REPO/.local/bin/validate.bin"   # name varies by VAL version
cp /tmp/VAL/build/lib/libVAL.so "$REPO/.local/lib/"
```

Then the wrapper that makes the shared library resolvable — this exact file is
what exists on the working machine:

```bash
cat > "$REPO/.local/bin/validate" <<'EOF'
#!/bin/sh
# VAL plan validator wrapper: locates libVAL.so next to this script's ../lib.
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
LD_LIBRARY_PATH="$here/../lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" exec "$here/validate.bin" "$@"
EOF
chmod +x "$REPO/.local/bin/validate"
```

Verify: `PATH="$REPO/.local/bin:$PATH" validate 2>&1 | head -3` should print
VAL's usage banner, not a loader error. The service reports this as
`"val": true` in `/health`.

If a distro package is easier, any `validate` on `PATH` (or `VAL_BIN=/path/to/validate`)
satisfies the lookup.

### 0.4 — Fast Downward

The submodule ships source only; build it once:

```bash
cd "$REPO/fast_downward" && ./build.py && cd "$REPO"   # -> fast_downward/builds/release/
```

`server.py:600` expects `fast_downward/fast-downward.py`; `/health` reports
`"fd": true`. **`fd` is not optional** — the health check hard-exits the trial
when it is false, because `fd_only` and `fd_first` cannot work and even
`evoplan` seeds from the FD result.

### 0.5 — Docker image + colcon workspace

Build the image (details and the UID/GID rationale are in the top-level
[`README.md`](../README.md#install)):

```bash
docker compose build --build-arg UNAME=user --build-arg UID=$(id -u) --build-arg GID=$(id -g)
```

Then build the workspace **inside** the container — `build/` and `install/` are
git-ignored, so a fresh clone has no install tree:

```bash
docker compose run --rm -T ros_humble bash -lc \
  'source /opt/ros/humble/setup.bash; colcon build --symlink-install'
```

The trial specifically requires `evoplan_bridge` to be importable from the
install tree (`run_trial.sh:292`). If only that package is stale:

```bash
docker compose run --rm -T ros_humble bash -lc \
  'source install/setup.bash; colcon build --packages-select evoplan_bridge evo_skill_ros'
```

### 0.6 — Host directories the compose file bind-mounts

`docker-compose.yml` mounts several `$HOME` paths. Docker **creates a missing
source path as an empty root-owned directory** rather than failing, so absent
ones surface later as confusing permission or "no robot description" errors.
Create them up front:

```bash
mkdir -p ~/bags ~/clearpath ~/clearpath_2 ~/jackal_setup ~/warthog_setup ~/.cache/huggingface
```

`~/clearpath` must hold a **generated** Clearpath robot description, not just a
`robot.yaml` — see the simulation-setup section of the top-level README.
`run_trial.sh:304` stages `$ROBOT_YAML` (default `robot_4cam.yaml`, 4 cameras
≈360° detection) over `~/clearpath/robot.yaml` on every run, so generate once
with that file:

```bash
cp "$REPO/robot_4cam.yaml" ~/clearpath/robot.yaml
# then, inside the container: generate_bash   (alias, see docker/.bashrc)
```

`MULTICAM` must match `ROBOT_YAML` — a 4-cam description with `MULTICAM=0` (or
the reverse) is a silent perception downgrade, not an error.

### 0.7 — An X display

Gazebo and RViz are Qt apps that must open a real X display **from inside the
container**, and camera rendering wants the local GPU.

```bash
sudo bash "$REPO/start_x1.sh"     # headless GPU X server on :1, idempotent
xhost +local:                     # grant container access (undo: xhost -local:)
```

`run_trial.sh` handles two subtleties for you: an ssh-forwarded `DISPLAY`
(anything not starting with `:`, e.g. `localhost:10.0`) is **rewritten to `:1`**
— that is the `[trial] DISPLAY=... is forwarded; using :1` line you saw, and it
is correct behaviour, not a warning about your setup — and `XAUTHORITY` is
blanked into the container, because an ssh/gdm cookie path does not resolve
there and shadows local access.

Override with `SIM_DISPLAY=:0` if you want the real desktop display.

### 0.8 — The LLM key

Only needed when `PLANNER_MODE` can reach EvoPlan (`fd_first`, `evoplan`,
`evoplan_only`); `fd_only` never calls out.

```bash
cp pipeline/trial.env.example pipeline/trial.env
$EDITOR pipeline/trial.env        # OPENAI_API_KEY=sk-...  EVOPLAN_MODEL=gpt-5-mini
```

`pipeline/trial.env` is git-ignored and sourced by `run_trial.sh:27-29`; never
put the key in `run_trial.sh`, which is committed.

The service reads the key **at boot**. `run_trial.sh` compares the running
service's `OPENAI_API_KEY`/`EVOPLAN_MODEL` (via `/proc/<pid>/environ`) against
this run's and restarts it when they differ — so changing the key or model mid-
session is handled, but a service started by hand with a stale key is not.

---

## Smoke test (no sim)

Fastest way to prove Phase 0 without waiting on a 3-minute Gazebo bringup:

```bash
pipeline/replan_service.sh start
curl -s 127.0.0.1:8077/health | python3 -m json.tool
```

Healthy output:

```json
{ "ok": true, "val": true, "fd": true, "mock": false,
  "missions": ["factory_mission_01", ...], "uptime_s": 0.1 }
```

Read it as: `val:false` → 0.3 · `fd:false` → 0.4 (**fatal**, the trial exits
here) · `mock:true` → the service was started with `--mock-plan` and will not
really plan · `missions: []` → 0.1, `evolve_stl_pddl` is empty.

Service management: `pipeline/replan_service.sh start|stop|status|health|log`
(pidfile `/tmp/evoplan_replan.pid`, log `/tmp/evoplan_replan.log`,
`REPLAN_PORT=8077`). It is long-lived on purpose — mission library, region graph
and VAL discovery are paid once at boot — and `KEEP_SERVICE=1` (the default)
leaves it up between trials.

Then the cheapest full trial, no tokens spent:

```bash
MISSION=factory_mission_08 CROWD=0 PLANNER_MODE=fd_only ./pipeline/run_trial.sh
```

---

## Running a trial

```bash
MISSION=factory_tour_03 CROWD=0 PLANNER_MODE=evoplan \
FIND_OBJECT="traffic cone" YOLO_CLASSES="person,traffic cone,chair,suitcase,banana" \
SYMBOLIC_DEADLINE=150 ./pipeline/run_trial.sh
```

Every knob is a top-of-file env override in `run_trial.sh` (lines 17–95); the
commonly used ones:

| var | default | notes |
|---|---|---|
| `MISSION` | `factory_mission_08` | needs a plan in `factory_mission_plans/` **and** a PDDL problem in `evolve_stl_pddl/jackal/in/` or `pipeline/missions/` |
| `CROWD` | `30` | selects `worlds/warehouse_people<CROWD>.sdf`; `WORLD_NAME` overrides outright |
| `PLANNER_MODE` | `fd_first` | `fd_only` \| `fd_first` \| `evoplan` \| `evoplan_only` — see below |
| `FIND_OBJECT` | — | two-phase find-an-object mission; forces `OBS_LOG=1`; **must name a class in `YOLO_CLASSES`** |
| `INSPECT_OBJECTS` | — | open-world find-and-inspect (see below); comma-separated classes; forces `OBS_LOG=1`; mutually exclusive with `FIND_OBJECT` |
| `SYMBOLIC_DEADLINE` | `45.0` | wall seconds per replan |
| `RVIZ` / `SIM_DISPLAY` | `false` / `:1` | |
| `TRIAL_TIMEOUT` | `1800` | hard cap on the whole trial |
| `RESULTS_DIR` / `TAG` | `results/single_trials` / timestamp | |

**`evoplan` still runs Fast Downward.** This surprises people, so: only
`evoplan_only` skips it.

| mode | Fast Downward | EvoPlan | if the LLM returns nothing |
|---|---|---|---|
| `fd_only` | the planner | never called | — |
| `fd_first` | the planner | only if FD's plan fails VAL | FD's plan |
| `evoplan` | **runs first** — solvability guard, fallback plan, and the length reference EvoPlan is scored against | always | falls back to FD's plan |
| `evoplan_only` | **never consulted** | the only planner | nothing; the robot holds and the round is abandoned |

`evoplan`'s FD pass is not a leftover. It supplies `shortest_known_length` (the
length-shaping baseline in `evaluator_sim.py`), a validated plan to fall back
on, and a cheap "is this even solvable" check — handing an impossible problem
to an LLM burns the whole deliberation budget for nothing. `evoplan_only`
exists for the two cases where that check is *wrong*: a find-an-object mission,
whose goal is unsolvable by construction and whose useful answer is the plan's
valid prefix, and any measurement of what EvoPlan achieves unaided.

A third thing is FD-shaped and is not a mode at all: the **initial** plan comes
from a static file in `factory_mission_plans/`, generated offline. No planner
runs in the container at startup, whatever `PLANNER_MODE` says — it only governs
replans.

**Float-typed knobs.** ROS 2 parameters are strictly typed and `evo_plan_deploy`
declares several as DOUBLE. `SYMBOLIC_DEADLINE`, `MISSION_TIMEOUT_S`,
`PLAN_EXHAUSTION_GRACE_S` and `FIND_OBJECT_MIN_SCORE` go through `as_float()`
(`run_trial.sh:46`) for this reason — `150` becomes `150.0`. If you add a
DOUBLE-typed knob, route it through `as_float` too, or the node dies at startup
with `InvalidParameterTypeException` while the bringup still reports
"pipeline is UP" and the robot simply never moves.

**Adding a variable the sim must see.** Add it to `PASSTHROUGH_ENV`
(`run_trial.sh:313`), not as a one-off `-e`. `YOLO_CLASSES` was silently dropped
this way once — the cameras ran the default warehouse vocabulary and the tour
logged nothing.

Outputs land in `$RESULTS_DIR/${MISSION}_p${CROWD}_${PLANNER_MODE}_${TAG}*`:
`.json` metrics, `_evo_log.json` event timeline, `_logs/` (sim/nav2/evo bringup
logs, deliberately host-side so a failed bringup leaves evidence), and for
`FIND_OBJECT` runs `_observations.jsonl` + `_belief.json`.

---

## Mission styles

Three, and the difference between them is *what the problem is allowed to
assume about the objects*.

| style | mission knows | example | goal predicate |
|---|---|---|---|
| delivery / coverage | every object, up front | `factory_mission_08` | `delivered`, `visited` |
| find-an-object | that ONE named object exists; not where | `factory_tour_03` + `FIND_OBJECT` | `reached` |
| **find-and-inspect** | **nothing: not how many, not where** | `factory_survey_01` + `INSPECT_OBJECTS` | `inspected-object` |

```bash
MISSION=factory_survey_01 CROWD=0 PLANNER_MODE=evoplan \
INSPECT_OBJECTS="traffic cone,chair" \
YOLO_CLASSES="person,traffic cone,chair,suitcase,banana" \
SYMBOLIC_DEADLINE=150 ./pipeline/run_trial.sh
```

### How the open-world one runs

1. **Survey.** `pipeline/missions/factory_survey_01.pddl` declares no targets at
   all and asks only for region coverage, so unlike the find-an-object tours it
   is perfectly solvable and its plan is driven whole — no prefix truncation.
2. **Discover.** Every `find_object_poll_s` (2 s) the executor re-clusters the
   observation log into individual objects
   (`observation_memory.cluster_detections`) and mints a stable PDDL name for
   each (`object_registry.ObjectRegistry`): `traffic_cone_1`, `traffic_cone_2`,
   … The name must be stable for the whole run, or the planner cannot tell an
   object already inspected from a new one.
3. **Grow the problem.** Each discovery splices `traffic_cone_2 - target` into
   `(:objects ...)`, `(object-at traffic_cone_2 r3)` into `(:init ...)` and
   `(inspected-object traffic_cone_2)` into the goal. The mission's own
   coverage conjuncts survive, so a replan mid-survey still finishes the sweep.
4. **Inspect.** When the plan runs out, anything uninspected triggers a replan.
   `inspect-object` is the only non-`move` action with an executor: it lowers
   to a Nav2 goal at the object's region, oriented at the object's map
   position, followed by a **5 s stationary dwell** (`INSPECT_DWELL_S`) with a
   turn-to-face correction from the live pose.
5. **Repeat, then stop.** Driving to one object often reveals another, so 4
   repeats until a poll adds nothing and everything known is done. With an
   unknown quantity there is no certificate that all objects were found;
   "nothing new was discovered and everything known has been inspected" is the
   strongest available statement and it is what ends the run.

**Requires `SYMBOLIC_REPLAN=1`** (the default). The objects do not exist as PDDL
symbols until they are discovered, so the inspections can only ever be
*replanned* — with Tier 2 off the run surveys and stops, and says so at startup.

Knobs beyond `INSPECT_OBJECTS`, all optional:

| var | default | notes |
|---|---|---|
| `INSPECT_DWELL_S` | `5.0` | sim seconds held facing each object |
| `INSPECT_STANDOFF_M` | `0.0` | 0 = stand at the region centroid and turn. Centroids are known-drivable; a computed stand-off pose can land inside the obstacle |
| `INSPECT_CLUSTER_RADIUS_M` | `5.0` | detections closer than this are one object. Was 1.5 m, which split one cone into two and sent the robot to inspect empty floor — depth projection scatters detections far wider than sensor noise. At 5 m, **two real objects of one class closer than 5 m are counted as one**; lower it for a world that places them side by side |
| `INSPECT_MAX_OBJECTS` | `12` | cap, so a mis-tuned detector cannot grow the mission without bound |

Progress is in `_evo_log.json` as `inspection_objects_discovered`,
`inspection_round_started`, `inspect_started`, `inspect_completed`,
`inspection_complete`, and live on `<ns>/evoplan/status` under `inspection`
(counts, per-object regions, which are done).

---

## Error index

Preflight, in the order `run_trial.sh` checks:

| message | cause / fix |
|---|---|
| `no world at .../warehouse_peopleN.sdf` | `CROWD` has no world. `ls worlds/`; generate with `worlds/gen_density_worlds.py`, or set `WORLD_NAME`. |
| `no plan at factory_mission_plans/<M>.txt` | bad `MISSION`. |
| `no PDDL problem for <M>` | mission has a plan but no problem; add it to `pipeline/missions/`. |
| `FIND_OBJECT and INSPECT_OBJECTS are different missions` | they are: approach one known object vs inspect every object found. Set one. |
| `no robot config at $REPO/<yaml>` | bad `ROBOT_YAML`. |
| **`no host venv; see pipeline/README`** | **§0.2** — `.venv-evoplan/bin/python` missing or not executable. |
| `docker compose unavailable` | Docker not installed/running, or your user is not in the `docker` group. |
| `could not derive a target region` | the plan file has no `(move ... <region>)`; the target is the last one. |
| `PLANNER_MODE=... needs OPENAI_API_KEY` | §0.8, or use `PLANNER_MODE=fd_only`. |
| `no X server on :1 (missing /tmp/.X11-unix/X1)` | §0.7 — `sudo bash start_x1.sh`. |

Service startup:

| message | cause / fix |
|---|---|
| `no interpreter at .../.venv-evoplan/bin/python` | §0.2 (or set `REPLAN_PYTHON`). |
| `service did not become healthy within 15s` | read `/tmp/evoplan_replan.log`; usually an import error in the venv (§0.2) or port 8077 already taken (`REPLAN_PORT`). |
| `service came up but /health is unreachable` | something else is bound to the port, or a proxy is intercepting `127.0.0.1`. |
| `fast-downward missing; fd_only and fd_first cannot work` | §0.4. |
| `WARNING graph file not found` | `src/planning_ros_pkgs/evo_skill_ros/config/graph.json` missing — regions cannot be checked against the map. |
| `NOTE: service is in MOCK mode` | started with `--mock-plan`; `replan_service.sh stop` and restart without it. |

Container and sim:

| message | cause / fix |
|---|---|
| `ros2 never became usable inside the container` | image built but broken, or `/opt/ros/humble` missing — §0.5. |
| `the container cannot open X display` | `xhost +local:` on the host (§0.7). |
| `evoplan_bridge is not in the workspace install tree` | §0.5 — build it; otherwise `evo_plan_deploy` dies on import while bringup still says "UP". |
| `Conflict. The container name is already in use` | a previous trial's container survived; `docker rm -f evoplan-trial`. `cleanup_container()` waits for the name to free, and INT/TERM traps remove it on Ctrl-C — an interrupt that skips them leaves gazebo/DDS holding the GPU and poisons the next trial. |
| `inspect_object_classes is set but enable_symbolic_replan is false` | the inspections can only be replanned; run with `SYMBOLIC_REPLAN=1`. |
| `[INSPECTION COMPLETE] ... no object ... was ever found` | the survey ran and perception never cleared the evidence thresholds. Check `_observations.jsonl` for the class name (it must match `YOLO_CLASSES` exactly), then `INSPECT_CLUSTER_RADIUS_M` and the min-hits/score floors. |
| `[INSPECT] <name> is not in the object registry` | a replan named an object this executor never discovered (an LLM plan can). It still drives there and dwells, without a facing. |
| `evo_plan_deploy died -- the robot was never commanded` | grep `_logs/evo.log` for `InvalidParameterType` (a DOUBLE knob passed as int — see above) or `ModuleNotFoundError` (§0.5). |
| `TRIAL FAILED: no metrics were produced` | the run never reached the metrics writer. Read the `_logs/` tails the script prints. Note the freshness check: `scand_metrics_out.json` and `evo_plan_deploy_log.json` live at fixed repo paths and are only rewritten on success, so stale files are deliberately **not** copied — a "missing" result is honest, not a bug. |
| `trial hit the ...s cap` (rc 124) | raise `TRIAL_TIMEOUT`, or the robot is stuck — check the event timeline for `nav2_stuck_detected` / `nav2_goal_aborted`. |
| bringup says UP but the robot never moves | almost always one of: dead `evo_plan_deploy` (above), an unforwarded env var (`PASSTHROUGH_ENV`), or a container-vs-host path confusion. |

Gazebo/Nav2/YOLO problems that are not specific to this pipeline are in the
top-level [README's troubleshooting section](../README.md#troubleshooting).

---

## What is committed vs. machine-local

A checklist for "it works there but not here" — none of these travel with a
clone:

- `.venv-evoplan/` — §0.2
- `.local/` (VAL) — §0.3
- `fast_downward/builds/` — §0.4
- `build/`, `install/`, `log/` — §0.5
- `~/clearpath` and the other bind-mount dirs — §0.6
- `pipeline/trial.env` — §0.8
- `results/`, `worlds/_variants/` — outputs
- `$HOME/openevolve` — §0.2
