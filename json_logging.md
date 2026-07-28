# JSON observation logging

Runtime logging of **what the robot saw, and where**, for diagnosing
plan/perception desync: the PDDL plan expects object A at R3, the robot drives
there, and perception finds object B instead.

Nothing in the stack recorded this before. `eveo_plan_deploy.tracked_objects`
(`nodes/eveo_plan_deploy.py:541`) is a transient obstacle set that expires after
`tracked_object_timeout_s` = 2 s, and `tracker_with_yolo` publishes only a bare
`vision_msgs/Detection3D` position, discarding the 2D bbox.

---

## 1. What it produces

Two files, written by the new `observation_logger` node.

### `observations.jsonl` — per-tick stream

One JSON object per line, appended and flushed every `log_period_s`
**sim** second. Written even when nothing is detected, so `wc -l` is a health
check and `objects: []` positively asserts "nothing visible".

```json
{"seq":42,"wall_time":"2026-07-28T18:03:11.5Z","ros_time_sec":1873.45,
 "use_sim_time":true,"log_period_s":1.0,"frame":"map",
 "robot":{"x":2.81,"y":-13.04,"z":0.13,"qx":0,"qy":0,"qz":0.383,"qw":0.924,
          "yaw":0.785,"tf_stamp_sec":1873.40},
 "robot_region":"r3","robot_region_dist":0.12,"robot_tf_error":null,
 "n_objects":1,
 "objects":[{"camera":0,"tracking_topic":"/yolo_0/tracking","uid":"0:3",
   "track_id":"3","class_id":1,"class_name":"chair","object_type":"chair",
   "score":0.83,"det_stamp_sec":1873.40,"cloud_stamp_sec":1873.40,
   "cloud_frame":"camera_0_link",
   "bbox_px":{"cx":412.0,"cy":233.5,"w":88.0,"h":210.0,
              "xmin":368.0,"ymin":128.5,"xmax":456.0,"ymax":338.5},
   "position_map":{"x":3.41,"y":-12.88,"z":0.42},
   "position_cam":{"x":0.41,"y":-0.09,"z":2.79},"range_m":2.83,
   "region":"r3","region_dist":0.68}],
 "dropped":{"no_data":0,"desync":0,"no_bbox":0,"bad_depth":1,"no_tf":0,
            "class_filtered":0,"excluded":0,"low_score":0,
            "region_out_of_range":0},
 "stale_camera":[3]}
```

**This is a sampled snapshot, not a complete event log.** Cameras publish at
~15 Hz; at the default 1 s period 14 of 15 frames are never examined. The
belief map is what accumulates across ticks.

Every rejection is counted under a *named* `dropped` reason, so "my object is
missing" is answerable from the file alone:

| reason | meaning |
|---|---|
| `no_data` | camera has not published yet |
| `desync` | \|det stamp − cloud stamp\| > `sync_slop_s` — refuses to project a bbox from one instant into depth from another |
| `no_bbox` / `bad_depth` | bbox unreadable / no valid cloud point at that pixel |
| `no_tf` | camera→`map` transform unavailable |
| `class_filtered` | not in the `log_classes` allowlist |
| `excluded` | in `exclude.json` |
| `low_score` | below `min_score` |
| `region_out_of_range` | farther than `region_snap_max_m` from any region; logged with `region: null` rather than misattributed |

### `belief.json` — cumulative, atomically rewritten

Per region: what `graph.json` **expects**, what was actually **observed**, and
visit/dwell counts.

```json
{"graph_file":"...","exclude_file":"...","excluded_classes":["column","pallet","shelf"],
 "mismatch_ignore_types":["human"],"frame":"map","updated_ros_sec":1889.2,
 "visit_radius_m":2.0,"region_snap_max_m":6.0,
 "regions":{
   "r1":{"coords":[-12.29,-12.29],"expected":{},
         "observed":{"chair":{"object_type":"chair","n":11,"first_ros":25.0,
                     "last_ros":30.0,"centroid":[-10.6,-13.0],"max_score":0.72,
                     "uids":["0:3","1:2"]}},
         "visits":1,"dwell_s":10.0,"last_visit_ros":30.0,
         "expected_not_observed":[],"observed_not_expected":["chair"]}}}
```

Three deliberate behaviours:

- **`expected_not_observed` is populated only where `visits > 0`.** Otherwise
  every unvisited region would fabricate a mismatch. `visits` is edge-triggered
  on entering `visit_radius_m`, so it counts arrivals, not ticks.
- **Excluded classes are removed from `expected` too.** Excluding `shelf` while
  `graph.json` still expects one at R3 would otherwise make every visited region
  report a missing shelf.
- **`mismatch_ignore_types` (default `human`) is excluded from both sides.**
  `graph.json` describes static furniture and contains no people, so every
  pedestrian would be structurally guaranteed to register as unexpected. People
  are still fully logged — only the mismatch computation ignores them.

---

## 2. Operating the YOLO / detection parameters

### The four filtering layers, in order

```
YOLO_CLASSES  ->  exclude.json  ->  log_classes  ->  min_score
(what YOLO        (denylist,        (allowlist,      (confidence
 detects)          logging only)     logging only)     floor)
```

**1. `YOLO_CLASSES` — the primary filter.** yolo-world is open-vocabulary and
detects **nothing** outside the classes handed to `set_classes`. Anything absent
here can never appear anywhere downstream.

```bash
YOLO_CLASSES=person,chair,suitcase,backpack,banana ./run_sim.sh
```

Default in `run_sim.sh` / `start_yolo.sh`: `person,chair,table,shelf,column,box,pallet`.

*These do not become STL obstacles.* `eveo_plan_deploy.py:891` discards every
non-human track before it becomes an `ObstacleConstraint`, and `:1114` inflates
the costmap only for humans. Extra classes cost extra `track_callback` monitor
invocations and `evo.log` noise, not extra replans.

**2. `exclude.json` — denylist (logging only).** Lives at
`src/planning_ros_pkgs/evo_skill_ros/config/exclude.json`, installed via
`setup.py`'s `package_files('config/*')` glob.

```json
{ "classes": ["shelf", "pallet", "column"] }
```

A bare array (`["shelf","pallet"]`) also works. Case-insensitive. **Deny wins
over allow.** Missing file = exclude nothing (INFO); malformed = ERROR, node
keeps running. Point elsewhere with `obs_exclude_file:=/path/to/other.json`.

> Name it `*.json`, not `.exclude` — `glob('config/*')` does not match dotfiles,
> so a dotfile would silently never install and filter nothing.

Current contents hold the **environment constants** (fixed racking). `chair` and
`table` are deliberately *not* excluded — they are used as searchable objects.

**3. `log_classes` — allowlist (logging only).** Empty = log everything, which is
the safe default: a typo'd entry cannot silently empty the file.

```bash
OBS_LOG_CLASSES=chair,suitcase ./run_sim.sh
```

**4. `min_score`** — per-run confidence floor on the log, without touching
`/tracks`. Note `yolo_ros` has a single global `threshold`; there are no
per-class thresholds.

> **Inconsistency worth knowing:** `start_yolo.sh:20` passes
> `threshold:=${YOLO_THRESHOLD}` (0.7), but `run_sim.sh` launches
> `yolo-world.launch.py` *without* a `threshold:=` argument, so it silently uses
> the launch default of **0.5**. Observed chair scores of 0.51–0.72 come from
> that. Not yet reconciled.

### Class-name matching

`utility/factory_graph.object_type_from_name` normalises graph object names and
YOLO class names into one vocabulary, which is the only space where
expected-vs-observed is comparable:

```
short_shelf_2, tall_shelf_3, "shelf"  -> shelf
column_7, "column"                    -> column
chair_1, "chair"                      -> chair
person, human                         -> human
anything else (incl. "box")           -> obstacle
```

`box` currently falls into the generic `obstacle` bucket. If boxes become the
primary searchable class they should get their own rule.

---

## 3. Turning it on

**Off by default everywhere** — each logged camera adds a `PointCloud2`
subscription to a sim already RTF-capped at 0.5.

```bash
# single run (inside the container)
OBS_LOG=true ./run_sim.sh

# density sweep
OBS_LOG=1 python3 density_sweep.py

# batch harnesses (host); both forward OBS_* into the container
OBS_LOG=1    ./run_ablation.sh
OBS_LOG=true ./run_experiments_par.sh
```

### Environment variables

| var | default | notes |
|---|---|---|
| `OBS_LOG` | `false` / `0` | `run_sim.sh` takes `true`; `density_sweep.py` tests for `"1"` |
| `OBS_LOG_FILE` | `$WS/observations.jsonl` | per-run under `$ld_cont` in `run_experiments_par.sh` |
| `OBS_BELIEF_FILE` | `$WS/belief.json` | |
| `OBS_LOG_PERIOD` | `1.0` | **sim** seconds — at RTF 0.5 that is ~2 s wall |
| `OBS_LOG_CAMERAS` | `0,1,2,3` when `MULTICAM=1`, else `0` | each adds a cloud subscription |
| `OBS_LOG_CLASSES` | `""` (all) | omitted entirely when empty — `ros2 launch` rejects a bare `name:=` |
| `OBS_EXCLUDE_FILE` | packaged `config/exclude.json` | |

`density_sweep.py` writes per-run `world<N>_observations.jsonl` /
`world<N>_belief.json` under `RESULTS_DIR` and deletes stale copies first (the
JSONL appends and the belief accumulates, so a leftover would merge two trials).

Outputs at the repo root are gitignored; sweep outputs sit under `results/`,
already ignored.

### Reading it

```bash
cat belief.json                                    # plain JSON
head -1 observations.jsonl | python3 -m json.tool  # one snapshot
wc -l observations.jsonl                           # ≈ sim_seconds / log_period_s
jq -c '[.seq,.robot_region,.n_objects]' observations.jsonl | head
jq -c 'select(.n_objects>0)|.objects[]|[.class_name,.region,.position_map]' observations.jsonl
jq '.regions|to_entries[]|select(.value.visits>0)|
    {r:.key,missing:.value.expected_not_observed,extra:.value.observed_not_expected}' belief.json
```

**Troubleshooting**

| symptom | cause |
|---|---|
| `stale_camera:[0,1,2,3]` on every line | `use_sim_time` false — sim vs wall epochs compared |
| `robot:null`, `dropped.no_tf` = detection count | missing `-r /tf:=tf -r /tf_static:=tf_static` on a standalone `ros2 run` |
| `UNORGANIZED (height<=1)` | wrong `points_topic` — fatal, no positions ever |
| zero lines written | no `/clock` yet; the node runs with `use_sim_time` |

---

## 4. Full node parameter list

| parameter | default |
|---|---|
| `namespace` | `/j100_0000` |
| `cameras` | `0` (comma-separated) |
| `tracking_topic_template` | `/yolo_{cam}/tracking` |
| `points_topic_template` | `/sensors/camera_{cam}/points` |
| `graph_file` | packaged `config/graph.json` |
| `exclude_file` | packaged `config/exclude.json` |
| `log_classes` | `""` = all |
| `min_score` | `0.0` |
| `log_period_s` | `1.0` (sim s) |
| `mismatch_ignore_types` | `human` |
| `region_snap_max_m` | `6.0` |
| `visit_radius_m` | `2.0` |
| `target_frame` / `base_frame` | `map` / `base_link` |
| `detection_timeout_s` | `1.0` |
| `sync_slop_s` | `0.15` |
| `min_range_m` / `max_range_m` | `0.1` / `50.0` |
| `points_qos_depth` | `1` |

All are declared with `dynamic_typing`, so `-p cameras:=0` (which YAML-infers as
INTEGER) does not fail against a string parameter.

---

## 5. Files changed this session

**New**

| path | purpose |
|---|---|
| `evo_skill_ros/nodes/observation_logger.py` | the node |
| `evo_skill_ros/utility/detection_projection.py` | bbox → organized-cloud depth → TF, lifted verbatim from `tracker_with_yolo.py` so both compute identical map positions |
| `evo_skill_ros/utility/factory_graph.py` | `load_graph`, `nearest_region`, `object_type_from_name`, `expected_objects_by_region` |
| `evo_skill_ros/config/exclude.json` | class denylist |
| `worlds/gen_object_test_world.py` | builds a test world with searchable objects along `plan.txt` |

`tracker_with_yolo.py` and `eveo_plan_deploy.py` keep their private copies of
those helpers — they are launched 4× by every harness and feed STL replanning,
so migrating them is a separate, revertable, pure-deletion commit.

**Modified**

- `evo_plan_run.launch.py` — `observation_log_node` gated on
  `enable_observation_log` (default false) + 9 launch args.
- `setup.py` (entry point), `package.xml` (`rcl_interfaces`).
- `run_sim.sh`, `start_yolo.sh`, `density_sweep.py`, `run_ablation.sh`,
  `run_experiments_par.sh` — `OBS_*` plumbing; expanded `YOLO_CLASSES`.
- `.gitignore` — `observations.jsonl`, `belief.json`.

**Incidental `run_sim.sh` fixes** (found while wiring, unrelated to logging):

- **`$PLAN` was ignored.** `plan_file` was hardcoded to `/home/user/plan.txt`,
  which does not exist, so `resolve_path()` silently fell back to the packaged
  `config/plan.txt` — meaning `run_experiments_par.sh:145` passing
  `PLAN=factory_mission_<N>.txt` had *no effect* and every mission ran the same
  plan. Now honoured, with a fail-fast check.
- **`TARGET` defaulted to `R10`** while the packaged plan ends at **R11**. Now
  derived from the plan's last `(move …)`, as `run_ablation.sh:62-64` does.
- **STL knobs** were the untuned launch defaults (1.0 / 2.0); now 0.3 / 5.0,
  matching `density_sweep.py` and `run_experiments_par.sh`.
- **`WS`** was `$HOME/autonomy_stack_ros_humble`, correct only inside the
  container; now self-locating like `run_ablation.sh:37`.
- **Forwarded `DISPLAY`.** Under `ssh -X/-Y`, `docker-compose.yml:21` passes
  `localhost:10.0` into the container, which has no X cookie for it →
  *"X11 connection rejected because of wrong authentication"*. Now rewritten to
  `:1`, with a warning if that socket is missing.
- **Container-only guard** — a clear error instead of cascading
  `ros2: command not found`.

---

## 6. Status and known gaps

**Validated live.** A `VisitorChair` placed at R1 `(-10.69,-12.29)` was detected
as `chair`, localised to `(-10.53,-13.01)` — 0.5–0.8 m from truth, consistent
with the bbox centre projecting to the chair's front surface plus SLAM error.
Seen independently by cam0 and cam1 (agreeing to ~0.2 m), attributed to `r1`,
and reported as `observed_not_expected: ['chair']` while 113 nearby pedestrian
detections correctly produced no mismatch.

**Open items**

1. **`expected` is sourced from the wrong place.** `graph.json` contains *only*
   environment constants (27 objects: columns, shelves, chairs, tables). The
   actual searchable object, `box_2`, has no coordinates there — the plan
   declares its location instead: `(pickup-box jackal_1 box_2 R1)`. Sourcing
   `expected` from the PDDL plan would make the desync signal work as intended,
   and would make it time-varying (a box's expected region changes after a
   `dropoff`), which the current static per-region map cannot represent.
2. **Furniture is not detected in the warehouse world.** A person-only run
   logged 1402/1402 `person` despite all seven classes prompted. Test whether
   shelves appear at *any* confidence; if not, the class wording needs changing
   (`"shelving unit"`, `"pallet rack"`) rather than threshold tuning.
3. **`YOLO_THRESHOLD` is not passed by `run_sim.sh`** (see §2).
4. **Object-test run did not finish.** A pedestrian breached the 1.0 m STL
   clearance, the costmap inflation left R11 unplannable, Nav2 aborted, and
   `waypoint_follower` reported "completed" with `Missed waypoints: [0,0,1,2,3,0]`
   — so `evo_plan_deploy` declared success with the robot parked at
   `(-2.12, 5.76)`. Only the R1 chair was reached. Re-run against
   `worlds/warehouse_people0.sdf` (empty crowd) to isolate object logging from
   social-navigation interference.
5. **Performance unmeasured.** Four extra `PointCloud2` subscriptions on an
   RTF-0.5 sim. Compare wall-clock for one `WORLDS=30` trial at `OBS_LOG=0` vs
   `OBS_LOG=1`; if it exceeds ~10%, drop to `OBS_LOG_CAMERAS=0`.
6. **`run_sim.sh`'s Nav2 check uses the ROS 2 daemon.** A stale daemon makes
   `ros2 lifecycle get` return `Node not found` indefinitely and reports
   "Nav2 did not activate" when Nav2 is healthy. `--no-daemon` would be robust.
