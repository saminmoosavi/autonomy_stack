# Jackal end-to-end planning pipeline

Two independent codebases meet here:

| | what it is | who owns it |
|---|---|---|
| `src/planning_ros_pkgs/evo_skill/` (`evo_skill_ros`) | ROS 2 package that **deploys** a PDDL plan to Nav2 | Samin Moosavi — separate git repo |
| `evolve-stl-pddl/` | EvoPlan: OpenEvolve + LLM + VAL/FD that **generates** the plan | this user's repo (CoRL 2026 paper code) |

The pipeline is: **EvoPlan writes a plan text file → `evo_plan_deploy` reads it via `plan_file:=` and drives Nav2 waypoints.** That file is the entire interface. No ROS code change is needed to swap in a freshly generated plan.

## Hard constraint

**Do not modify anything in the ROS stack** — `src/`, `Dockerfile`, `docker-compose.yml`, `docker/`, or the container image. The user has stated this repeatedly. All work goes in `evolve-stl-pddl/`.

Note `src/planning_ros_pkgs/evo_skill/` has pre-existing uncommitted edits to `config/evoskill_plan_lab.txt`, `config/graph_lab.json`, and `launch/evo_plan_run.launch.py`. They are the user's, from before any Claude session. Don't "clean them up".

## Container toolchain (already built into the image — verified 2026-07-30)

The `ubuntu-22-humble:latest` image already carries everything EvoPlan needs; there is nothing left to install.

- Fast Downward at `/opt/fast-downward`, `FD_PATH=/opt/fast-downward/fast-downward.py`
- VAL `validate` on `PATH` (`/usr/local/bin/validate`)
- `openevolve==0.3.2` (pip), with a shim at `~/openevolve/openevolve-run.py` so path-based invocations work
- `OPENAI_API_KEY` reaches the container via `docker-compose.yml` ← `autonomy_stack_ros_humble/.env`
- The repo is bind-mounted at `/home/user/autonomy_stack_ros_humble/evolve-stl-pddl`

## Generating a plan

`evolve-stl-pddl/jackal/gen_plan.sh` is the entry point. Safe to run while the Jackal stack is up — it only talks to the OpenAI API and local FD/VAL, never to ROS.

```bash
docker exec -it autonomy-humble-container \
  ~/autonomy_stack_ros_humble/evolve-stl-pddl/jackal/gen_plan.sh lab
```

Presets: `lab` (lens lab patrol) and `factory` (factory delivery); or pass `--domain`/`--problem`. Env overrides: `ITERS` (default 15), `MODEL` (default `gpt-4.1-mini`), `CONFIG`.

It runs `pddl_evolve/run_evoplan.py`, strips the LLM's stray line-continuation `\` from `best_plan.txt`, re-validates the cleaned file with VAL, writes `jackal/out/<preset>/plan.txt`, and prints the matching `ros2 launch` command.

### Model choice — do not use gpt-5

Measured on the lab task (~6.9k input + ~1.7k output tokens per iteration):

| model | $/iteration | result |
|---|---|---|
| `gpt-4o-mini` | ~$0.0021 | **plateaus at score 0.625**, too weak for multi-step precondition chains |
| `gpt-4.1-mini` | ~$0.0055 | **reached `valid=1.0` at iteration 9** — the default |
| `gpt-5` | ~$0.041+ | reasoning tokens bill at output rate; 20–50× cost, no benefit. User explicitly rejected it. |

`jackal/config_jackal_openai.yaml` still names `gpt-5` — that's the paper's cloud config. `gen_plan.sh` sed-overrides `primary_model`, so don't edit it. OpenEvolve 0.3.2 treats `gpt-5*`/`o*` as reasoning models (drops `temperature`, sends `max_completion_tokens`); `gpt-4*-mini` takes the normal path.

## Timing — measured, lens-lab patrol task (2026-07-30)

`gen_plan.sh` prints a `=== timing ===` block and writes `<out>/timings.txt` on every run:
per-iteration avg/min/max, a per-iteration breakdown, the iteration that first reached
`valid=1.0`, and total wall clock. Raw OpenEvolve output is kept at `<out>/run.log`.

Every run below: same task (`lens_lab_patrol_01` on `lens_lab_domain`), `ITERS=15`, one run each.
"Actions" = length of the final VAL-accepted plan. All valid plans satisfy the same 6 goals;
plan length varies with route choice, correctness does not.

| # | model / setup | s/iter | iters ok | bad diffs | first valid | **time to valid** | total wall | actions | valid |
|---|---|---|---|---|---|---|---|---|---|
| 1 | cloud `gpt-4o-mini`, `DIFF=false` | 18.2 | 15/15 | 0 | never | — | 274 s | 16 | **NO** (0.625) |
| 2 | cloud `gpt-4.1-mini`, `DIFF=false` | 44.2 | 15/15 | 0 | iter 5 | 215.6 s | 663 s | 12 | yes |
| 3 | cloud `gpt-4.1-mini`, `DIFF=true` | **3.4** | 15/15 | 0 | iter 13 | 43.9 s | **51 s** | 13 | yes |
| 4 | local `gpt-oss-20b`, `DIFF=true` | 41.3 | 8/15 | 7 | iter 1 | 25.9 s | 736 s | 14 | yes |
| 5 | local `gpt-oss-20b`, `DIFF=true REASONING=low` | 15.6 | 10/15 | 5 | iter 2 | **23.6 s** | 213 s | 15 | yes |
| 6 | local `Qwen3.6-35b-A3B`, `DIFF=true`, no-think | — | **0/15** | **15** | never | — | 478 s | 0 | **NO** |

Per-iteration detail:
- run 4: `#1=25.8 #2=41.5 #3=41.5 #4=15.5 #7=52.2 #8=55.5 #10=39.2 #13=59.0`
- run 5: `#1=11.8 #2=11.7 #3=11.7 #4=14.8 #6=12.6 #7=39.6 #9=18.6 #10=11.1 #12=12.3 #15=11.7`

Gaps in the iteration numbers are iterations lost to malformed diffs.

**Run 6 failed completely — `Qwen3.6-35b-A3B` could not produce a single parseable
SEARCH/REPLACE block in 15 attempts**, burning 478 s for nothing. Raw generation was by far the
fastest of any model tried (26 tokens / 0.95 s on a probe, vs gpt-oss's 99 tokens / ~4 s), so
this is purely a diff-format-compliance failure, not a speed or capability one. Before reusing
this model, either run it with `DIFF=false` (slower per iteration but no diff parsing) or
harden the diff instructions in the config's `system_message`. Do not assume a faster model is
a better one here.

**Best cloud setup: run 3.** **Best local setup: run 5.** Local is ~4.6x slower per iteration
than cloud but free and offline.

Reading these correctly matters:

- **`DIFF=true` is the one big, reliable lever — 13× on per-iteration cost (44.2 s → 3.4 s).**
  With `diff_based_evolution: false` OpenEvolve uses its full-rewrite template, so the model
  retypes the frozen `DOMAIN_PDDL` + `PROBLEM_PDDL` every iteration: measured at **96% of its
  output** (~3000 tokens emitted to change a ~112-token plan). Always run `DIFF=true`.
- **s/iteration is the stable, controllable number. "First valid at iteration N" is noisy** —
  it came out at 1, 5, and 13 across these three runs. Don't read much into a single run's
  time-to-valid, and don't conclude the local model "beats" the cloud one from the 25.9 s above.
- **`REASONING=low` is the second big lever, and it is local-only.** `gpt-oss-20b` is a
  *reasoning* model whose hidden chain-of-thought dominates its output, and `DIFF=true` cannot
  shrink that because it only trims the visible answer. On an isolated probe (same prompt, same
  quality answer) `reasoning_effort` high→low measured **1039 → 99 completion tokens (10.5×)**;
  across a full run it gave **41.3 s → 15.6 s per iteration and 736 s → 214 s total**, still
  reaching a VAL-valid plan. Do NOT set it for cloud `gpt-4.1-mini` — OpenAI rejects the
  parameter on non-reasoning models. Local is still ~4.6× slower per iteration than cloud diff
  (15.6 s vs 3.4 s); to close that further try `Qwen3.6-35b-A3B` (MoE, ~3B active params →
  small-model generation speed).
- **Diff-format compliance is the deciding factor for local models, not speed.** Bad-diff rates:
  cloud 0/15, `gpt-oss-20b` 7/15 (5/15 at `REASONING=low`), `Qwen3.6-35b-A3B` **15/15 — total
  failure**. OpenEvolve logs `No valid diffs found in response` and skips the iteration, so each
  one is pure wasted wall clock. This dominates any tokens/sec advantage: the fastest-generating
  model tried was also the only one that produced nothing at all.

## Per-model launch requirements (local vLLM)

Each model gates its hidden reasoning differently, and getting this wrong is not just slow —
with reasoning on and no reasoning parser configured, the chain-of-thought lands in `content`
and corrupts every diff OpenEvolve tries to parse.

- **`gpt-oss-20b`** — gated by the `reasoning_effort` *request* parameter, so `gen_plan.sh`'s
  `REASONING=low` works. Measured 1039 → 99 completion tokens (10.5x).
  ```bash
  vllm serve openai/gpt-oss-20b --served-model-name local-pddl --port 8000 --max-model-len 16384
  ```
- **`Qwen3.6-35b-A3B`** — gated by a *chat-template kwarg*, which must be set at launch;
  `REASONING` has no effect on it. Thinking on vs off measured 873 tokens / 43 s vs 26 tokens /
  0.95 s (45x). Serve with:
  ```bash
  vllm serve Qwen/Qwen3.6-35b-A3B --served-model-name local-pddl --port 8000 \
    --max-model-len 16384 --gpu-memory-utilization 0.85 \
    --default-chat-template-kwargs '{"enable_thinking": false}'
  ```

Both are already in `~/.cache/huggingface` (473 GB of models cached; nothing to download).
Serve on the **host**, not in the ROS container — the container is CUDA 11.7 and cannot build
Blackwell kernels, but `network_mode: host` means it reaches `localhost:8000` with no config.
Install: `conda create -n vllm python=3.12 && pip install vllm` (native aarch64 wheel;
torch 2.11+cu130; GB10 is `sm_121` and the `sm_120` cubins are binary-compatible).
- **Nothing early-stops on `valid=1.0`** — a run always burns all `ITERS` iterations even after
  it has a valid plan. Time-to-valid is therefore much less than total wall clock, and adding an
  early stop is the easiest remaining win for both cloud and local.

For live mid-mission replanning, none of these numbers are in the right range: Fast Downward
solves this same problem in **0.003 s**, i.e. ~10^4-10^5 times faster than any LLM path here.
Use EvoPlan offline for the initial plan; use FD in the loop. See the next section.

## Deploying the plan

```bash
EVO_CFG="$(ros2 pkg prefix evo_skill_ros)/share/evo_skill_ros/config"
ros2 launch evo_skill_ros evo_plan_run.launch.py \
  namespace:=/j100_0612 robot_name:=jackal_1 target_region:=fire \
  graph_file:=$EVO_CFG/graph_lab.json domain_file:=$EVO_CFG/lens_lab_domain.pddl \
  plan_file:=/home/user/autonomy_stack_ros_humble/evolve-stl-pddl/jackal/out/lab/plan.txt \
  costmap_edit_max_radius:=1.0 require_map:=true \
  tracking_topic:=/yolo/tracking points_topic:=/sensors/camera_0/points \
  odom_topic:=/platform/odom/filtered tracker_out_topic:=/tracks
```

What `evo_plan_deploy` does with the file:
- Parses any `(action arg ...)` line; non-paren lines are ignored.
- **Does not symbolically validate** — an invalid plan is dispatched silently. VAL checking is EvoPlan's job.
- `move*` actions become Nav2 waypoints. Non-move actions (inspect/check/service/recharge) are **stationary dwells**, not no-ops: the plan is split into legs, and each non-move action adds `inspect_dwell_s` (default 3.0 s, per-leg cap `inspect_dwell_max_s` = 15 s) of holding position at the region it occurs in. Three inspects at `lambdas` = one 9 s hold. This is the observation window perception needs.
- A new plan can be loaded **without restarting the node**: publish the path on `/evo_plan_deploy/load_plan` (`std_msgs/String`, empty string re-reads the current file). It cancels the active goal, clears STL costmap edits, re-aligns the first move to the robot's live region, resets the STL replan budget, and rolls back to the previous plan if the new file fails to parse.
- **Waiting for a replan:** publish `true` on `/evo_plan_deploy/hold` (`std_msgs/Bool`) and the robot parks at the end of the current dwell instead of advancing, which covers EvoPlan's ~10-50 s. The hold is released automatically by the next plan load, or by `hold_timeout_s` (default 120 s) so a failed replan can't strand the robot. Do **not** try to cover replan latency with a long `inspect_dwell_s` — it is global (every inspection gets it) and is read once at startup, so `ros2 param set` has no effect.
- Every `move` **target** must be a region in `graph_lab.json` (`top, bottom, entrance, lambdas, fire`, plus stray `R10–R14`). Unknown targets are warned and skipped.
- `start` is not a graph region. It's a placeholder: the first move's *source* is rewritten to the robot's live nearest region, or the move is dropped if the robot is already at the target.

## Domain mismatch (intentional, don't "fix")

`jackal/lab/lens_lab_domain.pddl` (EvoPlan) is **STRIPS**; the ROS copy at `evo_skill/config/lens_lab_domain.pddl` is **durative/temporal with fluents**. Plan generation uses the STRIPS one — VAL can't validate the temporal version against a sequential plan. Grounded action *names and arities* match, which is all the deploy node needs.

## Known issues

- **The shipped `evo_skill/config/evoskill_plan_lab.txt` is symbolically invalid.** It moves `start→bottom→top` then does `(check-lab-object jackal_1 whiteboard bottom)` while the robot is at `top`; VAL fails at time 3, and `checked-zone top` is never satisfied. It still "runs" because the deploy node skips validation. Replacing it with EvoPlan output is the point of this pipeline.
- **`jackal/*.job` and `blocksworld/*.job` cannot run here.** They hardcode `/scratch/user/bhavya_sai_tamu.edu`, TAMU HPC conda envs, and a local vLLM Qwen3-32B server. The container path is `run_evoplan.py` / `gen_plan.sh` with the cloud API.
- `run_evoplan.py`'s `extract_plan()` regexes the raw file, so `PLAN = """\` leaves a literal `\` as the first line of `best_plan.txt`. Harmless (both VAL and the ROS parser skip it) but `gen_plan.sh` strips it into `plan.txt`.
- `pddl_evolve/cost_report.py`'s `PRICING` table has no `gpt-4.1-mini` row; add `{"in": 0.40, "out": 1.60}` before using it on these runs.

## Verified working (2026-07-30)

Ran `gen_plan.sh lab` with `gpt-4.1-mini`, 15 iterations: `valid=1.0`, 12 actions, VAL `Plan valid`, all move targets present in `graph_lab.json`. Output at `evolve-stl-pddl/jackal/out/lab/plan.txt`. FD independently solves the same task in ~0.003s (14 steps) as a sanity baseline.

**Not yet done:** the generated plan has never been executed on the robot or in sim — the Jackal stack was not running during setup. The remaining validation is a real `evo_plan_run.launch.py` trial. The `factory` preset was also never run end-to-end.
