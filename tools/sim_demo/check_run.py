"""check_run.py TAG -- one-line summary of a trial; exit 0 if it succeeded with at least one replan."""
import json, sys, collections, glob
import os
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../results/single_trials/")
tag = sys.argv[1]
try:
    d = json.load(open(glob.glob(R + f"*_{tag}.json")[0]))
    c = collections.Counter(e["event"] for e in json.load(open(glob.glob(R + f"*_{tag}_evo_log.json")[0]))["events"])
except Exception as e:
    print(tag, "no metrics", repr(e)); sys.exit(1)
print(tag, {k: d.get(k) for k in ["success", "path_progress_pct", "duration_s", "symbolic_replans", "shield_vetoes", "collisions"]},
      {k: c.get(k, 0) for k in ["nav2_replan_requested", "nav2_goal_superseded", "nav2_goal_aborted", "nav2_spurious_success", "symbolic_replan_applied"]}, flush=True)
sys.exit(0 if d.get("success") and (c.get("nav2_replan_requested", 0) + (d.get("symbolic_replans") or 0)) >= 1 else 1)
