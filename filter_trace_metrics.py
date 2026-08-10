#!/usr/bin/env python3
"""Recompute the proxemic metrics from a scand_metrics per-tick trace, excluding
individual humans around "unavoidable" close encounters.

Motivation: gz <actor> people follow fixed scripted trajectories and do not avoid
the robot, so an actor sometimes walks straight into a STOPPED robot. That is the
actor's doing, not a social-navigation failure of the planner, but the live metric
counts it: scand_metrics collapses every human into one scalar min_clr per tick,
so the intruding person inflates intim/pers/social % and P_int (and, below the
collision radius, the collision count) with no way to attribute or remove it.

This script reads the per-tick trace (scand_metrics --trace-out), finds those
encounters, blanks out the offending human for a window either side of each one,
and recomputes the metrics over the REMAINING humans. Everything else -- zone
thresholds, the P_int reference distance, the per-tick accounting, the
pct = 100*count/max(ticks,1) denominator -- is identical to scand_metrics, so the
filtered numbers are directly comparable to the unfiltered ones.

Only the human that triggered the encounter is excluded, and only inside its
window; every other person still counts for those ticks.

Usage:
  # one trial, printing filtered vs original
  python3 filter_trace_metrics.py results/warehouse_ablation/6_2_trace.jsonl

  # whole sweep, writing a JSON summary
  python3 filter_trace_metrics.py results/warehouse_ablation/*_trace.jsonl \
      --json-out results/warehouse_ablation/filtered_metrics.json

Defaults: exclude a human for +/-3 s of SIM time around any approach closer than
0.45 m (the intimate-zone radius -- catches the 0.30-0.45 m band that never
registers as a collision) that happens while the robot is essentially stopped
(< 0.05 m/s). Use --no-require-static to exclude regardless of robot motion.
"""
import argparse
import glob
import json
import math
import os
import re
import shutil
import sys

# --- must mirror scand_metrics.py exactly -------------------------------------
ZONES = (("intim", 0.45), ("pers", 1.2), ("social", 3.6))
PERSONAL = 1.2          # P_int reference distance
# ------------------------------------------------------------------------------


def load_trace(path):
    """Read a trace.jsonl -> list of tick dicts (bad/partial lines skipped)."""
    ticks = []
    with open(path, errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue          # tolerate a truncated final line
            if "humans" in d and "robot" in d:
                ticks.append(d)
    return ticks


def find_exclusions(ticks, trigger_r, require_static, static_speed):
    """-> {human_name: [(t_start, t_end), ...]} merged exclusion windows."""
    events = {}
    for tk in ticks:
        if require_static and (tk["robot"].get("speed") or 0.0) >= static_speed:
            continue
        t = tk["t"]
        for h in tk["humans"]:
            if h["d"] < trigger_r:
                events.setdefault(h["name"], []).append(t)
    return events


def merge_windows(times, window_s):
    """[t1,t2,...] -> merged [(start,end)] intervals of +/-window_s."""
    if not times:
        return []
    iv = sorted((t - window_s, t + window_s) for t in times)
    out = [list(iv[0])]
    for s, e in iv[1:]:
        if s <= out[-1][1]:
            out[-1][1] = max(out[-1][1], e)
        else:
            out.append([s, e])
    return [tuple(x) for x in out]


def in_windows(t, windows):
    return any(s <= t <= e for s, e in windows)


def compute(ticks, excl_windows, coll_r):
    """Recompute the metrics, skipping excluded humans. Mirrors scand_metrics.tick()."""
    zone_counts = {z: 0 for z, _ in ZONES}
    p_int = 0.0
    collisions = 0
    in_collision = False
    min_clr_overall = float("inf")
    n_ticks = len(ticks)
    excluded_samples = 0

    for tk in ticks:
        t = tk["t"]
        min_clr = float("inf")
        for h in tk["humans"]:
            w = excl_windows.get(h["name"])
            if w and in_windows(t, w):
                excluded_samples += 1
                continue                      # this person is blanked out here
            if h["d"] < min_clr:
                min_clr = h["d"]
        if math.isfinite(min_clr):
            min_clr_overall = min(min_clr_overall, min_clr)
            for z, thr in ZONES:
                if min_clr < thr:
                    zone_counts[z] += 1
            p_int += max(0.0, PERSONAL - min_clr)
            if min_clr < coll_r:
                if not in_collision:
                    collisions += 1
                    in_collision = True
            else:
                in_collision = False

    n = max(n_ticks, 1)
    return {
        "ticks": n_ticks,
        "intim_pct": round(100.0 * zone_counts["intim"] / n, 3),
        "pers_pct": round(100.0 * zone_counts["pers"] / n, 3),
        "social_pct": round(100.0 * zone_counts["social"] / n, 3),
        "p_int": round(p_int, 3),
        "min_human_clearance_m": (round(min_clr_overall, 4)
                                  if math.isfinite(min_clr_overall) else None),
        "collisions": collisions,
        "excluded_samples": excluded_samples,
    }


USABLE_AREA_M2 = 1214.0   # matches density_sweep.py / gen_ablation_table.py

TEX_TEMPLATE = """%% Auto-generated by filter_trace_metrics.py (clearance filter applied)
\\begin{table}[t]
\\centering
\\caption{SCAND social-compliance metrics, single trial, with humans removed
$\\pm$%(clearance)g s around each contact closer than %(trigger)g m%(static)s.
Density = actors / 1214\\,m$^2$ navigable area.}
\\label{tab:density_sweep}
\\begin{tabular}{ccccccccccc}
\\toprule
Actors & Density & Succ & Time\\,[s] & Path\\,[m] & Coll & Intim\\,\\%% & Pers\\,\\%% & Social\\,\\%% & $P_{\\mathrm{int}}$ & MinClr\\,[m] \\\\
\\midrule
%(row)s \\\\
\\bottomrule
\\end{tabular}
\\end{table}
"""


def _tex_val(v, prec=1):
    return "--" if v is None else f"{v:.{prec}f}"


def write_trial_tex(out_dir, key, orig, new, args):
    """Emit a filtered <N>_<r>.tex in the layout plot_runs.py / gen_ablation_table.py read."""
    n_actors = int(key.split("_")[0])
    reached = bool(orig.get("success")) or (orig.get("path_progress_pct") or 0) >= 100
    succ = reached and (new.get("collisions") or 0) == 0
    row = " & ".join([
        str(n_actors),
        f"{n_actors / USABLE_AREA_M2:.4f}",
        "\\checkmark" if succ else "\\texttimes",
        _tex_val(orig.get("duration_s"), 0),
        _tex_val(orig.get("path_length_m"), 1),
        _tex_val(new.get("collisions"), 0),
        _tex_val(new.get("intim_pct"), 1),
        _tex_val(new.get("pers_pct"), 1),
        _tex_val(new.get("social_pct"), 1),
        _tex_val(new.get("p_int"), 1),
        _tex_val(new.get("min_human_clearance_m"), 2),
    ])
    body = TEX_TEMPLATE % {
        "clearance": args.clearance,
        "trigger": args.trigger_radius,
        "static": " while the robot was stationary" if args.require_static else "",
        "row": row,
    }
    with open(os.path.join(out_dir, f"{key}.tex"), "w") as fh:
        fh.write(body)


def sibling_metrics(trace_path):
    """<N>_<r>_trace.jsonl -> the original <N>_<r>.json, if present."""
    base = os.path.basename(trace_path)
    if base.endswith("_trace.jsonl"):
        cand = os.path.join(os.path.dirname(trace_path),
                            base[: -len("_trace.jsonl")] + ".json")
        if os.path.exists(cand):
            try:
                return json.load(open(cand, errors="replace"))
            except (json.JSONDecodeError, OSError):
                return None
    return None


def fmt(v, prec=2):
    return "--" if v is None else f"{v:.{prec}f}"


def main():
    ap = argparse.ArgumentParser(
        description="Recompute proxemic metrics from a scand_metrics trace, excluding "
                    "humans around unavoidable close encounters.")
    ap.add_argument("traces", nargs="+", help="trace.jsonl file(s); globs allowed")
    ap.add_argument("--clearance", "--window-s", dest="clearance", type=float, default=3.0,
                    help="remove the offending human from the log for +/- this many "
                         "seconds around each collision (default 3.0). Seconds are "
                         "SIMULATED scenario time by default -- that is what the trace "
                         "records and what governs actor motion; see --clearance-base")
    ap.add_argument("--clearance-base", choices=("sim", "wall"), default="sim",
                    help="interpret --clearance as simulated seconds (default) or "
                         "wall-clock seconds; wall converts as sim = wall * rtf")
    ap.add_argument("--rtf", type=float, default=0.5,
                    help="real-time factor used for --clearance-base wall (default 0.5, "
                         "the value the warehouse world caps physics at)")
    ap.add_argument("--trigger-radius", type=float, default=0.45,
                    help="distance that counts as a collision and starts an exclusion "
                         "window (default 0.45 = the intimate zone, which catches the "
                         "0.30-0.45 m contacts that never register as collisions; use "
                         "0.3 to match the scand_metrics collision radius exactly)")
    ap.add_argument("--collision-radius", type=float, default=0.3,
                    help="distance counted as a collision when recomputing (default 0.3, "
                         "matches scand_metrics)")
    ap.add_argument("--emit-tex", default="",
                    help="write filtered per-trial <N>_<r>.tex into this directory, in the "
                         "layout plot_runs.py and gen_ablation_table.py already read, so "
                         "both regenerate from it unchanged. Trials with no trace (the N=0 "
                         "baseline) are copied through verbatim")
    ap.add_argument("--require-static", dest="require_static", action="store_true",
                    default=False,
                    help="only exclude when the robot was essentially stopped. NOTE: on "
                         "this dataset that matches ZERO events (closest approach while "
                         "stopped is 0.71 m), so it makes the filter a no-op")
    ap.add_argument("--no-require-static", dest="require_static", action="store_false",
                    help="exclude around every close approach, moving or not (default)")
    ap.add_argument("--static-speed", type=float, default=0.05,
                    help="robot speed below which it counts as stopped (m/s, default 0.05)")
    ap.add_argument("--json-out", default="",
                    help="write the per-trial filtered results to this JSON path")
    args = ap.parse_args()

    paths = []
    for p in args.traces:
        paths.extend(sorted(glob.glob(p)) if any(c in p for c in "*?[") else [p])
    paths = [p for p in paths if os.path.exists(p)]
    if not paths:
        print("No trace files found.", file=sys.stderr)
        sys.exit(1)

    # --clearance is in simulated seconds unless the caller asked for wall-clock
    window_sim = args.clearance * (args.rtf if args.clearance_base == "wall" else 1.0)
    mode = ("robot stopped (< %.2f m/s)" % args.static_speed) if args.require_static \
        else "any robot motion"
    base = (f"{args.clearance:g}s wall x rtf {args.rtf:g} = {window_sim:g}s sim"
            if args.clearance_base == "wall" else f"{window_sim:g}s sim")
    print(f"Filter: remove a human +/-{base} around contacts "
          f"< {args.trigger_radius:g} m, {mode}\n")

    if args.emit_tex:
        os.makedirs(args.emit_tex, exist_ok=True)

    hdr = (f"{'trial':>16} {'ticks':>6} {'excl':>5} "
           f"{'intim%':>15} {'pers%':>15} {'social%':>15} {'P_int':>15} {'minClr':>13} {'coll':>7}")
    print(hdr)
    print("-" * len(hdr))

    results = {}
    for path in paths:
        ticks = load_trace(path)
        name = os.path.basename(path).replace("_trace.jsonl", "")
        if not ticks:
            print(f"{name:>16}  -- empty/unreadable trace --")
            continue
        events = find_exclusions(ticks, args.trigger_radius,
                                 args.require_static, args.static_speed)
        windows = {h: merge_windows(ts, window_sim) for h, ts in events.items()}
        new = compute(ticks, windows, args.collision_radius)
        orig = sibling_metrics(path)
        if args.emit_tex and orig is not None:
            write_trial_tex(args.emit_tex, name, orig, new, args)

        def pair(key, prec=2):
            f = new[key]
            o = orig.get(key) if orig else None
            fs = fmt(f, prec)
            return f"{fmt(o, prec)}->{fs}" if o is not None else fs

        print(f"{name:>16} {new['ticks']:>6} {len(windows):>5} "
              f"{pair('intim_pct'):>15} {pair('pers_pct'):>15} {pair('social_pct'):>15} "
              f"{pair('p_int', 1):>15} {pair('min_human_clearance_m'):>13} "
              f"{pair('collisions', 0):>7}")

        results[name] = {
            "trace": path,
            "filtered": new,
            "original": ({k: orig.get(k) for k in
                          ("intim_pct", "pers_pct", "social_pct", "p_int",
                           "min_human_clearance_m", "collisions", "ticks")}
                         if orig else None),
            "excluded_humans": {h: [[round(s, 2), round(e, 2)] for s, e in w]
                                for h, w in windows.items()},
            "params": {
                "clearance": args.clearance,
                "clearance_base": args.clearance_base,
                "window_sim_s": window_sim,
                "trigger_radius": args.trigger_radius,
                "collision_radius": args.collision_radius,
                "require_static": args.require_static,
                "static_speed": args.static_speed,
            },
        }

    # Trials with no trace (the reused N=0 baseline) have nothing to filter, but the
    # level must still appear in the regenerated tables/figures -> copy them through.
    if args.emit_tex:
        src_dir = os.path.dirname(os.path.abspath(paths[0]))
        passed = 0
        for tex in sorted(glob.glob(os.path.join(src_dir, "*.tex"))):
            b = os.path.basename(tex)
            if not re.match(r"^\d+_\d+\.tex$", b):
                continue
            dst = os.path.join(args.emit_tex, b)
            if os.path.exists(dst):
                continue                      # filtered version already written
            shutil.copyfile(tex, dst)
            passed += 1
        print(f"\nWrote filtered .tex -> {args.emit_tex}"
              + (f" ({passed} untraced trial(s) copied through unchanged)" if passed else ""))

    if args.json_out:
        with open(args.json_out, "w") as fh:
            json.dump(results, fh, indent=2)
        print(f"Wrote {args.json_out}")

    print("\n('a->b' = original -> filtered; 'excl' = humans with >=1 exclusion window)")


if __name__ == "__main__":
    main()
