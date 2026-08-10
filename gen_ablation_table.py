#!/usr/bin/env python3
"""Aggregate the crowd-density ablation results into one table.

Reads results/warehouse_ablation/<N>_<r>.json (scand_metrics JSON summaries, one
per trial r at crowd size N, written by run_ablation.sh) and emits ONE row per
density level N — Succ. = successes/trials, remaining columns mean$\\pm$std over
the trials at that N. Density = actors / 1214 m^2 navigable area (matches
density_sweep.py). Skips <N>_<r>_evo_log.json.

Usage:
  python3 gen_ablation_table.py [results_dir]
      -> <results_dir>/ablation_table.tex          (unfiltered; the live metrics)

  python3 gen_ablation_table.py [results_dir] --filtered <filtered_metrics.json>
      -> <results_dir>/ablation_table_filtered.tex

With --filtered, the proxemic columns (intim/pers/social/P_int/min-clearance/
collisions) are replaced per trial by the recomputed values from
filter_trace_metrics.py, which excludes a scripted actor that walked into a
stopped robot. Trials missing from that file keep their unfiltered values.
Success is re-derived as "reached the goal AND no collisions remain", so a trial
whose only collision was such an intrusion flips to a success.
"""
import argparse
import glob
import json
import os
import re
from collections import defaultdict
from statistics import mean, pstdev

USABLE_AREA_M2 = 1214.0   # warehouse navigable floor area (matches density_sweep.py)

# <N>_<r>.json  (N = crowd size, r = trial); exclude the *_evo_log.json companions
TRIAL_RE = re.compile(r"^(\d+)_(\d+)\.json$")


def parse_metrics_json(path):
    try:
        d = json.loads(open(path, errors="replace").read())
    except (json.JSONDecodeError, OSError):
        return None
    if d.get("node") != "scand_metrics":
        return None
    return {
        "succ":        bool(d.get("success")),
        "progress":    d.get("path_progress_pct"),
        "duration":    d.get("duration_s"),
        "path_length": d.get("path_length_m"),
        "coll":        d.get("collisions"),
        "min_clr":     d.get("min_human_clearance_m"),
        "intim_pct":   d.get("intim_pct"),
        "pers_pct":    d.get("pers_pct"),
        "social_pct":  d.get("social_pct"),
        "p_int":       d.get("p_int"),
    }


def ms(vals, prec=1):
    """mean$\\pm$std over the non-None values, as 'm$\\pm$s' (or '--')."""
    xs = [v for v in vals if v is not None]
    if not xs:
        return "--"
    m = mean(xs)
    s = pstdev(xs) if len(xs) > 1 else 0.0
    return f"{m:.{prec}f}$\\pm${s:.{prec}f}"


def ms_plain(vals, prec=1):
    xs = [v for v in vals if v is not None]
    if not xs:
        return "--"
    m = mean(xs)
    s = pstdev(xs) if len(xs) > 1 else 0.0
    return f"{m:.{prec}f}±{s:.{prec}f}"


TEX_TRIAL_RE = re.compile(r"^(\d+)_(\d+)\.tex$")


def _tex_num(tok):
    tok = tok.strip()
    return None if tok in ("--", "") else float(tok)


def parse_trial_tex(path):
    """Recover one trial's metrics from the per-trial density_table .tex row.

    Fallback for trials whose <N>_<r>.json was lost: run_ablation.sh copies a
    single-row table per trial, and that row carries every headline metric.
    Columns: Actors & Density & Succ & Time & Path & Coll & Intim & Pers &
    Social & P_int & MinClr.  path_progress_pct is not in the row, but the
    harness only ever logs 100%-route trials, so it is taken as 100.
    """
    try:
        txt = open(path, errors="replace").read()
    except OSError:
        return None
    for line in txt.splitlines():
        line = line.strip()
        if not line.endswith("\\\\") or "&" not in line or line.startswith("Actors"):
            continue
        cols = [c.strip() for c in line[:-2].split("&")]
        if len(cols) < 11:
            continue
        try:
            return {
                "succ":        "checkmark" in cols[2],
                "progress":    100.0,
                "duration":    _tex_num(cols[3]),
                "path_length": _tex_num(cols[4]),
                "coll":        _tex_num(cols[5]),
                "intim_pct":   _tex_num(cols[6]),
                "pers_pct":    _tex_num(cols[7]),
                "social_pct":  _tex_num(cols[8]),
                "p_int":       _tex_num(cols[9]),
                "min_clr":     _tex_num(cols[10]),
            }
        except ValueError:
            return None
    return None


def apply_filtered(d, filt):
    """Overlay recomputed proxemics from filter_trace_metrics.py onto one trial."""
    for k in ("intim_pct", "pers_pct", "social_pct", "p_int",
              "min_human_clearance_m", "collisions"):
        if filt.get(k) is not None:
            d[{"min_human_clearance_m": "min_clr"}.get(k, k)] = filt[k]
    # every logged trial reached the goal (the harness only logs pct==100), so a
    # trial is a success once the surviving collisions are zero
    reached = d["succ"] or (d.get("progress") or 0) >= 100
    d["succ"] = bool(reached and (filt.get("collisions") or 0) == 0)
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results_dir", nargs="?", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "results", "warehouse_ablation"))
    ap.add_argument("--filtered", default="",
                    help="filtered_metrics.json from filter_trace_metrics.py; overlays the "
                         "recomputed proxemic columns and writes ablation_table_filtered.tex")
    args = ap.parse_args()

    global RESULTS_DIR
    RESULTS_DIR = args.results_dir

    filt_map = {}
    if args.filtered:
        try:
            raw = json.load(open(args.filtered, errors="replace"))
        except (json.JSONDecodeError, OSError) as exc:
            raise SystemExit(f"Could not read --filtered {args.filtered}: {exc}")
        for key, rec in raw.items():
            if isinstance(rec, dict) and isinstance(rec.get("filtered"), dict):
                filt_map[key] = rec["filtered"]

    runs = defaultdict(list)   # N -> [metrics dict, ...]
    n_overlaid = 0
    for f in sorted(glob.glob(os.path.join(RESULTS_DIR, "*.json"))):
        m = TRIAL_RE.match(os.path.basename(f))
        if not m:
            continue
        d = parse_metrics_json(f)
        if d is None:
            continue
        key = f"{m.group(1)}_{m.group(2)}"
        if key in filt_map:
            d = apply_filtered(d, filt_map[key])
            n_overlaid += 1
        runs[int(m.group(1))].append(d)

    # Fallback: recover trials whose <N>_<r>.json is missing from the per-trial
    # .tex row that run_ablation.sh copied alongside it.
    n_from_tex = 0
    for f in sorted(glob.glob(os.path.join(RESULTS_DIR, "*.tex"))):
        m = TEX_TRIAL_RE.match(os.path.basename(f))
        if not m:
            continue
        if os.path.exists(os.path.join(RESULTS_DIR, f"{m.group(1)}_{m.group(2)}.json")):
            continue                      # json present; it wins
        d = parse_trial_tex(f)
        if d is None:
            continue
        key = f"{m.group(1)}_{m.group(2)}"
        if key in filt_map:
            d = apply_filtered(d, filt_map[key])
            n_overlaid += 1
        runs[int(m.group(1))].append(d)
        n_from_tex += 1
    if n_from_tex:
        print(f"NOTE: recovered {n_from_tex} trial(s) from per-trial .tex rows "
              f"(their .json summaries are missing).")

    if not runs:
        raise SystemExit(f"No parseable <N>_<r>.json or <N>_<r>.tex trials in {RESULTS_DIR}")

    if args.filtered:
        missing = sum(len(v) for v in runs.values()) - n_overlaid
        print(f"Filtered overlay: {n_overlaid} trial(s) from {args.filtered}"
              + (f"; {missing} trial(s) had no trace and keep unfiltered values" if missing else ""))

    Ns = sorted(runs)

    # ---- LaTeX table ----
    lines = [
        "% Auto-generated by gen_ablation_table.py",
        "% Crowd-density ablation (warehouse, MULTICAM 360 detection); mean$\\pm$std over trials.",
        "\\begin{table*}[t]",
        "\\centering",
        "\\caption{Social-navigation metrics vs.\\ crowd density (warehouse, "
        "MULTICAM 360$^\\circ$ detection). Density = actors\\,/\\,1214\\,m$^2$ navigable "
        "area; Succ.\\ = successes/trials; other columns mean$\\pm$std over trials."
        + (" Proxemic columns exclude scripted actors that walked into a stationary "
           "robot (see filter\\_trace\\_metrics)." if args.filtered else "") + "}",
        "\\label{tab:density_ablation" + ("_filtered" if args.filtered else "") + "}",
        "\\begin{tabular}{cccccccccc}",
        "\\toprule",
        "Actors & Density & Succ. & Time\\,[s] & Path\\,[m] & Coll & MinClr\\,[m] & "
        "Intim\\,\\% & Pers\\,\\% & Social\\,\\% \\\\",
        "\\midrule",
    ]
    for N in Ns:
        rs = runs[N]
        n = len(rs)
        nsucc = sum(1 for r in rs if r["succ"])
        row = " & ".join([
            str(N),
            f"{N / USABLE_AREA_M2:.4f}",
            f"{nsucc}/{n}",
            ms([r["duration"] for r in rs], 0),
            ms([r["path_length"] for r in rs], 1),
            ms([r["coll"] for r in rs], 1),
            ms([r["min_clr"] for r in rs], 2),
            ms([r["intim_pct"] for r in rs], 1),
            ms([r["pers_pct"] for r in rs], 1),
            ms([r["social_pct"] for r in rs], 1),
        ])
        lines.append(row + " \\\\")
    lines += ["\\bottomrule", "\\end{tabular}", "\\end{table*}", ""]

    out = os.path.join(RESULTS_DIR,
                       "ablation_table_filtered.tex" if args.filtered else "ablation_table.tex")
    open(out, "w").write("\n".join(lines))

    # ---- plaintext preview ----
    total = sum(len(v) for v in runs.values())
    print(f"Parsed {total} trials across {len(Ns)} density levels "
          f"(N={Ns}) in {RESULTS_DIR}\n")
    hdr = (f"{'N':>3} {'dens':>7} {'succ':>6} {'time[s]':>12} {'path[m]':>12} "
           f"{'coll':>9} {'minClr':>11} {'social%':>11}")
    print(hdr)
    print("-" * len(hdr))
    for N in Ns:
        rs = runs[N]
        n = len(rs)
        nsucc = sum(1 for r in rs if r["succ"])
        print(f"{N:>3} {N/USABLE_AREA_M2:>7.4f} {f'{nsucc}/{n}':>6} "
              f"{ms_plain([r['duration'] for r in rs],0):>12} "
              f"{ms_plain([r['path_length'] for r in rs],1):>12} "
              f"{ms_plain([r['coll'] for r in rs],1):>9} "
              f"{ms_plain([r['min_clr'] for r in rs],2):>11} "
              f"{ms_plain([r['social_pct'] for r in rs],1):>11}")
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
