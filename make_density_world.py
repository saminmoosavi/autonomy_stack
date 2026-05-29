#!/usr/bin/env python3
"""Generate a crowd-density variant of a people world by keeping a random subset
of its walking <actor> blocks. Static <model> people and everything else are
left untouched. The SDF isn't strict XML (gz writes loose markup), so this works
on contiguous text blocks rather than an XML parser.

Usage:
  make_density_world.py --base worlds/warehouse_people.sdf \
      --out worlds/_variants/wh_p01_r1.sdf --seed 101 [--min 4 --max 16]

Picks a walker count N uniformly in [min, max] from the seed (reproducible),
keeps a seeded random subset of that many actors, and prints "WALKERS=N".
"""
import argparse
import os
import random
import re
import sys

AP = argparse.ArgumentParser()
AP.add_argument("--base", required=True)
AP.add_argument("--out", required=True)
AP.add_argument("--seed", type=int, required=True)
AP.add_argument("--min", type=int, default=4)
AP.add_argument("--max", type=int, default=16)
AP.add_argument("--keep", type=int, default=None, help="force exact walker count (overrides random)")
a = AP.parse_args()

lines = open(a.base, errors="replace").read().splitlines(keepends=True)

# find contiguous <actor ...> ... </actor> blocks
blocks = []  # (name, start_idx, end_idx_inclusive)
i = 0
start = None
name = None
for idx, ln in enumerate(lines):
    m = re.match(r"\s*<actor\s+name=[\"']([^\"']+)[\"']", ln)
    if m:
        start = idx
        name = m.group(1)
    if start is not None and re.match(r"\s*</actor>\s*$", ln):
        blocks.append((name, start, idx))
        start = None
        name = None

names = [b[0] for b in blocks]
if not names:
    print("ERROR: no <actor> blocks found", file=sys.stderr)
    sys.exit(1)

rng = random.Random(a.seed)
lo = max(0, a.min)
hi = min(a.max, len(names))
n = a.keep if a.keep is not None else rng.randint(lo, hi)
n = max(0, min(n, len(names)))
keep = set(rng.sample(names, n))

# mark line ranges to drop (removed actors)
drop = [False] * len(lines)
for bname, s, e in blocks:
    if bname not in keep:
        for j in range(s, e + 1):
            drop[j] = True

os.makedirs(os.path.dirname(a.out), exist_ok=True)
with open(a.out, "w") as f:
    for j, ln in enumerate(lines):
        if not drop[j]:
            f.write(ln)

print(f"WALKERS={n}")
