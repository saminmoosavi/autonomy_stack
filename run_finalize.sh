#!/usr/bin/env bash
# Block until the parallel batch finishes, run a sequential cleanup pass for any
# runs that failed under contention, then build + print the LaTeX results table.
set -uo pipefail
cd /planning/autonomy_stack
OUT=results/factory_missions
REPS=${REPS:-3}
TOTAL=$((5 * REPS))

count_missing() {
  local n=0 p r sm
  for p in 01 02 03 04 05; do for r in $(seq 1 "$REPS"); do
    sm=$OUT/plan${p}_rep${r}_metrics.json
    { [ -s "$sm" ] && python3 -c "import json,sys; json.load(open('$sm'))" 2>/dev/null; } || n=$((n+1))
  done; done
  echo "$n"
}

NW=${NW:-3}
echo "=== waiting for parallel batch ($NW workers) to finish ==="
while :; do
  d=0
  for k in $(seq 1 "$NW"); do grep -q "\] done\." /tmp/exp_par_w$k.log 2>/dev/null && d=$((d+1)); done
  [ "$d" -ge "$NW" ] && break
  sleep 30
done
echo "=== parallel batch finished; missing summaries: $(count_missing)/$TOTAL ==="

# cleanup pass: re-run sequentially (WORKERS=1) so only the failed items redo,
# one at a time, with minimal contention. Up to 2 cleanup rounds.
for round in 1 2; do
  m=$(count_missing)
  [ "$m" = 0 ] && break
  echo "=== cleanup round $round: $m missing -> sequential rerun ==="
  WORKERS=1 REPS=$REPS ./run_experiments_par.sh
done

echo "=== final missing: $(count_missing)/$TOTAL ==="
echo "=== GENERATING TABLE ==="
python3 gen_results_table.py
echo
echo "=== results_table.tex ==="
cat "$OUT/results_table.tex"
