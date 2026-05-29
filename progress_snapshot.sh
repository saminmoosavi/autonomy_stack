#!/usr/bin/env bash
# Wait until the completed-run count rises above $1 (or ~18 min elapse), then
# print a one-shot progress snapshot of the factory-mission batch and exit.
# Used to drive periodic progress updates.
set -uo pipefail
cd /home/mvarun/Research/temp/autonomy_stack
OUT=results/factory_missions
BASE=${1:-0}
REPS=${REPS:-3}
TOTAL=$((5 * REPS))

completed() {
  local n=0 p r sm
  for p in 01 02 03 04 05; do for r in $(seq 1 "$REPS"); do
    sm=$OUT/plan${p}_rep${r}.summary
    { [ -s "$sm" ] && grep -q "SCAND-shield runtime metrics" "$sm"; } && n=$((n+1))
  done; done
  echo "$n"
}

# wait for a new completion or timeout (~18 min)
for i in $(seq 1 18); do
  c=$(completed); [ "$c" -gt "$BASE" ] && break
  # also break if the whole batch already finalized
  grep -q "results_table.tex" /tmp/run_finalize.out 2>/dev/null && break
  sleep 60
done

NOW=$(completed)
echo "=== PROGRESS SNAPSHOT ($NOW/$TOTAL runs complete) ==="
echo "load: $(uptime | sed 's/.*load average/load/')"
echo
echo "-- completed runs --"
for p in 01 02 03 04 05; do for r in $(seq 1 "$REPS"); do
  sm=$OUT/plan${p}_rep${r}.summary
  if [ -s "$sm" ] && grep -q "SCAND-shield runtime metrics" "$sm"; then
    s=$(sed -E 's/^\[[^]]*\] //' "$sm")
    succ=$(echo "$s" | grep -oE "succ[^:]*: (yes|no)" | grep -oE "(yes|no)$")
    dur=$(echo "$s" | grep -oE "duration=[0-9.]+s" | head -1)
    pl=$(echo "$s" | grep -oE "path_length=[0-9.]+" | head -1)
    coll=$(echo "$s" | grep -oE "coll[^:]*: [0-9]+" | grep -oE "[0-9]+$")
    pers=$(echo "$s" | grep -oE "pers[^(]*\([0-9.]+%\)" | grep -oE "[0-9.]+%")
    printf "  M%s r%s: succ=%s coll=%s %s %s pers=%s\n" "$((10#$p))" "$r" "${succ:-?}" "${coll:-?}" "${dur:-}" "${pl:-}" "${pers:-?}"
  fi
done; done
echo
echo "-- in-progress workers --"
for k in 1 2; do
  cur=$(tail -n1 /tmp/exp_par_w$k.log 2>/dev/null | grep -oE "plan[0-9]+ rep[0-9]+" | tail -1)
  f=$(ls -t $OUT/_log/w$k/p*_r*/runsim.log 2>/dev/null | head -1)
  st=$(grep -aoh "pipeline is UP\|ERROR: Nav2 did not activate\|Nav2 to activate\|waiting for robot" "$f" 2>/dev/null | tail -1)
  ev=$(ls -t $OUT/_log/w$k/p*_r*/evo.log 2>/dev/null | head -1)
  ticks=$(grep -ac "STL monitor satisfied" "$ev" 2>/dev/null)
  printf "  w%s: %s [%s] stl-ticks=%s\n" "$k" "${cur:-(between runs)}" "${st:-launching}" "${ticks:-0}"
done