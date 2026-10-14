#!/usr/bin/env bash
# loop.sh "<env assignments>" n1 n2 ... -- recorded trials until one completes with a replan
S=$(cd "$(dirname "$0")" && pwd); L=${LOG_DIR:-$S/../../results/demo_logs}; mkdir -p "$L"; EXTRA=$1; shift
for n in "$@"; do
  "$S/run_rec.sh" fix$n ${MISSION:-factory_mission_08} MAX_SYMBOLIC_REPLANS=4 $EXTRA > "$L/loop_fix$n.out" 2>&1 < /dev/null
  python3 "$S/check_run.py" fix$n
done
echo LOOP_DONE
