#!/usr/bin/env bash
# inspect_run.sh n1 n2 ... -- inspect-and-report trials; a trial whose YOLO
# nodes died at startup (CLIP download race) is stopped early and not counted.
S=$(cd "$(dirname "$0")" && pwd); R=$(cd "$S/../.." && pwd); L=${LOG_DIR:-$R/results/demo_logs}; mkdir -p "$L"
while docker ps --format '{{.Names}}' | grep -q '^evoplan-trial$'; do sleep 10; done
for n in "$@"; do
  TAG=fix$n
  ( sleep 240
    if grep -q "process has died" $R/results/single_trials/*_${TAG}_logs/yolo_*.log 2>/dev/null; then
      echo "$TAG: a YOLO node died at startup; stopping this trial"; docker rm -f evoplan-trial >/dev/null 2>&1
    fi ) &
  "$S/run_rec.sh" $TAG ${INSPECT_MISSION:-factory_survey_03} WORLD_NAME=warehouse_people0 "INSPECT_OBJECTS=traffic cone" \
     "YOLO_CLASSES=person,traffic cone,chair,suitcase,banana" MAX_SYMBOLIC_REPLANS=8 TRIAL_TIMEOUT=1800 \
     > "$L/loop_$TAG.out" 2>&1 < /dev/null
  wait
  python3 "$S/check_run.py" $TAG
  grep -hE "DISCOVER|INSPECTED|REPORTED|rejected locally|EXHAUST|MISSION COMPLETE|INSPECTION COMPLETE" $R/results/single_trials/*_${TAG}_logs/evo.log | cut -c60-230 | tail -14
  grep -q "REPORTED" $R/results/single_trials/*_${TAG}_logs/evo.log && grep -q "MISSION COMPLETE" $R/results/single_trials/*_${TAG}_logs/evo.log && { echo "GOOD $TAG"; break; }
done
echo LOOP_DONE
