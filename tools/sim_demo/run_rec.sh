#!/usr/bin/env bash
# run_rec.sh TAG MISSION [extra env assignments...] -- one recorded trial.
TAG=$1; MISSION=$2; shift 2
S=$(cd "$(dirname "$0")" && pwd); R=$(cd "$S/../.." && pwd); cd $R
L=${LOG_DIR:-$R/results/demo_logs}; mkdir -p "$L"
# Orphaned FastDDS shared-memory files from killed runs break discovery between
# some node pairs (Nav2's controller never sees map->odom). Clear the ones no
# process holds open.
inuse=$(lsof +D /dev/shm 2>/dev/null | awk 'NR>1{print $NF}' | grep fastrtps | sort -u)
for f in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
  [ -e "$f" ] || continue; echo "$inuse" | grep -qx "$f" || rm -f "$f"
done
mkdir -p results/recordings; rm -rf results/recordings/$TAG results/recordings/${TAG}_* results/recordings/$TAG.log
env -u OPENAI_API_KEY MISSION=$MISSION CROWD= WORLD_NAME=warehouse_people PLANNER_MODE=fd_only TAG=$TAG \
    TRIAL_TIMEOUT=2400 SIM_DISPLAY=:1 "$@" ./pipeline/run_trial.sh > $L/trial_$TAG.log 2>&1 &
TP=$!
for i in $(seq 1 80); do grep -q "waiting for robot" $L/trial_$TAG.log 2>/dev/null && break; kill -0 $TP 2>/dev/null || break; sleep 2; done
docker exec -d evoplan-trial bash -lc "export ROS_LOCALHOST_ONLY=1; source /opt/ros/humble/setup.bash; source /home/user/autonomy_stack_ros_humble/install/setup.bash; cd /home/user/autonomy_stack_ros_humble; python3 record_run.py --ns /j100_0000 --out results/recordings/$TAG --fps 6 --cameras 0 > results/recordings/$TAG.log 2>&1"
W=""
for i in $(seq 1 60); do W=$(xwininfo -display :1 -root -tree 2>/dev/null | grep '"Gazebo": ("ign-gazebo-gui"' | awk '{print $1}' | head -1); [ -n "$W" ] && break; sleep 2; done
FP=""
if [ -n "$W" ]; then
  sleep 5
  date +%s.%N > results/recordings/${TAG}_gz_start.txt
  ffmpeg -nostdin -y -loglevel error -f x11grab -framerate 10 -window_id $W -i :1 -t 2400 -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p results/recordings/${TAG}_gazebo.mkv > $L/ffgrab_$TAG.log 2>&1 &
  FP=$!
fi
# Chase camera: place the GUI camera behind the robot directly, about twice a
# second. Gazebo's own /gui/follow lags by metres whenever the GUI renders
# slowly and leaves the camera inside the shelving.
for i in $(seq 1 60); do
  docker exec evoplan-trial bash -lc 'export IGN_IP=127.0.0.1; ign model -m j100_0000/robot -p 2>/dev/null | grep -q XYZ' && break
  sleep 4
done
docker exec -d evoplan-trial bash -lc 'cd /home/user/autonomy_stack_ros_humble; tools/sim_demo/chase_cam.sh > /tmp/chase.log 2>&1' && echo "chase camera started"
wait $TP; echo "TRIAL_RC=$?"
[ -n "$FP" ] && kill -INT $FP 2>/dev/null; sleep 2
sed -n '/\[trial\].*results/,$p' $L/trial_$TAG.log | cut -c1-160
