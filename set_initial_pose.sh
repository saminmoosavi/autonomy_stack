#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Publish AMCL's initial pose from the command line — the CLI equivalent of
# RViz's "2D Pose Estimate" button. Useful when RViz cannot draw the robot
# (no map->odom yet, so nothing renders) or when you want a repeatable start.
#
#   ./set_initial_pose.sh                 # main_door, the office mission start
#   ./set_initial_pose.sh 9.03 6.51 0.0   # x y yaw(rad)
#   ./set_initial_pose.sh --region main_center
#
# Run it AFTER `loc` prints "Managed nodes are active" — /initialpose is not
# latched, so a message published before amcl subscribes is simply lost.
#
# Office regions (graph_office.json):
#   main_door 6.39 8.04 | main_center 9.03 6.51 | doorway 12.64 8.39
#   conference_door 15.02 8.75 | conference_center 14.36 6.26
#   main_empty_wall 10.88 5.03
# ---------------------------------------------------------------------------
set -uo pipefail

NS="${NS:-/j100_0612}"

declare -A REGION=(
  [main_door]="6.39 8.04"     [main_center]="9.03 6.51"
  [doorway]="12.64 8.39"      [conference_door]="15.02 8.75"
  [conference_center]="14.36 6.26" [main_empty_wall]="10.88 5.03"
)

if [[ "${1:-}" == "--region" ]]; then
  R="${2:-main_door}"
  [[ -n "${REGION[$R]:-}" ]] || { echo "unknown region: $R" >&2; exit 2; }
  read -r X Y <<<"${REGION[$R]}"
  YAW="${3:-0.0}"
else
  X="${1:-6.39}"; Y="${2:-8.04}"; YAW="${3:-0.0}"
fi

# Yaw -> quaternion about z.
read -r QZ QW < <(python3 -c "
import math
y = float('$YAW')
print(math.sin(y/2.0), math.cos(y/2.0))
")

# Covariance: RViz's own defaults for the pose-estimate tool. x/y 0.25 m^2 and
# yaw 0.0685 rad^2 tell AMCL how wide to spread its particles — leave them at
# zero and the filter starts overconfident and cannot recover from a bad guess.
COV="[0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]"

echo "=== initial pose -> $NS/initialpose ==="
echo "  x=$X  y=$Y  yaw=$YAW rad  (qz=$QZ qw=$QW), frame=map"

# -w 1: wait for amcl to actually subscribe. --times 3: /initialpose is a plain
# reliable-volatile topic, so a single publish that races the subscription match
# is dropped silently.
ros2 topic pub -w 1 --times 3 -r 2 "$NS/initialpose" \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'},
    pose: {pose: {position: {x: $X, y: $Y, z: 0.0},
                  orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}},
           covariance: $COV}}"

echo
echo "=== did it take? map -> odom must now exist ==="
timeout 5 ros2 run tf2_ros tf2_echo map odom \
  --ros-args -r /tf:="$NS/tf" -r /tf_static:="$NS/tf_static" 2>&1 \
  | grep -A3 "At time" | head -5
echo "(if that is empty, amcl is still not processing scans — the pose was not the problem)"
