#!/usr/bin/env bash
# Launch YOLO-world on all 4 cameras and set 'person' class once each node is ready.
NS=${NS:-/j100_0000}
# 0.7 (up from the launch default 0.5): low-confidence "person" hits on shelves
# become phantom tracks that trigger STL replans/goal cancels.
YOLO_THRESHOLD=${YOLO_THRESHOLD:-0.7}
for i in 0 1 2 3; do
  ros2 launch yolo_bringup yolo-world.launch.py \
    input_image_topic:=${NS}/sensors/camera_${i}/color/image namespace:=yolo_${i} \
    threshold:=${YOLO_THRESHOLD} &
done
for i in 0 1 2 3; do
  until ros2 service list 2>/dev/null | grep -q "/yolo_${i}/set_classes"; do sleep 2; done
  ros2 service call /yolo_${i}/set_classes yolo_msgs/srv/SetClasses "{classes: [person]}"
done
wait
