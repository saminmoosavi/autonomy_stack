#!/usr/bin/env bash
# Launch YOLO-world on all 4 cameras and set the target classes once each node
# is ready. yolo-world is open-vocabulary: it detects NOTHING until set_classes
# is called, so this list is the primary filter on everything downstream.
NS=${NS:-/j100_0000}
# 0.7 (up from the launch default 0.5): low-confidence "person" hits on shelves
# become phantom tracks that trigger STL replans/goal cancels. Note yolo_ros has
# a single global threshold -- there are no per-class thresholds -- so this one
# value has to cover every class below.
YOLO_THRESHOLD=${YOLO_THRESHOLD:-0.7}
# Matches the graph.json object types so observation_logger can compare expected
# vs observed per region. Non-human classes do NOT become STL obstacles --
# evo_plan_deploy drops them at eveo_plan_deploy.py:891 -- they only add STL
# monitor invocations and evo.log noise. Use YOLO_CLASSES=person for
# person-only runs; to drop classes from the log only, use config/exclude.json.
YOLO_CLASSES=${YOLO_CLASSES:-person,chair,table,shelf,column,box,pallet}
for i in 0 1 2 3; do
  ros2 launch yolo_bringup yolo-world.launch.py \
    input_image_topic:=${NS}/sensors/camera_${i}/color/image namespace:=yolo_${i} \
    threshold:=${YOLO_THRESHOLD} &
done
for i in 0 1 2 3; do
  until ros2 service list 2>/dev/null | grep -q "/yolo_${i}/set_classes"; do sleep 2; done
  ros2 service call /yolo_${i}/set_classes yolo_msgs/srv/SetClasses "{classes: [${YOLO_CLASSES}]}"
done
wait
