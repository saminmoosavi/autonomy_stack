# Evo Skill ROS

Instructions for running `evo_plan_run.launch.py` inside Docker.

## Build and Source

From the workspace root inside the Docker container:

```bash
cd /home/samin/autonomy_stack_ros_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select evo_skill_ros
source install/setup.bash
```

Resolve the installed config directory:

```bash
EVO_CFG="$(ros2 pkg prefix evo_skill_ros)/share/evo_skill_ros/config"
EVO_RVIZ="$(ros2 pkg prefix evo_skill_ros)/share/evo_skill_ros/rviz/evo_plan_navigation.rviz"
```

## Run in Simulation

```bash
ros2 launch evo_skill_ros evo_plan_run.launch.py \
  namespace:=/a200_0000 \
  robot_name:=jackal_1 \
  target_region:=R10 \
  graph_file:=$EVO_CFG/graph.json \
  domain_file:=$EVO_CFG/factory_sim_domain.pddl \
  plan_file:=$EVO_CFG/evoskill_plan_sim.txt \
  costmap_edit_max_radius:=1.0 \
  require_map:=false
```

## Run on the Actual Robot

```bash
ros2 launch evo_skill_ros evo_plan_run.launch.py \
  namespace:=/j100_0611 \
  robot_name:=jackal_1 \
  target_region:=fire \
  graph_file:=$EVO_CFG/graph_lab.json \
  domain_file:=$EVO_CFG/lens_lab_domain.pddl \
  plan_file:=$EVO_CFG/evoskill_plan_lab.txt \
  costmap_edit_max_radius:=1.0 \
  require_map:=true \
  tracking_topic:=/yolo/tracking \
  points_topic:=/sensors/camera_0/points \
  odom_topic:=/platform/odom/filtered \
  tracker_out_topic:=/tracks
```

## Notes

`require_map:=true` makes `evo_plan_deploy` wait for an occupancy grid on
`<namespace>/map` before sending the plan. Use `require_map:=false` when testing
without a ROS map.

The lab graph does not need a `start` region. The first move in
`evoskill_plan_lab.txt` can use `start` as a placeholder; at runtime,
`evo_plan_deploy` replaces it with the robot's live nearest graph region, or
skips that first move if the robot is already at the first target region.

This launch path loads the provided `plan_file` and dispatches it without
blocking on symbolic PDDL validation, so robot execution trials can be used to
measure the plan success rate. Fast Downward is only needed if the node is
changed to compute a new PDDL plan at runtime.

STL obstacle constraints are only generated for tracked human obstacles.
STL-triggered costmap inflation is capped at `costmap_edit_max_radius`.

## RViz

The RViz config includes the factory graph markers from `/factory_world_markers`
and the Nav2 waypoint path from `/ppddl_nav2_goals`.

```bash
ros2 launch clearpath_viz view_navigation.launch.py rviz_config:=$EVO_RVIZ
```

If your Clearpath launch file uses a different RViz config argument, check it
with:

```bash
ros2 launch clearpath_viz view_navigation.launch.py --show-args
```

You can also open the config directly:

```bash
rviz2 -d $EVO_RVIZ
```
