all terminals:

docker compose -f /planning/autonomy_stack/docker-compose.yml run --rm ros_humble
sc
cp /planning/autonomy_stack/robot_4cam.yaml ~/clearpath/robot.yaml
sc && export ROS_LOCALHOST_ONLY=1

terminal 1:
# IGNORE
# reduce human speed

sudo sed -i "s|<horizontal_fov>1.25</horizontal_fov>|<horizontal_fov>2.0</horizontal_fov>|"     /opt/ros/humble/share/clearpath_sensors_description/urdf/intel_realsense.urdf.xacro

# reduce robot speed

sudo sed -i -e 's/max_vel_x: 1\.0/max_vel_x: 0.5/' -e 's/max_vel_theta: 1\.0/max_vel_theta: 0.5/' -e 's/max_speed_xy: 1\.0/max_speed_xy: 0.5/' -e 's/max_velocity: \[1\.0, 0\.0, 1\.0\]/max_velocity: [0.5, 0.0, 0.5]/' -e 's/min_velocity: \[-1\.0, 0\.0, -1\.0\]/min_velocity: [-0.5, 0.0, -0.5]/' -e 's/max_rotational_vel: 1\.0/max_rotational_vel: 0.5/' -e 's/min_rotational_vel: 0\.2/min_rotational_vel: 0.1/' /opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml
# ENDIGNORE

export IGN_IP=127.0.0.1 && ros2 launch clearpath_gz simulation.launch.py world:=/home/user/autonomy_stack_ros_humble/worlds/warehouse_people10 y:=1.0

terminal 2:

# j100 only has lidar3d — relay it to lidar2d_0/scan so AMCL gets scans.
# wait for the robot to appear in Gazebo (terminal 1) before running this.
python3 /home/user/autonomy_stack_ros_humble/scan_relay.py /j100_0000/sensors/lidar3d_0/scan /j100_0000/sensors/lidar2d_0/scan --ros-args -p use_sim_time:=true

terminal 3:

ros2 launch clearpath_nav2_demos localization.launch.py map:=/home/user/autonomy_stack_ros_humble/factory_sim_map.yaml use_sim_time:=true setup_path:=/home/user/clearpath/

terminal 4:

ros2 launch clearpath_viz view_robot.launch.py namespace:=j100_0000 use_sim_time:=true config:=nav2.rviz

terminal 5:

ros2 launch /home/user/autonomy_stack_ros_humble/nav2_custom.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/

terminal 6:

/home/user/autonomy_stack_ros_humble/start_yolo.sh

terminal 7:
sc && export ROS_LOCALHOST_ONLY=1

ros2 run evo_skill_ros tracker_with_yolo --ros-args -r __node:=tracker_with_yolo_cam1 -r __ns:=/j100_0000 -p namespace:=/j100_0000 -p tracking_topic:=/yolo_1/tracking -p points_topic:=/sensors/camera_1/points -p out_topic:=/tracks -p target_frame:=map -p use_sim_time:=true -r /tf:=tf -r /tf_static:=tf_static &
ros2 run evo_skill_ros tracker_with_yolo --ros-args -r __node:=tracker_with_yolo_cam2 -r __ns:=/j100_0000 -p namespace:=/j100_0000 -p tracking_topic:=/yolo_2/tracking -p points_topic:=/sensors/camera_2/points -p out_topic:=/tracks -p target_frame:=map -p use_sim_time:=true -r /tf:=tf -r /tf_static:=tf_static &
ros2 run evo_skill_ros tracker_with_yolo --ros-args -r __node:=tracker_with_yolo_cam3 -r __ns:=/j100_0000 -p namespace:=/j100_0000 -p tracking_topic:=/yolo_3/tracking -p points_topic:=/sensors/camera_3/points -p out_topic:=/tracks -p target_frame:=map -p use_sim_time:=true -r /tf:=tf -r /tf_static:=tf_static &

terminal 8:

# WORLD must match the world path in terminal 1 (no .sdf extension).
# If you change the world in terminal 1, update WORLD here too — actors_sdf
# must point to the same file or phantom actor positions will corrupt metrics.
WORLD=/home/user/autonomy_stack_ros_humble/worlds/warehouse_people10
EVO_CFG="/home/user/autonomy_stack_ros_humble/src/planning_ros_pkgs/evo_skill_ros/config" && ros2 launch evo_skill_ros evo_plan_run.launch.py     namespace:=/j100_0000     robot_name:=jackal_1     target_region:=R10     graph_file:=$EVO_CFG/graph.json     domain_file:=$EVO_CFG/factory_sim_domain.pddl     plan_file:=/home/user/plan.txt     tracks_topic:=/j100_0000/tracks     tracking_topic:=/yolo_0/tracking     require_map:=false enable_metrics:=true metrics_world:=warehouse metrics_duration:=3000 metrics_stop_on_success:=false     metrics_actors_sdf:=${WORLD}.sdf     json_log_file:=/home/user/autonomy_stack_ros_humble/evo_plan_deploy_log.json

# metrics JSON is written to scand_metrics_out.json when terminal 8 finishes (or is Ctrl-C'd).
# planner event log is written to evo_plan_deploy_log.json continuously during the run.

post experiment (host, outside container):

# single trial — register under a plan/rep name then build the table:
WS=/planning/autonomy_stack
cp /home/user/autonomy_stack_ros_humble/scand_metrics_out.json \
   $WS/results/factory_missions/plan01_rep1_metrics.json
python3 $WS/gen_results_table.py
# writes results/factory_missions/results_table.tex and prints a plaintext preview

# batch experiments — run all trials then finalise:
cd $WS && ./run_experiments_par.sh        # runs 5 plans x 3 reps across 4 workers
cd $WS && ./run_finalize.sh               # waits, retries failures, calls gen_results_table.py
