# Install:
First, fetch the submodules (the planning/detection packages are git submodules):
```bash
git submodule update --init --recursive
```
**Note:** at minimum you need `src/planning_ros_pkgs/evo_skill` and `src/detection_ros_pkgs/yolo_ros` (provides `yolo_msgs`, required by `evo_skill_ros`).

Next, build the Docker container. Change directory into /autonomy_stack_ros_humble and run:
```bash
docker compose build --build-arg UNAME=user --build-arg UID=$(id -u) --build-arg GID=$(id -g)
```
**Why the build args:** the workspace is bind-mounted into the container, so the container user must share your host UID/GID or you get permission errors on the mounted files. The Dockerfile defaults to `1000:1000`; the args above match the container `user` to your host account (works for any UID, including AD/LDAP accounts that are not 1000). `UID` is a read-only bash variable, so pass it explicitly via `$(id -u)` rather than `export UID`.

Once the Docker image is built properly, start the container by running:
```bash
docker compose -f ~/autonomy_stack_ros_humble/docker-compose.yml run --rm  ros_humble
```
Next, build the ROS packages:
```bash
build
```
To source the ROS workspace:
```bash
sc
```
**Note: build and sc are aliases. You can find them in the /docker/.bashrc file.

# Simulation setup
The simulation is based on Clearpath packages. Full instructions can be found here:

https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate
https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc

Below is a snapshot of the main elements required for our setup.
The computer needs a copy of the robot.yaml file to generate the same setup.bash file used by the robot.
Create the clearpath folder in your home directory:

```bash
mkdir ~/clearpath/
```
Copy the robot.yaml file into the setup folder.

**Note: If you have workspaces defined in robot.yaml that do not exist on the offboard computer, remove them.

Generate the setup.bash file:
```bash
source /opt/ros/humble/setup.bash
ros2 run clearpath_generator_common generate_bash -s /home/user/clearpath
```


Add the following line to your ~/.bashrc file to automatically source the generated setup.bash file in new terminals:
If you are running in docker, make sure you give docker access to this directory.
# Run navigation and SLAM 

## Quick start: full simulation pipeline (one command)
`run_sim.sh` brings up the entire verified pipeline inside the container — Gazebo (Jackal) → Nav2 + SLAM → lidar scan relay → `evo_skill` plan deploy — with all the host-specific fixes already applied (loopback DDS, loopback gz-transport, 3D→2D scan relay). The robot drives the PDDL/STL plan in the warehouse world.

```bash
# on the HOST (once), so the Gazebo GUI can reach your X server:
xhost +local:

# inside the container:
./run_sim.sh
```
Useful env overrides: `NS` (robot namespace, default `/j100_0000`), `WORLD` (default `warehouse`), `TARGET` (goal region, default `R10`), `RVIZ=true`, `NO_EVO=1` (bring up sim + Nav2 + SLAM only). Logs are written to `/tmp/evo_sim/{sim,nav2,slam,relay,evo}.log`. Press `Ctrl-C` to tear the whole pipeline down. Verify the robot is moving with `ign model -m j100_0000/robot -p` (run twice and compare the pose).

The sections below explain each stage manually (and the fixes the script applies for you).

## Husky simulation
Start the simulation
```bash
ros2 launch clearpath_gz simulation.launch.py
```

**If the robot never spawns** (Gazebo opens but the robot is missing, and the logs loop on `Requesting list of world names` / `Waited for 10s for a subscriber to /gazebo/starting_world and got none`): this happens on hosts with multiple network interfaces (VPNs, bridges) because gz-transport picks the wrong one. Force it onto loopback before launching:
```bash
export IGN_IP=127.0.0.1     # Gazebo Garden/Harmonic uses GZ_IP instead
ros2 launch clearpath_gz simulation.launch.py
```

**For the Gazebo GUI (X11 forwarding from the container):** the container's `XAUTHORITY` may point to a path that is not mounted, so the GUI cannot reach the X server. On the host, allow local connections, then unset the stale auth inside the container:
```bash
# on the HOST:
xhost +local:
# inside the CONTAINER, before launching:
export DISPLAY=:1            # match your host DISPLAY
unset XAUTHORITY
```
To test the simulation, drive the robot in a circle
```bash
ros2 topic pub /a200_0000/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.3}}"
```
```bash
ros2 topic pub -r 10 /j100_0611/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

**Before launching Nav2/SLAM on a multi-NIC host (VPN, bridges):** export `ROS_LOCALHOST_ONLY=1` in *every* terminal (sim, Nav2, SLAM, evo). Otherwise the default DDS drops traffic and Nav2's `lifecycle_manager` hangs forever on "Configuring controller_server". Pinning DDS to loopback fixes it.
```bash
export ROS_LOCALHOST_ONLY=1
```

If it passed the test, launch the nav2 in simulation
```bash
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/
```
Run the SLAM package in simulation
```bash
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/
```
**Scan topic relay (required in sim):** the sim Jackal has only a 3D lidar publishing `/<ns>/sensors/lidar3d_0/scan`, but Clearpath SLAM/Nav2 subscribe to `/<ns>/sensors/lidar2d_0/scan` (no publisher), and `slam.launch.py` has no `scan_topic` argument. Without scans, SLAM never builds a map and Nav2's costmaps stay inactive. Relay the 3D scan onto the 2D topic (`scan_relay.py` is a tiny rclpy node in the repo root, used because `topic_tools` is not installed):
```bash
python3 ~/autonomy_stack_ros_humble/scan_relay.py \
  /j100_0000/sensors/lidar3d_0/scan /j100_0000/sensors/lidar2d_0/scan
```
View the path and maps in  RViz
```bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a200_0000 use_sim_time:=true
```
save the map 
```bash
ros2 run nav2_map_server map_saver_cli -f "factory_sim_map" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/a200_0000
```
run localization in a pre built map
```bash
ros2 launch clearpath_nav2_demos localization.launch.py map:=/home/user/autonomy_stack_ros_humble/factory_sim_map.yaml use_sim_time:=true setup_path:=/home/user/clearpath/
```
## Jackal robot
For a physical clearpath Jackal, copy the robot.yaml into ~/jackal_setup.Then inside the docker container generate the setup file. 
```bash
mkdir ~/jackal_setup/
ros2 run clearpath_generator_common generate_bash -s /home/user/jackal_setup
source /opt/ros/humble/setup.bash
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=false setup_path:=/home/user/jackal_setup/
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=false setup_path:=/home/user/jackal_setup/
ros2 launch clearpath_nav2_demos localization.launch.py map:=/home/user/autonomy_stack_ros_humble/lens_lab_map.yaml use_sim_time:=false setup_path:=/home/user/jackal_setup/
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/j100_0611 use_sim_time:=false
```

with ouster sensor:
```bash
ros2 launch clearpath_nav2_demos nav2.launch.py scan_topic:=/j100_0611/sensors/lidar3d_0/scan setup_path:=/home/user/jackal_setup/ use_sim_time:=false
ros2 launch clearpath_nav2_demos slam.launch.py scan_topic:=/j100_0611/sensors/lidar3d_0/scan setup_path:=/home/user/jackal_setup/ use_sim_time:=false


```
# Run YOLO
If running in simulation, skip directly to launching YOLO.

For a physical RealSense camera, start the RealSense drivers
```bash
ros2 launch realsense2_camera rs_launch.py serial_no:="'135122079298'"
```
If you encounter permission issues, try:
```bash
sudo chmod a+rw /dev/video48 /dev/video49 /dev/video50 /dev/video51 /dev/video52 /dev/video53
or
sudo chmod 777 /dev/video*
```
To run YOLO, make sure the input camera topic is set correctly:

  - For simulation: /a200_0000/sensors/camera_0/color/image
  - For RealSense camera: /camera/camera/color/image_raw
  - For jackal change topic to: /j100_0611/sensors/camera_0/color/image

```bash
ros2 launch yolo_bringup yolo-world.launch.py input_image_topic:=/a200_0000/sensors/camera_0/color/image
```
# Run SPINE
When working with YOLO, detections must be projected into 3D coordinates. This is done by associating detected bounding boxes with the depth camera. The coordinates are then transformed into the map frame.

Launch Spine:
```bash
ros2 launch spine_ros2 spine.launch.py ns:=a200_0000
```
To give a mission goal to it 
```bash
ros2 service call /a200_0000/region_goal spine_interface_ros2/srv/Task task:" R2"
```

# Troubleshooting
## Docker build fails with "At least one invalid signature was encountered" / "repository is not signed"
If `apt-get update` fails this way for **all** repositories at once (ubuntu, security, ros, nvidia) during the build, it is almost never a real GPG problem — it means the disk is **full**, which truncates apt's downloaded signature files. Free space and rebuild. Safe reclaims:
```bash
docker builder prune -af          # reclaim build cache
docker rmi ubuntu-22-humble:latest   # old image is regenerated by the build anyway
df -h /                            # confirm free space (a full rebuild needs ~18-20 GB)
```
Note: pruning all build cache forces every layer to rebuild from scratch, which makes the final "exporting layers" step slow (it must gzip + sha256 ~18 GB of fresh layers). A cached rebuild exports almost instantly.

## Permission errors on mounted workspace files
Rebuild the image with your host UID/GID — see the build-arg note in **Install** above.

## Gazebo opens but the robot never spawns
gz-transport picked the wrong network interface. `export IGN_IP=127.0.0.1` (Garden/Harmonic: `GZ_IP`) before launching. See the simulation section.

## Nav2 lifecycle_manager hangs on "Configuring controller_server"
The configure response was lost over DDS on a multi-NIC host. `export ROS_LOCALHOST_ONLY=1` in every terminal (all nodes must share it) and relaunch.

## SLAM never builds a map / Nav2 costmaps "no map received" / "frame map does not exist"
SLAM is subscribed to `lidar2d_0/scan`, but the sim Jackal only publishes `lidar3d_0/scan`. Run the scan relay (see the SLAM step above). `run_sim.sh` does this automatically.


