# Install:
First, build the Docker container. Change directory into /autonomy_stack_ros_humble and run:
```bash
docker compose build
```
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
## Husky simulation
Start the simulation
```bash
ros2 launch clearpath_gz simulation.launch.py
```
To test the simulation, drive the robot in a circle
```bash
ros2 topic pub /a200_0000/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.3}}"
```
```bash
ros2 topic pub -r 10 /j100_0611/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

If it passed the test, launch the nav2 in simulation
```bash
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/
```
Run the SLAM package in simulation
```bash
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/
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
ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=/home/user/jackal_setup/ use_sim_time:=false 
ros2 launch clearpath_nav2_demos slam.launch.py setup_path:=/home/user/jackal_setup/ use_sim_time:=false
ros2 launch clearpath_nav2_demos localization.launch.py map:=/home/user/autonomy_stack_ros_humble/lens_lab_map.yaml setup_path:=/home/user/jackal_setup/ use_sim_time:=false 
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/j100_0611 use_sim_time:=false
```
check tf
```bash
ros2 run warthog_nav2_bringup scan_relay 
ros2 run tf2_ros tf2_echo base_link lidar2d_0_laser --ros-args -r /tf:=/w200_0105/tf -r /tf_static:=/w200_0105/tf_static
ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=/home/user/warthog_setup/
ros2 launch clearpath_nav2_demos slam.launch.py setup_path:=/home/user/warthog_setup/
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/w200_0105
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


