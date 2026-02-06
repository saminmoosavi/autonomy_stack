https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate
https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc
The offboard computer will need a copy of the robot.yaml file to generate the same setup.bash file as the robot.

Create the folder
For the offboard computer we can create the clearpath folder in the home directory:

mkdir ~/clearpath/

Copy the robot.yaml file into the setup folder
nano robot.yaml in /home/user/clearpath/

note
If you have workspaces defined in the robot.yaml that do not exist on the offboard computer, remove them.

Generate the setup.bash file
source /opt/ros/humble/setup.bash
ros2 run clearpath_generator_common generate_bash -s /home/user/clearpath

Add the following line to your ~/.bashrc file to automatically source the generated setup.bash file in new terminals:
If you are running in docker, make sure you give docker access to this directory. 
Start the simulation
```bash
ros2 launch clearpath_gz simulation.launch.py
```
Drive the robot around a circle
```bash
ros2 topic pub /a200_0000/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.3}}"
```
Run the nav2 in simulation
```bash
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/
```
Run the SLAM in simulation
```bash
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a200_0000 use_sim_time:=true
```
Run realsense camera
```bash
sudo chmod a+rw /dev/video48 /dev/video49 /dev/video50 /dev/video51 /dev/video52 /dev/video53
ros2 launch realsense2_camera rs_launch.py serial_no:="'135122079298'"
```
Run Yolo, make sure the input camera is set correctly in the launch file.
for simulation, set it to /a200_0000/sensors/camera_0/color/image and for realsense camera to /camera/camera/color/image_raw. 
```bash
ros2 launch yolo_bringup yolo.launch.py
```

Run tracker with yolo:
```bash
ros2 run spine_ros2 tracker_with_yolo
```

Run SPINE:
```bash
ros2 launch spine_ros2 spine.launch.py ns:=a200_0000
```
-----------------------------------------------------------
#Tutorial on how to run the waypoint follower with nav2

