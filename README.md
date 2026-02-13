https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate
https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc
The offboard computer will need a copy of the robot.yaml file to generate the same setup.bash file as the robot.
# Install:
First build the docker container by cd into /autonomy_stack_ros_humble directory and run 
```bash
docker compose build
```
Once the doecker is build properly, initiate the docker container by 
```bash
docker compose -f ~/autonomy_stack_ros_humble/docker-compose.yml run --rm  ros_humble
```
Next, we need to build the ros packages.
```bash
build
```
To source the ros workspace
```bash
sc
```
**note: build and sc are aliases , and you can find them in /docker/.bashrc file. 

# Simulation setup
The simulation is based on clearpath packages and the full instruction can be found here:
https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/simulate
https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc

Here I will present snapshot of main elements needed for our problem.
The (offboard) computer will need a copy of the robot.yaml file to generate the same setup.bash file as the robot.
Create the folder
We can create the clearpath folder in the home directory:

```bash
mkdir ~/clearpath/
```
Copy the robot.yaml file into the setup folder
***note: If you have workspaces defined in the robot.yaml that do not exist on the offboard computer, remove them.
Generate the setup.bash file
```bash
source /opt/ros/humble/setup.bash
ros2 run clearpath_generator_common generate_bash -s /home/user/clearpath
```

Add the following line to your ~/.bashrc file to automatically source the generated setup.bash file in new terminals:
If you are running in docker, make sure you give docker access to this directory.
# Run navigation and slam 
Start the simulation
```bash
ros2 launch clearpath_gz simulation.launch.py
```
To test the simulation, drive the robot around a circle
```bash
ros2 topic pub /a200_0000/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.3}}"
```
If it passed the etst, run the nav2 in simulation
```bash
ros2 launch clearpath_nav2_demos nav2.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/
```
Run the SLAM package in simulation
```bash
ros2 launch clearpath_nav2_demos slam.launch.py use_sim_time:=true setup_path:=/home/user/clearpath/
```
View the path and maps in rviz
```bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=/a200_0000 use_sim_time:=true
```
# Run yolo
If in simulation skip to run yolo directly. For a physical realsense camera, run realsense drivers by
```bash
ros2 launch realsense2_camera rs_launch.py serial_no:="'135122079298'"
```
To solve permission issues, you can try 
```bash
sudo chmod a+rw /dev/video48 /dev/video49 /dev/video50 /dev/video51 /dev/video52 /dev/video53
```
To run yolo, make sure the input camera is set correctly. For simulation, set it to /a200_0000/sensors/camera_0/color/image and for realsense camera to /camera/camera/color/image_raw. 
```bash
ros2 launch yolo_bringup yolo-world.launch.py input_image_topic:=/a200_0000/sensors/camera_0/color/image
```
# Run Spine
If working with yolo, we need to proejct the detections into 3d coordinate. This is done by associating the detected bounding boxes with the depth camera. We then use the coordinate transfrom, to convert them into map frame.
```bash
ros2 run spine_ros2 tracker_with_yolo
```
Once the labes are generated and projected by tracker and yolo, we can run the spine
```bash
ros2 launch spine_ros2 spine.launch.py ns:=a200_0000
```



## List of commands
```bash
