### Docker
Run docker image
``` bash
docker compose -f ~/autonomy_stack/docker-compose.yml run --rm ros_noetic
```
### Simulation
launch gazebo
```bash
roslaunch jackal_gazebo jackal_world.launch
```
with laser scan 

```bash
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
```

launch RViz
```bash
roslaunch jackal_viz view_robot.launch
```
### Setup the nerwork
Setting up ROS master at Jackal's IP address! Add the following line to .bashrc:
```bash
export ROS_MASTER_URI=http://192.168.131.1:11311/  # Jackal
export ROS_HOSTNAME=192.168.131.50 # This computer
export ROS_IP=192.168.131.50 # This computer
```
Add this Jackal
```bash
export ROS_MASTER_URI=http://192.168.131.1:11311/  # Jackal
export ROS_HOSTNAME=192.168.131.1 # This computer
export ROS_IP=192.168.131.1 # This computer
```
### Navigation
Without the map
```bash
roslaunch jackal_navigation odom_navigation_demo.launch
roslaunch jackal_viz view_robot.launch config:=navigation

```
To send goals to the robot, select the 2D Nav Goal tool from the top toolbar, and then click anywhere in the rviz view to set the position. Alternatively, click and drag slightly to set the goal position and orientation.

Making a map
```bash
roslaunch jackal_navigation gmapping_demo.launch
roslaunch jackal_viz view_robot.launch config:=gmapping
```

Navigate with map

```bash
roslaunch jackal_navigation amcl_demo.launch map_file:=/path/to/my/map.yaml
roslaunch jackal_viz view_robot.launch config:=localization
```

