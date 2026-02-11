# Turtlebot3 Nav PickAndPlace

This is a ROS 2 package that is part of my thesis work. 
It provides a set of launch files and nodes that demonstrate the potential of autonomous robot manipulation
with open source technologies such as ROS 2 itself, the Nav2 navigation stack, and the MoveIt 2 framework. The 
demonstrations include a static, "on the spot" pick and place scenario and a dynamic, "move into position" 
pick and place scenario.
Both scenarios may be visualized in RViz and Gazebo Classic. 

It is tested on Ubuntu 22.04, ROS 2 Humble and Gazebo Classic.

## Setup
This package requires to be cloned into a workspace, along with the turtlebot2 simulation packages and the IFRA_LinkAttacher package (which, in short, allows to simulate the grasp without having to dive into friction values
in the sdf models for Gazebo).
``` bash
mkdir -p workspace_name/src
cd workspace_name/src
git clone https://github.com/MatteoVignaga/Turtlebot3_Nav_PickAndPlace.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
```

Then compile and source the setup:
``` bash
cd ..
colcon build symlink-install
source install/setup.bash
```

## Usage
### Static pick and place

In one terminal, spawn the robot in Gazebo and start RViz visualization through the launchfile:
``` bash
ros2 launch tb3_nav_pick_and_place pick_and_place.launch.py
```

In a second terminal, run the node that spawns the planning scene for
moveit and executes the pick and place sequence:
``` bash
ros2 run tb3_nav_pick_and_place static_pick_and_place
```

![Static pick and place img](/img/static_pick_and_place.png)

Alternatively, it is also possible to run a trial version with lower tables (discarded for the 
dynamic pick and place as the lidar wouldn't be able to see them).
``` bash
ros2 launch tb3_nav_pick_and_place basic_empty_pick_and_place.launch.py
ros2 run tb3_nav_pick_and_place basic_pick_and_place
```

### Dynamic pick and place

In one terminal, spawn the robot in Gazebo, start RViz with the moveit 
plugin and inizialize the moveit setup with the launchfile:
``` bash
ros2 launch tb3_nav_pick_and_place complete_scenario.launch.py
```

In a second terminal, launch the navigation stack with the official 
turtlebot3 manipulation navigation package and the custom map and parameter
file:
``` bash
ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py map_yaml_file:=/path/to/workspace/src/tb3_nav_pick_and_place/navigation/maps/spacious_map.yaml params_file:=/path/to/workspace/src/tb3_nav_pick_and_place/navigation/params/turtlebot3.yaml 
```
Then provide the 2D pose estimate for the robot in RViz.

In a third terminal, first run the node to position the robot and then
the node to perform the pick and place operation:
``` bash
ros2 run tb3_nav_pick_and_place navigate_through_poses
ros2 run tb3_nav_pick_and_place static_pick_and_place
```
**Note**: the navigate_through_poses node has an anachronistic name, as it merely requests navigate_to_pose twice; 
it was necessary to get close enough to the tables without risking for the robot to go all the way around them.

It is also worth noting that the pick and place is composed of multiple sequential requests, and that it may be 
necessary to restart the process in case one of them fails.

Here is a video that shows the entire execution:

[![Watch the video](https://img.youtube.com/vi/W7ODtW2BR80/hqdefault.jpg)](https://youtu.be/W7ODtW2BR80?si=IhLpHSls9wEBhn8q)

The entire pick and place setup assumes the correct positioning of the robot: in case the navigation doesn't allow the robot to reach the correct pose (as you may notice, it often stops just shy of the right position), an extra node has been added, to request
specific position goals. Here is the direct translation of the navigate through poses node, for reference:
``` bash
ros2 run tb3_nav_pick_and_place navigate_to_pose --ros-args -p x:=3.8 -p y:=0.6
ros2 run tb3_nav_pick_and_place navigate_to_pose --ros-args -p x:=4.05 -p y:=0.6
```
