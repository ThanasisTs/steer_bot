# Steer Bot

## Installation

```bash
# Create a workspace folder
mkdir -p <catkin_ws>/src

# Clone the repo
cd <catkin_ws>/src
git clone https://github.com/ThanasisTs/steer_bot

# Checkout a version of `steer_drive_ros` patched for ROS Melodic
git clone https://github.com/tsedl/steer_drive_ros.git
cd steer_drive_ros
git checkout melodic-devel
cd ../../

# Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro <ROS-DISTRO> -y

# Build
cd <catkin_ws>/src
catkin build
source devel/setup.bash
```

## Run

Start the Gazebo simulation:

Launch the simulation environment
```bash
roslaunch steer_bot_gazebo steer_bot_sim_obstacles.launch
```

Important Arguements:
* gui: true to launch the Gazebo GUI (false otherwise, default to true)
* teleop: true to enable teleoperation (false when used for autonomous navigation, default to false)

Launch the `move_base` file for autonomous navigation!

```bash
roslaunch steer_bot_navigation move_base.launch
```

Important Arguements:
* map_file: path to the map
* motion_premitives_file: path to the motion premitives file
**Note**: The map should be of the same resolution as the motion premitives files. Otherwise the SBPL planner won't run.

Example:

Terminal 1:
`roslaunch steer_bot_gazebo steer_bot_sim_obstacles.launch gui:=false`

Terminal 2:
`roslaunch steer_bot_navigation move_base.launch map_file:=/home/thanasis/catkin_ws/src/steer_bot/steer_bot_navigation/maps/tight_maps/map_res_0.01.yaml motion_premitives_file:=/home/thanasis/catkin_ws/src/steer_bot/steer_bot_navigation/motion_premitives/map_res_0.01.mprim`

If everything runs smoothly, you should see an image similar to the following in RViz.
![alt text](https://github.com/ThanasisTs/steer_bot/blob/master/steer_bot_navigation/rviz.png)

To give a goal pose, select the `2D Nav Goal` and set a goal in the map.
![alt text](https://github.com/ThanasisTs/steer_bot/blob/master/steer_bot_navigation/goal.png)

Αfter that, you should see the generated path as a green line.
![alt text](https://github.com/ThanasisTs/steer_bot/blob/master/steer_bot_navigation/path.png)

**Note**: In your PC, change the `/home/thanasis/...` path in the map_file and motion_premitives_file arguements to the absolute path of the map and the motion premitives in your PC.





