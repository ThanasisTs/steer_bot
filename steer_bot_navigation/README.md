# steer_bot_navigation

A ROS package for navigating the steer_bot robot using the `move_base` package

### Installation
Follow the instructions [here](https://github.com/ThanasisTs/steer_bot/blob/master/README.md).

### Run
To launch the model with a predetermined map and a predifined RViz configuration run
	`roslaunch steer_bot_gazebo steer_bot_sim_obstacles.launch`

<b>Note</b>: By default both Gazebo and RViz gui are launched. If you want to launch only the RViz gui, set the gui arguement to false in the bash command (`gui:=false`)

To launch the `move_base` node and to load the map run
	`roslaunch steer_bot_navigation move_base.launch`

In Rviz, set a goal using the `2D Nav goal` plugin. If the global planner was able to compute a path, a green line should appear on the map indicating the path. Currently the local planner does not work properly, meaning that the robot will not move.