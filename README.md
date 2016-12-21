# turtlebot_mpepc
Reimplement MPEPC from MattDerry using MPEPC technique presented in Park's paper.

This version use https://github.com/MattDerry/model_predictive_navigation as a local planner.

The original package of MattDerry does not use map to make global planner.

This package use the advantage of map and global planner (Dijkstra in this case) to build a potential map, which will be use in Optimization Progess in MPEPC local planner.

The other advantage of this current package is it organized like the original ROS move_base schema.

This package has tested on 2 popular simulator in ROS: Stage and Gazebo, with and without apply localization.

To Run:
1) Stage:
```
roslaunch turtlebot_mpepc move_base_stage.launch
```
2) Gazebo:
```
roslaunch turtlebot_mpepc move_base_gazebo.launch
```
