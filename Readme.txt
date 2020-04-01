# group6_rwa4

(a) Build Instructions

1. copy group6_rwa4.zip to "catkin_ws/src"
2. Run following command in the terminal:

 cd ~/catkin_ws/src
 unzip group6_rwa4.zip
 cd ..
 catkin_make --only-pkg-with-deps group6_rwa4


(b) Run Instructions

1. Open terminal
2. Type following commands in the terminal

 cd ~/catkin_ws
 source devel/setup.bash

3. Run following commands in new terminals:
 
 roslaunch group6_rwa4 group6_rwa4.launch
 roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
 rosrun group6_rwa4 main_node

