# RRT
A ROS package to visualize RRT path planning algorithms

Dependencies:

  ROS Melodic

Build Instruction:

  Clone the repository
  catkin_make or catkin build from the root of workspace
  
Run Instruction:

Open 3 terminals 

Terminal 1: roscore

Terminal 2: rosrun rviz rviz -d ~/RRT/src/rrt/launch/rviz_config.rviz

Terminal 3: rosrun rrt rrt_planning _obstacleFile:=/home/shankrith/RRT_ROS_WS/src/rrt/src/obstacles.txt 
