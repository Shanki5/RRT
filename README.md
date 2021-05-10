# RRT
A ROS package to visualize RRT path planning algorithms

## Dependencies:

  ROS Melodic

## Build Instruction:
  
 1. `cd ~/`
  
 2. `git clone https://github.com/Shanki5/RRT.git`
  
 3. `catkin build` or `catkin_make`
  
## Run Instruction:

Open 3 terminals 

Terminal 1: 

`roscore`

Terminal 2: 

`rosrun rviz rviz -d ~/RRT/src/rrt/launch/rviz_config.rviz`

Terminal 3: 

1. `cd ~/RRT`

2. `source devel/setup.bash`

3. `rosrun rrt rrt_planning _obstacleFile:=~/RRT_ROS_WS/src/rrt/src/obstacles.txt `
