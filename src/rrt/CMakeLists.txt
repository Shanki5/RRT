cmake_minimum_required(VERSION 3.0.2)
project(rrt)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrt
#  CATKIN_DEPENDS roscpp std_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(rrt_planning src/rrt_planning.cpp)
target_link_libraries(rrt_planning ${catkin_LIBRARIES})  

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
