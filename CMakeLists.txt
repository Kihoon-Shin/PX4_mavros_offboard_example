cmake_minimum_required(VERSION 3.0.2)
project(mavros_offboard_control_example)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_offb_node src/offb_node.cpp)
target_link_libraries(${PROJECT_NAME}_offb_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_offb_node_traj src/offb_node_traj.cpp)
target_link_libraries(${PROJECT_NAME}_offb_node_traj ${catkin_LIBRARIES})