cmake_minimum_required(VERSION 2.8.3)
project(mpepc_local_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  nav_core
  navfn
  nav_msgs
  roscpp
  roslib
  tf
  mpepc_global_planner
  )
  
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(
  cfg/AckermannPlanner.cfg
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EgoGoal.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(control_law 
	src/control_law.cpp
)

add_library(mpepc_local_planner
  src/mpepc_planner_ros.cpp
  )
  
add_dependencies(mpepc_local_planner
  ${mpepc_local_planner_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpepc_local_planner 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  control_law
  nlopt
  )

install(TARGETS mpepc_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(FILES blp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
