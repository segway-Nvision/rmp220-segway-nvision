cmake_minimum_required(VERSION 2.8.3)
project(rmp_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rmp_msgs
  tf
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_python_setup()

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pkg_name
 CATKIN_DEPENDS rmp_msgs roscpp rospy tf geometry_msgs nav_msgs sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(pkg_name
#   src/${PROJECT_NAME}/pkg_name.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(pkg_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(pkg_name_node src/pkg_name_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(pkg_name_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(pkg_name_node
#   ${catkin_LIBRARIES}
# )

