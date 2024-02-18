cmake_minimum_required(VERSION 3.0.2)
project(lqr_track)
set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  cmake_modules
  tf
  tf2
)

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES LQR_track
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


#add_library(LQR_LIBRARIES
#${PROJECT_SOURCE_DIR}/include/LQR_track/Tool.h
#${PROJECT_SOURCE_DIR}/include/LQR_track/trajectory.h
#${PROJECT_SOURCE_DIR}/include/LQR_track/LQR.h
#${PROJECT_SOURCE_DIR}/src/Tool.cpp
#${PROJECT_SOURCE_DIR}/src/trajectory.cpp
#${PROJECT_SOURCE_DIR}/src/LQR.cpp
#)



add_executable(${PROJECT_NAME} src/LQR_node_world.cpp src/Tool.cpp src/LQR.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} LQR_LIBRARIES)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

link_directories(
  ${catkin_LIB_DIRS}
  ${Eigen_LIB_DIRS}
)
