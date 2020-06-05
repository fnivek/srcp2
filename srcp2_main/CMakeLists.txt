cmake_minimum_required(VERSION 2.8.3)
project(srcp2_main)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  srcp2_main_msgs
  tf2_ros
  tf2_eigen
  geometry_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  laser_geometry
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
  INCLUDE_DIRS include
#  LIBRARIES srcp2_main
  CATKIN_DEPENDS
    roscpp
    rospy
    srcp2_main_msgs
    tf2_ros
    tf2_eigen
    geometry_msgs
    pcl_conversions
    pcl_ros
    sensor_msgs
    laser_geometry
    visualization_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # TODO(Kevin) make relative
  /home/user/catkin_ws/src/srcp2/submodules/libigl/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/srcp2_main.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Quals 1
add_executable(${PROJECT_NAME}_quals_1 src/nodes/quals_1.cpp src/odometry.cpp)
set_target_properties(${PROJECT_NAME}_quals_1 PROPERTIES OUTPUT_NAME quals_1 PREFIX "")
add_dependencies(${PROJECT_NAME}_quals_1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_quals_1 ${catkin_LIBRARIES})

# Point to mesh test script
add_executable(${PROJECT_NAME}_point_to_mesh scripts/point_to_mesh.cpp)
set_target_properties(${PROJECT_NAME}_point_to_mesh PROPERTIES OUTPUT_NAME point_to_mesh PREFIX "")
add_dependencies(${PROJECT_NAME}_point_to_mesh ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_point_to_mesh ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_srcp2_main.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
