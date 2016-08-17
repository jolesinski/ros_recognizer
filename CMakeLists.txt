cmake_minimum_required(VERSION 3.2)
project(ros_recognizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
#find_package(V4R REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/local_3d_describer.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ros_recognizer_preprocessing
  CATKIN_DEPENDS dynamic_reconfigure nodelet pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(ros_recognizer_preprocessing
  src/preprocessing/local_3d_describer.cpp
)
target_link_libraries(ros_recognizer_preprocessing
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(ros_recognizer_preprocessing ${PROJECT_NAME}_gencfg)

# add_executable(ros_recognizer_node src/ros_recognizer_node.cpp)
# add_dependencies(ros_recognizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ros_recognizer_node
#   ${catkin_LIBRARIES}
# )

# install(TARGETS ros_recognizer ros_recognizer_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_recognizer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
