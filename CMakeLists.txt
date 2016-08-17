cmake_minimum_required(VERSION 2.8.3)
project(ros_recognizer)

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
find_package(V4R REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_recognizer
  CATKIN_DEPENDS dynamic_reconfigure nodelet pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(ros_recognizer
#   src/${PROJECT_NAME}/ros_recognizer.cpp
# )
# add_dependencies(ros_recognizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
