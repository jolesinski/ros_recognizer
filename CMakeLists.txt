cmake_minimum_required(VERSION 3.2)
project(ros_recognizer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin COMPONENTS
  dynamic_reconfigure
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  REQUIRED
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PCL 1.8 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#find_package(V4R REQUIRED)
MESSAGE( STATUS "PCL_INCLUDE_HEADERS: " ${PCL_INCLUDE_DIRS} )

generate_dynamic_reconfigure_options(
  cfg/local_3d_describer.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ros_recognizer_preprocessing
  CATKIN_DEPENDS
    dynamic_reconfigure
    nodelet
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
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

catkin_add_gtest(test_preprocessing
    test/test_preprocessing.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)
if(TARGET test_preprocessing)
  target_link_libraries(test_preprocessing
      ros_recognizer_preprocessing
      ${PCL_LIBRARIES}
      ${catkin_LIBRARIES}
  )
endif()