cmake_minimum_required(VERSION 3.2)
project(ros_recognizer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fopenmp")

find_package(catkin COMPONENTS
  dynamic_reconfigure
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  eigen_conversions
  geometry_msgs
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
  cfg/local_matcher.cfg
  cfg/verifier.cfg
  cfg/visualizer.cfg
)

add_message_files(
  DIRECTORY
    msg
  FILES
    CorrespondenceClusters.msg
    CorrespondenceArray.msg
    Correspondence.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    set_model_from_cloud.srv
    set_model_from_pcd.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ros_recognizer_description
  CATKIN_DEPENDS
    dynamic_reconfigure
    nodelet
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    eigen_conversions
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(ros_recognizer_description
  src/description/local_3d_describer.cpp
)
target_link_libraries(ros_recognizer_description
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(ros_recognizer_description ${PROJECT_NAME}_gencfg)

add_library(ros_recognizer_matching
    src/matching/local_matcher.cpp
)
target_link_libraries(ros_recognizer_matching
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(ros_recognizer_matching ${PROJECT_NAME}_gencfg)

add_library(ros_recognizer_verification
    src/verification/verifier.cpp
)
target_link_libraries(ros_recognizer_verification
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(ros_recognizer_verification ${PROJECT_NAME}_gencfg)

add_library(ros_recognizer_visualization
    src/visualization/visualizer.cpp
)
target_link_libraries(ros_recognizer_visualization
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(ros_recognizer_visualization ${PROJECT_NAME}_gencfg)

add_library(ros_recognizer_nodelets
    src/nodelet/recognizer.cpp
    src/nodelet/viewer.cpp
)
target_link_libraries(ros_recognizer_nodelets
    ros_recognizer_description
    ros_recognizer_matching
    ros_recognizer_verification
    ros_recognizer_visualization
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

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

catkin_add_gtest(test_recognition
    test/test_local_3d_pipeline.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)
if(TARGET test_recognition)
  target_link_libraries(test_recognition
      ros_recognizer_description
      ros_recognizer_matching
      ros_recognizer_verification
      ${PCL_LIBRARIES}
      ${catkin_LIBRARIES}
)
endif()