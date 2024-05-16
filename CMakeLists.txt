cmake_minimum_required(VERSION 3.0.2)
project(evs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbag
  roscpp
  rospy
  std_msgs
)

find_package(VISP REQUIRED)

IF(VISP_FOUND)
 include(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
        ${rosbag_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_reference_trajectory src/reference_trajectory.cpp ${VISP_SOURCES})

add_dependencies(${PROJECT_NAME}_reference_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_reference_trajectory
   ${catkin_LIBRARIES}
        ${rosbag_LIBRARIES}
 )
