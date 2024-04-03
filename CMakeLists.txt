cmake_minimum_required(VERSION 3.0.2)
project(DHPGMVS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visp_bridge
  message_filters
  sensor_msgs
  tf
)
find_package(VISP 3 REQUIRED)

find_package(PER REQUIRED per_core per_io per_features per_sensor_pose_estimation per_estimation)
if(PER_FOUND)
  include(${PER_USE_FILE})
endif(PER_FOUND)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES nodes/DHPGMVS.cpp )
add_executable(
        DHPGMVS
        ${SOURCES}
)
target_link_libraries(
        DHPGMVS
        ${catkin_LIBRARIES}
)
