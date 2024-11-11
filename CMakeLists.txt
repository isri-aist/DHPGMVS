cmake_minimum_required(VERSION 3.0.2)
project(dhpgmvs)

find_package(DifferentiableImage REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visp_bridge
  message_filters
  sensor_msgs
  tf
  rosbag
  message_generation
  dynamic_reconfigure
  image_transport
  sensor_msgs
)
find_package(VISP 3 REQUIRED)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

find_package(PER REQUIRED per_core per_io per_features per_sensor_pose_estimation per_estimation)
if(PER_FOUND)
  include(${PER_USE_FILE})

  ## Generate services in the 'srv' folder
add_service_files(
  FILES
  diff_lambda.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)
  # Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/lambda_g.cfg
)

catkin_package(
  CATKIN_DEPENDS image_transport
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS dynamic_reconfigure
)
endif(PER_FOUND)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# #Service for calculating initial lambda_g 
add_executable(lambdaServiceDiff nodes/lambdaService.cpp)
target_link_libraries(lambdaServiceDiff ${catkin_LIBRARIES} DifferentiableImage)

set(SOURCES nodes/dhpgmvs.cpp )
add_executable(
        dhpgmvs
        ${SOURCES}
)
target_link_libraries(
        dhpgmvs
        ${catkin_LIBRARIES}
        ${rosbag_LIBRARIES}
        ${image_transport_LIBRARIES}
)

#make sure configure headers are built before any node using them
add_dependencies(dhpgmvs ${PROJECT_NAME}_gencfg)

# Add cmake target dependencies of the library
# as an example, code may need to be generated before libraries
# either from message generation or dynamic reconfigure
add_dependencies(dhpgmvs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(dhpgmvs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

