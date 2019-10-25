cmake_minimum_required(VERSION 2.8.3)
project(cam_lidar_calibration)
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS nodelet
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  image_transport
  cv_bridge
  sensor_msgs
  message_generation
  cmake_modules
  dynamic_reconfigure
  )

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
## Setup include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )
add_message_files(
  FILES
  calibration_data.msg
  extrinsics.msg
  )
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  )
generate_dynamic_reconfigure_options(
  cfg/bounds.cfg
  )
catkin_package(
  CATKIN_DEPENDS nodelet
  roscpp
  std_msgs
  sensor_msgs
  message_runtime
  )

add_executable(input_sample src/input_sample.cpp)
target_link_libraries(input_sample ${catkin_LIBRARIES})

add_executable(optimizer src/optimizer.cpp)
target_link_libraries(optimizer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBRARIES})
add_dependencies(optimizer cam_lidar_calibration_generate_messages_cpp)

## Create the nodelet tutorial library
add_executable(feature_extraction src/feature_extractor.cpp)
add_dependencies(feature_extraction ${PROJECT_NAME}_gencfg)

target_link_libraries(feature_extraction ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(feature_extraction ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS feature_extraction optimizer input_sample
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_definition.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  )

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  )
