cmake_minimum_required(VERSION 2.8.3)
project(cam_lidar_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_generation
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rviz
  sensor_msgs
  std_msgs
  )

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)

## Setup include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )
#link_directories(${catkin_LIBRARY_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_AUTOMOC ON)

add_message_files(
  FILES
  calibration_data.msg
  extrinsics.msg
  )
add_service_files(
  FILES
  Sample.srv
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

add_executable(optimizer src/optimizer.cpp)
target_link_libraries(optimizer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBRARIES})

add_executable(feature_extraction
  src/feature_extractor.cpp
  )

target_link_libraries(feature_extraction ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Eigen_LIBRARIES})

add_library(cam_lidar_calibration
  src/cam_lidar_panel.cpp)
target_link_libraries(cam_lidar_calibration Qt5::Widgets ${catkin_LIBRARIES})


## Mark the nodelet library for installations
install(TARGETS feature_extraction optimizer cam_lidar_calibration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_definition.xml plugin_description.xml
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
