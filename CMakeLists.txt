cmake_minimum_required(VERSION 2.8.3)
project(pepper_image_sink)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs cv_bridge)

find_package(OpenCV)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

catkin_package(
  LIBRARIES pepper_image_sink
  CATKIN_DEPENDS nodelet roscpp std_msgs
)

## Create the nodelet tutorial library
add_library(pepper_image_sink src/PepperImageSink.cpp)

target_link_libraries(pepper_image_sink ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(pepper_image_sink ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS pepper_image_sink
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES pepper_image_sink.xml launch/pepper_image_sink_offload.launch launch/pointcloud_offload.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
