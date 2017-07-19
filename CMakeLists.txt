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


add_executable(head_depth_compressor src/head_depth_compressor.cpp)
add_dependencies(head_depth_compressor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(head_depth_compressor
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES pepper_image_sink.xml launch/pepper_image_sink.launch launch/pointcloud_offload.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  install(TARGETS head_depth_compressor
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
