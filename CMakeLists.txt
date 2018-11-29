cmake_minimum_required(VERSION 2.8.3)
project(struct_core_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  geometry_msgs
  rosbag
  nodelet
  cv_bridge
  image_transport
)

set(CMAKE_CXX_FLAGS "-std=c++0x -pthread -fPIC ${CMAKE_CXX_FLAGS} -D__STDC_CONSTANT_MACROS")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rosconsole
  geometry_msgs
  rosbag
  nodelet
  cv_bridge
  image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  /usr/local/lib/libStructureLinux.so)

add_executable(sc src/sc.cpp)
target_link_libraries(sc ${LINK_LIBRARIES} StructureLinux z rt m usb-1.0)
add_dependencies(sc ${catkin_EXPORTED_TARGETS})


