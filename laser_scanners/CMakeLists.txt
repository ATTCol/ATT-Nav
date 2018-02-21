cmake_minimum_required(VERSION 2.8.3)
project(laser_scanners)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  lms1xx
)

include_directories(${catkin_INCLUDE_DIRS} ~/ws_att/src/LMS1xx/include)
add_executable(laser_scanner_node src/laser_scanner_node.cpp)
target_link_libraries(laser_scanner_node ${catkin_LIBRARIES} ~/ws_att/devel/lib/libLMS1xx.so)
