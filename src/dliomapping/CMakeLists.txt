cmake_minimum_required(VERSION 3.0)
project(dliomapping LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
list(APPEND CATKIN_PACKAGES roscpp)
list(APPEND CATKIN_PACKAGES sensor_msgs)
list(APPEND CATKIN_PACKAGES geometry_msgs)
list(APPEND CATKIN_PACKAGES nav_msgs)
list(APPEND CATKIN_PACKAGES pcl_ros)
list(APPEND CATKIN_PACKAGES pcl_conversions)

## Find catkin and catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})
catkin_package(CATKIN_DEPENDS ${CATKIN_PACKAGES})

add_executable(dliomapping_node "dliomapping.cpp")
target_compile_options(dliomapping_node PRIVATE -std=c++17)
target_link_libraries(dliomapping_node ${catkin_LIBRARIES})
target_include_directories(dliomapping_node PRIVATE ${catkin_INCLUDE_DIRS})

install(TARGETS dliomapping_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )