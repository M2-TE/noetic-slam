cmake_minimum_required(VERSION 3.0.0)
project(tsdf LANGUAGES CXX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    pcl_conversions)

## Declare a catkin package
catkin_package()

add_executable(tsdf "src/tsdf.cpp")
target_link_libraries(tsdf ${catkin_LIBRARIES})
target_include_directories(tsdf PRIVATE ${catkin_INCLUDE_DIRS})