cmake_minimum_required(VERSION 2.8.3)
project(tsdf-backend-project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs)

## Declare a catkin package
catkin_package()

add_executable(tsdf-backend "src/tsdf-backend.cpp")
target_link_libraries(tsdf-backend ${catkin_LIBRARIES})