cmake_minimum_required(VERSION 3.0.0)
project(tsdf LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

set(CATKIN_PACKAGES 
    roscpp
    sensor_msgs geometry_msgs nav_msgs
    pcl_ros pcl_conversions
)

## Find catkin and catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})
catkin_package(CATKIN_DEPENDS ${CATKIN_PACKAGES})

add_executable(tsdf_node "src/tsdf_node.cpp")
target_link_libraries(tsdf_node ${catkin_LIBRARIES})
target_include_directories(tsdf_node 
    PRIVATE ${catkin_INCLUDE_DIRS}
    PRIVATE include
    PRIVATE ext/phmap)
install(TARGETS tsdf_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# install(DIRECTORY cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})