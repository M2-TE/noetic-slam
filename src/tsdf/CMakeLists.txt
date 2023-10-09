cmake_minimum_required(VERSION 3.0)
project(tsdf LANGUAGES CXX)

list(APPEND CATKIN_PACKAGES roscpp)
list(APPEND CATKIN_PACKAGES sensor_msgs)
list(APPEND CATKIN_PACKAGES geometry_msgs)
list(APPEND CATKIN_PACKAGES nav_msgs)
list(APPEND CATKIN_PACKAGES pcl_ros)
list(APPEND CATKIN_PACKAGES pcl_conversions)

## Find catkin and catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})
catkin_package(CATKIN_DEPENDS ${CATKIN_PACKAGES})

## Find Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(tsdf_node "src/tsdf_node.cpp")
target_compile_options(tsdf_node PRIVATE -std=c++17)
target_link_libraries(tsdf_node ${catkin_LIBRARIES})
target_link_libraries(tsdf_node Eigen3::Eigen)
target_include_directories(tsdf_node PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(tsdf_node PRIVATE include)
target_include_directories(tsdf_node PRIVATE ext/phmap)

install(TARGETS tsdf_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# install(DIRECTORY cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})