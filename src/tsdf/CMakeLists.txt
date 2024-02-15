cmake_minimum_required(VERSION 3.15)
project(tsdf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

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

## Find Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
## Find Boost
find_package(Boost 1.84 REQUIRED)

add_executable(tsdf_node "src/tsdf_node.cpp")
target_compile_options(tsdf_node PRIVATE -std=c++2a)
target_link_libraries(tsdf_node ${catkin_LIBRARIES})
target_link_libraries(tsdf_node Eigen3::Eigen Boost::headers)
target_include_directories(tsdf_node PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(tsdf_node PRIVATE include)
target_include_directories(tsdf_node PRIVATE ext/phmap)
target_include_directories(tsdf_node PRIVATE ext/libmorton/include)

install(TARGETS tsdf_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# install(DIRECTORY cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})