cmake_minimum_required(VERSION 3.15)
project(tsdf LANGUAGES CXX)

list(APPEND CMAKE_CXX_FLAGS "-march=native -ltbb")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

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

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.84 REQUIRED)
# list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
# find_package(Threads REQUIRED)
# find_package(VDBFusion REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX)
add_subdirectory("ext/highfive")



add_executable(tsdf_node "src/tsdf_node.cpp")
target_include_directories(tsdf_node PRIVATE 
    "include/"
    "ext/phmap"
    "ext/morton-nd/include"
    "${HDF5_CXX_INCLUDE_DIRS}"
    "${catkin_INCLUDE_DIRS}")
target_link_libraries(tsdf_node PRIVATE
    Eigen3::Eigen
    Boost::boost
    # VDBFusion::vdbfusion
    HighFive::Include
    ${HDF5_CXX_LIBRARIES}
    ${catkin_LIBRARIES})
target_compile_definitions(tsdf_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)

install(TARGETS tsdf_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# install(DIRECTORY cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})