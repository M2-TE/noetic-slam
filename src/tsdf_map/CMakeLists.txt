cmake_minimum_required(VERSION 3.15)
project(tsdf_map LANGUAGES CXX)

list(APPEND CMAKE_CXX_FLAGS "-march=native -ltbb")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# catkin trashbin
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
find_package(Boost 1.71 REQUIRED)
# list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
# find_package(Threads REQUIRED)
# find_package(VDBFusion REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
# add_subdirectory("ext/highfive/")
# add_subdirectory("ext/lvr2/ext/HighFive/")
find_package(OpenCV REQUIRED)
add_subdirectory("ext/lvr2/")
message("${OpenCV_INCLUDE_DIRS}")


add_executable(tsdf_map_node 
    "src/tsdf_map_node.cpp"
    "src/dag_lvr2.cpp"
    "src/dag.cpp")
target_include_directories(tsdf_map_node PRIVATE 
    "include/"
    "ext/phmap/"
    "ext/lvr2/include/"
    "ext/morton-nd/include/"
    "ext/lvr2/ext/HighFive/include/"
    "${HDF5_CXX_INCLUDE_DIRS}"
    "${OpenCV_INCLUDE_DIRS}"
    "${catkin_INCLUDE_DIRS}")
target_link_libraries(tsdf_map_node PRIVATE
    Eigen3::Eigen
    Boost::boost
    # VDBFusion::vdbfusion
    # HighFive::Include
    lvr2_static
    lvr2las_static
    lvr2rply_static
    ${LVR2_LIB_DEPENDENCIES}
    ${HDF5_CXX_LIBRARIES}
    ${catkin_LIBRARIES})
target_compile_definitions(tsdf_map_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)

install(TARGETS tsdf_map_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# install(DIRECTORY cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})