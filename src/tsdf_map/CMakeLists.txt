cmake_minimum_required(VERSION 3.15)
project(tsdf_map LANGUAGES CXX)

list(APPEND CMAKE_CXX_FLAGS "-march=native -ltbb")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER_LAUNCHER ccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

# main executable
add_executable(tsdf_map_node "src/tsdf_map_node.cpp")
target_include_directories(tsdf_map_node PRIVATE "include/")
target_compile_definitions(tsdf_map_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)

# catkin trashbin
list(APPEND CATKIN_PACKAGES roscpp)
list(APPEND CATKIN_PACKAGES sensor_msgs)
list(APPEND CATKIN_PACKAGES geometry_msgs)
list(APPEND CATKIN_PACKAGES nav_msgs)
list(APPEND CATKIN_PACKAGES pcl_ros)
list(APPEND CATKIN_PACKAGES pcl_conversions)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})
catkin_package(CATKIN_DEPENDS ${CATKIN_PACKAGES})
target_include_directories(tsdf_map_node PRIVATE SYSTEM "${catkin_INCLUDE_DIRS}")
target_link_libraries(tsdf_map_node PRIVATE ${catkin_LIBRARIES})

# system dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.71 REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
target_include_directories(tsdf_map_node PRIVATE SYSTEM
    "${HDF5_CXX_INCLUDE_DIRS}")
target_link_libraries(tsdf_map_node PRIVATE
    Eigen3::Eigen
    Boost::boost
    ${HDF5_CXX_LIBRARIES})

# git submodule dependencies
add_subdirectory("ext/lvr2/")
target_include_directories(tsdf_map_node PRIVATE SYSTEM
    "ext/phmap/"
    "ext/lvr2/include/"
    "ext/mortonnd/include/"
    "ext/lvr2/ext/HighFive/include/")
target_link_libraries(tsdf_map_node PRIVATE
    lvr2_static
    ${LVR2_LIB_DEPENDENCIES})

## fetch project dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED OFF)
# tsdf mapping backend
set(CHAD_POPCOUNT_INSTRUCTION __builtin_popcount)
set(CHAD_LEAF_RESOLUTION 0.1)
set(CHAD_LEAF_BITS 4)
FetchContent_Declare(chad_tsdf GIT_REPOSITORY "https://github.com/M2-TE/chad_tsdf.git" GIT_TAG "main" GIT_SHALLOW ON)
FetchContent_MakeAvailable(chad_tsdf)
target_link_libraries(tsdf_map_node PRIVATE chad_tsdf)
# octomap
# set(BUILD_OCTOVIS_SUBPROJECT OFF)
# set(BUILD_DYNAMICETD3D_SUBPROJECT OFF)
# FetchContent_Declare(octomap GIT_REPOSITORY "https://github.com/OctoMap/octomap.git" GIT_TAG "devel" GIT_SHALLOW ON)
# FetchContent_MakeAvailable(octomap)
# vdbfusion
# TODO
# voxblox
# TODO

install(TARGETS tsdf_map_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# install(DIRECTORY cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})