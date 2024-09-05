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

# system dependencies TODO: put these into the proj deps that need em
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(tsdf_map_node PRIVATE Eigen3::Eigen)
find_package(Boost 1.71 REQUIRED)
target_link_libraries(tsdf_map_node PRIVATE Boost::boost)
target_compile_definitions(tsdf_map_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
target_include_directories(tsdf_map_node PRIVATE SYSTEM "${HDF5_CXX_INCLUDE_DIRS}")
target_link_libraries(tsdf_map_node PRIVATE ${HDF5_CXX_LIBRARIES})

## fetch project dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED OFF)
# tsdf mapping backend
set(CHAD_POPCOUNT_INSTRUCTION __builtin_popcount)
set(CHAD_LEAF_RESOLUTION 0.1)
set(CHAD_LEAF_BITS 8)
FetchContent_Declare(chad_tsdf GIT_REPOSITORY "https://github.com/M2-TE/chad_tsdf.git" GIT_TAG "main" GIT_SHALLOW ON)
FetchContent_MakeAvailable(chad_tsdf)
target_link_libraries(tsdf_map_node PRIVATE chad_tsdf)
# parallel hashmap
FetchContent_Declare(phmap GIT_REPOSITORY "https://github.com/greg7mdp/parallel-hashmap.git" GIT_TAG "v1.3.12" GIT_SHALLOW ON)
FetchContent_MakeAvailable(phmap)
target_link_libraries(tsdf_map_node PRIVATE phmap)
# morton-nd
FetchContent_Declare(morton-nd GIT_REPOSITORY "https://github.com/morton-nd/morton-nd.git" GIT_TAG "v4.0.0" GIT_SHALLOW ON)
FetchContent_MakeAvailable(morton-nd)
target_link_libraries(tsdf_map_node PRIVATE morton-nd::MortonND)
# lvr2
FetchContent_Declare(lvr2 GIT_REPOSITORY "https://gitlab.informatik.uni-osnabrueck.de/Las_Vegas_Reconstruction/Develop.git" GIT_TAG "master" GIT_SHALLOW ON)
FetchContent_MakeAvailable(lvr2)
target_include_directories(tsdf_map_node PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/include/" "${lvr2_SOURCE_DIR}/ext/HighFive/include/")
target_link_libraries(tsdf_map_node PRIVATE lvr2_static ${LVR2_LIB_DEPENDENCIES})
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