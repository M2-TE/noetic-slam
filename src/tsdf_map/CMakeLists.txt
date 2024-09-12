cmake_minimum_required(VERSION 3.15)
project(tsdf_map LANGUAGES CXX)

list(APPEND CMAKE_C_FLAGS "-march=native")
list(APPEND CMAKE_CXX_FLAGS "-march=native -ltbb")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER_LAUNCHER ccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW) # disallow option() from overwriting set()

# catkin trashbin
list(APPEND CATKIN_PACKAGES roscpp)
list(APPEND CATKIN_PACKAGES sensor_msgs)
list(APPEND CATKIN_PACKAGES geometry_msgs)
list(APPEND CATKIN_PACKAGES nav_msgs)
list(APPEND CATKIN_PACKAGES pcl_ros)
list(APPEND CATKIN_PACKAGES pcl_conversions)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})
catkin_package(CATKIN_DEPENDS ${CATKIN_PACKAGES})

# main executable
add_executable(tsdf_map_node "src/tsdf_map_node.cpp")
target_include_directories(tsdf_map_node PRIVATE "include/")
target_include_directories(tsdf_map_node PRIVATE SYSTEM "${catkin_INCLUDE_DIRS}")
target_link_libraries(tsdf_map_node PRIVATE ${catkin_LIBRARIES})

# system dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
target_link_libraries(tsdf_map_node PRIVATE Eigen3::Eigen)

## fetch project dependencies
include(FetchContent)
set(FETCHCONTENT_QUIET ON)
set(FETCHCONTENT_UPDATES_DISCONNECTED OFF)
# tsdf mapping backend
set(CHAD_POPCOUNT_INSTRUCTION __builtin_popcount)
set(CHAD_NORM_MIN_DOT 0.1)
set(CHAD_NORM_NEIGH_UPPER 50)
set(CHAD_NORM_NEIGH_LOWER 10)
set(CHAD_NORM_MIN_NEIGH 6)
set(CHAD_NORM_RADIUS_MOD 1.0)
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
# fmt
FetchContent_Declare(fmt GIT_REPOSITORY "https://github.com/fmtlib/fmt.git" GIT_TAG "11.0.0" GIT_SHALLOW ON)
FetchContent_MakeAvailable(fmt)
target_link_libraries(tsdf_map_node PRIVATE fmt::fmt)
# lvr2
FetchContent_Declare(lvr2 GIT_REPOSITORY "https://gitlab.informatik.uni-osnabrueck.de/Las_Vegas_Reconstruction/Develop.git" GIT_TAG "master" GIT_SHALLOW ON)
FetchContent_MakeAvailable(lvr2)
target_include_directories(tsdf_map_node PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/include/")
target_include_directories(tsdf_map_node PRIVATE SYSTEM "${lvr2_SOURCE_DIR}/ext/HighFive/include/")
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
target_include_directories(tsdf_map_node PRIVATE SYSTEM "${HDF5_CXX_INCLUDE_DIRS}")
target_link_libraries(tsdf_map_node PRIVATE lvr2_static ${LVR2_LIB_DEPENDENCIES})
# vdbfusion
FetchContent_Declare(vdbfusion GIT_REPOSITORY "https://github.com/PRBonn/vdbfusion.git" GIT_TAG "v0.1.6" GIT_SHALLOW ON)
FetchContent_MakeAvailable(vdbfusion)
target_link_libraries(tsdf_map_node PRIVATE VDBFusion::vdbfusion)
# voxblox
FetchContent_Declare(voxblox GIT_REPOSITORY "https://github.com/M2-TE/voxblox.git" GIT_TAG "master" GIT_SHALLOW ON)
FetchContent_MakeAvailable(voxblox)
add_subdirectory("${voxblox_SOURCE_DIR}/voxblox" "${voxblox_BINARY_DIR}")
target_link_libraries(tsdf_map_node PRIVATE voxblox)
# octomap
set(BUILD_OCTOVIS_SUBPROJECT OFF)
set(BUILD_DYNAMICETD3D_SUBPROJECT OFF)
set(OCTOMAP_OMP ON)
FetchContent_Declare(octomap GIT_REPOSITORY "https://github.com/OctoMap/octomap.git" GIT_TAG "v1.10.0" GIT_SHALLOW ON)
FetchContent_MakeAvailable(octomap)
target_include_directories(tsdf_map_node PRIVATE "${octomap_SOURCE_DIR}/octomap/include/")
target_link_libraries(tsdf_map_node PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomap.a") # warcrime A
target_link_libraries(tsdf_map_node PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomath.a") # warcrime B

install(TARGETS tsdf_map_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# install(DIRECTORY cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})