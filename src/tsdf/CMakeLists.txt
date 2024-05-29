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

## Find Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
## Find Boost
find_package(Boost 1.84 REQUIRED)
## Find VDBFusion
# list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
# find_package(Threads REQUIRED)
# find_package(VDBFusion REQUIRED)
# # Matplot++
# include(FetchContent)
# set(MATPLOTPP_BUILD_EXAMPLES OFF)
# set(MATPLOTPP_BUILD_TESTS OFF)
# FetchContent_Declare(matplot GIT_REPOSITORY "https://github.com/alandefreitas/matplotplusplus.git" GIT_TAG "master" GIT_SHALLOW ON)
# FetchContent_MakeAvailable(matplot)


add_executable(tsdf_node "src/tsdf_node.cpp")
target_link_libraries(tsdf_node PRIVATE ${catkin_LIBRARIES})
target_link_libraries(tsdf_node PRIVATE Eigen3::Eigen)
# target_link_libraries(tsdf_node PRIVATE Boost::headers)
# target_link_libraries(tsdf_node PRIVATE VDBFusion::vdbfusion)
# target_link_libraries(tsdf_node PRIVATE matplot)
target_include_directories(tsdf_node PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(tsdf_node PRIVATE include)
target_include_directories(tsdf_node PRIVATE ext/phmap)
target_include_directories(tsdf_node PRIVATE ext/morton-nd/include)
target_compile_definitions(tsdf_node PRIVATE BOOST_BIND_GLOBAL_PLACEHOLDERS)

install(TARGETS tsdf_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
# install(DIRECTORY cfg launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})