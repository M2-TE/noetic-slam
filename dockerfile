# FROM m2te/noetic-slam-base:latest
FROM ros:noetic-perception
# set env var during docker build only
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y build-essential cmake git ccache

# Use gcc-11 instead of standard gcc-9
RUN apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get install -y gcc-11 g++-11
ENV CXX "/usr/bin/g++-11"
ENV CC "/usr/bin/gcc-11"

## Dependencies
# Core:
RUN apt-get install -y python3-catkin-tools ros-noetic-pcl-ros
# Ouster:
RUN apt-get install -y ros-noetic-rviz libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev
# DLIO:
RUN apt-get install -y libomp-dev libpcl-dev
# LVR2:
RUN apt-get install -y ninja-build build-essential cmake cmake-curses-gui libflann-dev libgsl-dev libeigen3-dev
RUN apt-get install -y libopenmpi-dev openmpi-bin opencl-c-headers ocl-icd-opencl-dev libcgal-dev doxygen
RUN apt-get install -y libvtk7-dev libvtk7-qt-dev libboost-all-dev freeglut3-dev libhdf5-dev qtbase5-dev 
RUN apt-get install -y qt5-default libqt5opengl5-dev liblz4-dev libopencv-dev libyaml-cpp-dev libspdlog-dev
# Voxblox:
RUN apt-get install -y python3-wstool python3-catkin-tools ros-noetic-cmake-modules protobuf-compiler autoconf
RUN git clone https://github.com/catkin/catkin_simple.git
RUN cd catkin_simple && bash -c ". /opt/ros/noetic/setup.bash && cmake -B build && cmake --build build && cmake --install build"

WORKDIR /root/repo/
ENTRYPOINT [ "/bin/bash", "/root/repo/scripts/.entrypoint.sh" ]
ENV ROSCONSOLE_FORMAT='[ROS${severity}]: ${message}'
ENV ROS_DISTRO=noetic
ENV LIDAR_ADDR=192.168.168.128
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu
ENV RVIZ_OUSTER=false
ENV RVIZ_DLIO=false
