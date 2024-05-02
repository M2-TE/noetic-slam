# FROM m2te/noetic-slam-base:latest

FROM ros:noetic
# set env var during docker build only
ARG DEBIAN_FRONTEND=noninteractive
# INSTALL DEPENDENCIES
RUN apt-get update && apt-get upgrade -y
# common build tools
RUN apt-get install -y build-essential cmake git
# catkin_tools
RUN apt-get install -y python3-catkin-tools catkin-lint
# PCL tools
RUN apt-get install -y ros-noetic-pcl-ros pcl-tools
# Ouster-ros dependencies
RUN apt-get install -y ros-noetic-rviz libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev
# DLIO dependencies
RUN apt-get install -y libomp-dev libpcl-dev
# extra utils
RUN apt-get install -y iputils-ping mesa-utils

# Use gcc-11 instead of standard gcc-9
RUN apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get install -y gcc-11 g++-11
ENV CXX "/usr/bin/g++-11"
ENV CC "/usr/bin/gcc-11"

# Boost 1.84.0
RUN apt-get install -y wget
RUN wget https://boostorg.jfrog.io/artifactory/main/release/1.84.0/source/boost_1_84_0.tar.gz
RUN tar -xf boost_1_84_0.tar.gz
RUN cd boost_1_84_0 && ./bootstrap.sh && ./b2 install
RUN rm -r /boost_1_84_0 /boost_1_84_0.tar.gz

# OpenVDB
RUN apt-get install --no-install-recommends -y libblosc-dev libboost-iostreams-dev libboost-system-dev libboost-system-dev 
RUN apt-get install -y libjemalloc-dev libtbb-dev
RUN git clone --depth 1 https://github.com/nachovizzo/openvdb.git -b nacho/vdbfusion
RUN cd openvdb && mkdir build && cd build && cmake  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DUSE_ZLIB=OFF .. &&  sudo make -j$(nproc) all install
RUN rm -r /openvdb
# VDBfusion
RUN apt-get install -y ros-noetic-tf2-sensor-msgs
RUN git clone --depth 1 https://github.com/PRBonn/vdbfusion.git
RUN cd vdbfusion && mkdir build && cd build && cmake .. &&  sudo make -j$(nproc) all install
RUN rm -r /vdbfusion

WORKDIR /root/repo/
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/repo/devel/setup.bash' >> /root/.bashrc
RUN echo 'PATH=$PATH:/root/repo/scripts' >> /root/.bashrc
ENTRYPOINT [ "/bin/bash", "/root/repo/scripts/.entrypoint.sh" ]
ENV ROSCONSOLE_FORMAT='[ROS${severity}]: ${message}'
ENV NOETICSLAM_DOCKER=1

## configurable env vars
ENV AUTOSTART=false
ENV LIDAR_ADDR=192.168.168.128
ENV FILENAME=hsfd_nov2023_testing

# Topics (Pointcloud and IMU)
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu

# RVIZ
ENV RVIZ_OUSTER=false
ENV RVIZ_DLIO=false
