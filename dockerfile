FROM ros:noetic-perception

# set env var during docker build only
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y build-essential cmake git ccache

# Use gcc-11 instead of standard gcc-9
RUN apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get update && apt-get install -y gcc-11 g++-11
ENV CXX "/usr/bin/g++-11"
ENV CC "/usr/bin/gcc-11"

## Dependencies
# Core:
RUN apt-get update && apt-get install -y python3-catkin-tools ros-noetic-pcl-ros gdb pip
# Ouster:
RUN apt-get update && apt-get install -y ros-noetic-rviz libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev
# DLIO:
RUN apt-get update && apt-get install -y libomp-dev libpcl-dev
# rosbag conversion from ros1 to ros2 or vice versa
RUN pip install rosbags

WORKDIR /root/repo/
ENTRYPOINT [ "/bin/bash", "/root/repo/scripts/.entrypoint.sh" ]
ENV ROSCONSOLE_FORMAT='[ROS${severity}]: ${message}'
ENV ROS_DISTRO=noetic
ENV LIDAR_ADDR=192.168.168.128
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu
ENV RVIZ_OUSTER=false
ENV RVIZ_DLIO=false
