FROM m2te/noetic-slam-base:latest AS init

# FROM ros:noetic AS init
# # set env var during docker build only
# ARG DEBIAN_FRONTEND=noninteractive
# # INSTALL DEPENDENCIES
# RUN apt-get update && apt-get upgrade -y
# # common build tools
# RUN apt-get install -y build-essential cmake git
# # catkin_tools
# RUN apt-get install -y python3-catkin-tools catkin-lint
# # PCL tools
# RUN apt-get install -y ros-noetic-pcl-ros pcl-tools
# # Ouster-ros dependencies
# RUN apt-get install -y ros-noetic-rviz libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev
# # DLIO dependencies
# RUN apt-get install -y libomp-dev libpcl-dev
# # extra utils
# RUN apt-get install -y iputils-ping mesa-utils

FROM init AS setup
WORKDIR /root/repo/
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/repo/devel/setup.bash' >> /root/.bashrc
RUN echo 'PATH=$PATH:/root/repo/scripts' >> /root/.bashrc
ENTRYPOINT [ "/bin/bash", "/root/repo/scripts/.entrypoint.sh" ]
ENV ROSCONSOLE_FORMAT='[ROS${severity}]: ${message}'
ENV NOETICSLAM_DOCKER=1
# ENV QT_X11_NO_MITSHM=1
# --env __NV_PRIME_RENDER_OFFLOAD=1 \
# --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
# --env NVIDIA_VISIBLE_DEVICES=all \
# --env NVIDIA_DRIVER_CAPABILITIES=all \

## configurable env vars
FROM setup AS config
# options: replay, record, stream
ENV MODE=replay
ENV AUTOSTART=false
ENV FILENAME=hsfd_nov2023_testing
ENV LIDAR_ADDR=192.168.168.128

# Hardware
ENV LIDAR=ouster
# options: integrated, nvidia, amd
# ENV GPU=nvidia

# Topics (Pointcloud and IMU)
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu

# RVIZ
ENV RVIZ_OUSTER=true
ENV RVIZ_DLIO=true

# DLIO specific setting for saving maps
ENV LEAF_SIZE=0.01
