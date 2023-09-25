FROM ros:noetic AS init

# set env var during docker build only
ARG DEBIAN_FRONTEND=noninteractive
# INSTALL DEPENDENCIES
RUN apt update && apt upgrade -y
# catkin_tools
RUN apt install -y python3-catkin-tools catkin-lint
# Ouster-ros dependencies
RUN apt install -y ros-noetic-pcl-ros ros-noetic-rviz build-essential libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev cmake
# DLIO dependencies
RUN apt install -y libomp-dev libpcl-dev libeigen3-dev ros-noetic-pcl-ros
# extras
RUN apt install -y git iputils-ping pcl-tools

FROM init AS setup
WORKDIR /root/repo/
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/repo/devel/setup.bash' >> /root/.bashrc
RUN echo 'PATH="$PATH:/root/repo/scripts"' >> /root/.bashrc
ENTRYPOINT [ "/bin/bash", "/root/repo/scripts/entrypoint.sh" ]
ENV ROSCONSOLE_FORMAT='[ROS${severity}]: ${message}'
ENV NOETICSLAM_DOCKER=1
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,display

## configurable env vars
FROM setup AS config
ENV AUTOSTART=true
# MODE=replay, record, stream
ENV MODE=replay
ENV FILENAME=hsfulda33_sep2023
ENV LIDAR_ADDR=192.168.168.128

# Topics (Pointcloud and IMU)
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu

# RVIZ
ENV OUSTER_RVIZ=true
ENV DLIO_RVIZ=true

# DLIO specific setting for saving maps
ENV LEAF_SIZE=0.01