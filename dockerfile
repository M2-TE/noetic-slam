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
RUN apt install -y git pcl-tools ros-noetic-rviz

FROM init AS setup
WORKDIR /root/repo/
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
ENTRYPOINT [ "/bin/bash", "/root/repo/scripts/entrypoint.sh" ]
ENV ROSCONSOLE_FORMAT='[${severity}]: ${message}'
ENV NOETICSLAM_DOCKER=1

# configurable env vars
FROM setup AS config
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu
ENV AUTOSTART=false
ENV RVIZ_ON=false
ENV MODE=replay
# TODO: implement record mode properly
# TODO: implement sensor mode properly

# Requirements
# MODE=replay: BAG_PATH, MAP_PATH
# MODE=record: LIDAR_ADDR, BAG_PATH
# MODE=sensor: LIDAR_ADDR, MAP_PATH
# paths are relative to workspace root
ENV BAG_PATH=bags/hsfulda33_aug2023.bag
ENV MAP_PATH=maps/hsfulda33_aug2023
ENV LIDAR_ADDR=192.168.168.128

# DLIO specific setting to save maps
ENV LEAF_SIZE=0.01