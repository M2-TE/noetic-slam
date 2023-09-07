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
ENV ROSCONSOLE_FORMAT='[${severity}]: ${message}'
ENV NOETICSLAM_DOCKER=1

# config env vars
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu
ENV RVIZ_ON=false
ENV BAG_PATH=bags/hsfulda33.bag
ENV LEAF_SIZE=0.01
ENV OUT_NAME=hsfulda33