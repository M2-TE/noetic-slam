FROM ros:noetic AS init

# set env var during docker build only
ARG DEBIAN_FRONTEND=noninteractive
# INSTALL DEPENDENCIES
RUN apt update && apt upgrade -y
# general purpose
RUN apt install -y git
# catkin_tools
RUN apt install -y python3-catkin-tools
# DLIO dependencies
RUN apt install -y libomp-dev libpcl-dev libeigen3-dev ros-noetic-pcl-ros
# extras
RUN apt install -y pcl-tools ros-noetic-rviz expect-dev

FROM init AS setup
WORKDIR /root/repo/
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
ENV NOETICSLAM_DOCKER=1

# config env vars
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu
ENV RVIZ_ON=false
ENV BAG_PATH=bags/hsfulda33.bag
ENV LEAF_SIZE=0.01
ENV OUT_NAME=hsfulda33