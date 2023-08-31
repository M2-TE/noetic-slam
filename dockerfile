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
RUN apt install -y pcl-tools ros-noetic-rviz

FROM init AS setup
WORKDIR /root/repo/
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
# set build/runtime environment vars
ARG PCL_TOPIC=/ouster/points
ENV PCL_TOPIC_ENV=${PCL_TOPIC}
ARG IMU_TOPIC=/ouster/imu
ENV IMU_TOPIC_ENV=${IMU_TOPIC}
ENV NOETICSLAM_DOCKER=1