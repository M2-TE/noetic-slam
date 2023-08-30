FROM ros:noetic

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

# build catkin workspace
COPY . /root/repo
WORKDIR /root/repo
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin init && catkin build'

# prepare entry
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/repo/devel/setup.bash' >> /root/.bashrc