FROM ros:noetic AS init

# set env var during docker build only
ARG DEBIAN_FRONTEND=noninteractive
# INSTALL DEPENDENCIES
RUN apt-get update && apt-get upgrade -y
# catkin_tools
RUN apt-get install -y python3-catkin-tools catkin-lint
# Ouster-ros dependencies
RUN apt-get install -y ros-noetic-pcl-ros ros-noetic-rviz build-essential libeigen3-dev libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev cmake
# DLIO dependencies
RUN apt-get install -y libomp-dev libpcl-dev
# extra utils
RUN apt-get install -y git iputils-ping pcl-tools
# enable rendering
RUN apt-get install -y libgl1-mesa-glx libgl1-mesa-dri

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
# MODE=replay, record, stream
ENV AUTOSTART=false
ENV MODE=replay
ENV FILENAME=hsfd_nov2023
ENV LIDAR_ADDR=192.168.168.128

# sensor_frame (default) or lidar_frame (reprojection/raytracing)
ENV OUSTER_FRAME=sensor_frame

# Topics (Pointcloud and IMU)
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu

# RVIZ
ENV RVIZ_OUSTER=true
ENV RVIZ_DLIO=true

# DLIO specific setting for saving maps
ENV LEAF_SIZE=0.01