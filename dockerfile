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
# for mesa rendering
# RUN apt-get install -y mesa-utils libgl1-mesa-glx libgl1-mesa-dri

FROM init AS setup
WORKDIR /root/repo/
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/repo/devel/setup.bash' >> /root/.bashrc
RUN echo 'PATH=$PATH:/root/repo/scripts' >> /root/.bashrc
ENTRYPOINT [ "/bin/bash", "/root/repo/scripts/.entrypoint.sh" ]
ENV ROSCONSOLE_FORMAT='[ROS${severity}]: ${message}'
ENV NOETICSLAM_DOCKER=1
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV QT_X11_NO_MITSHM=1

## configurable env vars
FROM setup AS config
# MODE=replay, record, stream
ENV MODE=replay
ENV AUTOSTART=false
ENV FILENAME=hsfd_nov2023
ENV LIDAR_ADDR=192.168.168.128

# Hardware
ENV LIDAR=ouster
# GPU=integrated, nvidia, amd
ENV GPU=integrated

# Topics (Pointcloud and IMU)
ENV PCL_TOPIC=/ouster/points
ENV IMU_TOPIC=/ouster/imu

# RVIZ
ENV RVIZ_OUSTER=false
ENV RVIZ_DLIO=false

# DLIO specific setting for saving maps
ENV LEAF_SIZE=0.01

# pcl_concatenate_points_pcd maps/*.pcd
# rosrun pcl_ros pointcloud_to_pcd input:=/my_cloud _prefix:=/tmp/pcd/vel_ TODO EDIT THIS