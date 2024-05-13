#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then
    if [ -z "$1" ]; then
        echo "usage: scripts/ouster-record [bags/*.bag]"
        exit -1
    fi
    roslaunch ouster_ros record.launch \
        sensor_hostname:=${LIDAR_ADDR} \
        bag_file:=/root/repo/$1 \
        viz:=${RVIZ_OUSTER} \
        imu_port:=7008 \
        lidar_port:=7009
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && source /root/repo/devel/setup.bash && /root/repo/scripts/ouster-record.sh"
fi