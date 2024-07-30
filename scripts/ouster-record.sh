#!/bin/bash
if [ -z $ROS_DISTRO ]; then
    ROS_DISTRO="null"
fi
if [ $ROS_DISTRO = "noetic" ]; then
    if [ -z "$1" ]; then
        echo "usage: scripts/ouster-record [bags/*.bag]"
        exit -1
    fi
    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    source /opt/ros/noetic/setup.bash
    source ${SCRIPT_DIR}/../devel/setup.bash
    roslaunch ouster_ros record.launch \
        sensor_hostname:=${LIDAR_ADDR} \
        bag_file:=${SCRIPT_DIR}/../$1 \
        viz:=${RVIZ_OUSTER} \
        imu_port:=7008 \
        lidar_port:=7009
else
    docker exec -it noeticslam bash -c "/root/repo/scripts/ouster-record.sh"
fi