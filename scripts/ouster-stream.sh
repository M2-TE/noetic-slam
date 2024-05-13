#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then
    source /opt/ros/noetic/setup.bash
    source ${SCRIPT_DIR}/../devel/setup.bash
    roslaunch ouster_ros driver.launch \
        sensor_hostname:=${LIDAR_ADDR} \
        viz:=${RVIZ_OUSTER} \
        imu_port:=7008 \
        lidar_port:=7009
else
    docker exec -it noeticslam bash -c "/root/repo/scripts/ouster-stream.sh"
fi