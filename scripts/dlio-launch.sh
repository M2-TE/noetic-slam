#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then
    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    source /opt/ros/noetic/setup.bash
    source ${SCRIPT_DIR}/../devel/setup.bash
    roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=${RVIZ_DLIO} pointcloud_topic:=${PCL_TOPIC} imu_topic:=${IMU_TOPIC}
else
    docker exec -it noeticslam bash -c "/root/repo/scripts/dlio-launch.sh"
fi