#!/bin/bash
if [ -z $ROS_DISTRO ]; then
    ROS_DISTRO="null"
fi
if [ $ROS_DISTRO = "noetic" ]; then
    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    source /opt/ros/noetic/setup.bash
    source ${SCRIPT_DIR}/../devel/setup.bash
    roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=${RVIZ_DLIO} pointcloud_topic:=${PCL_TOPIC} imu_topic:=${IMU_TOPIC}
    # roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=${RVIZ_DLIO} pointcloud_topic:=/os1_points imu_topic:=/imu/data_raw
else
    docker exec -it noeticslam bash -c "/root/repo/scripts/dlio-launch.sh"
fi