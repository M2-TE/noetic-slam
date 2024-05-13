#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then
    source /root/repo/devel/setup.bash
    roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=${RVIZ_DLIO} pointcloud_topic:=${PCL_TOPIC} imu_topic:=${IMU_TOPIC}
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/dlio-launch.sh"
fi