#!/bin/sh
if [ $NOETICSLAM_DOCKER ]
then
    bash -c "\
    . devel/setup.bash && \
    roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=\$RVIZ_ON pointcloud_topic:=\$PCL_TOPIC imu_topic:=\$IMU_TOPIC"
else
    docker exec -it noeticslam bash -c "\
    . devel/setup.bash && \
    roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=true pointcloud_topic:=$1 imu_topic:=$1"
fi