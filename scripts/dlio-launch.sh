#!/bin/sh
if [ $NOETICSLAM_DOCKER ]
then
    bash -c "\
    . devel/setup.bash && \
    roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=false pointcloud_topic:=\${PCL_TOPIC_ENV} imu_topic:=\$IMU_TOPIC_ENV"
else
    docker exec -it noeticslam bash -c "\
    . devel/setup.bash && \
    roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=false pointcloud_topic:=\${PCL_TOPIC_ENV} imu_topic:=\$IMU_TOPIC_ENV"
fi