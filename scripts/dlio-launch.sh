#!/bin/sh
docker exec -it noeticslam bash -c "\
. /opt/ros/noetic/setup.bash && \
. devel/setup.bash && \
roslaunch direct_lidar_inertial_odometry dlio.launch rviz:=false pointcloud_topic:=\${PCL_TOPIC_ENV} imu_topic:=\$IMU_TOPIC_ENV"