#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    source /root/repo/devel/setup.bash
    roslaunch ouster_ros driver.launch\
        sensor_hostname:=$LIDAR_ADDR\
        viz:=$OUSTER_RVIZ\
        imu_port:=7008\
        lidar_port:=7009
        # point_cloud_frame:=lidar_frame
        # else can use sensor_frame
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/ouster-stream.sh"
fi