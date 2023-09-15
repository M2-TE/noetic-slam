#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    source /root/repo/devel/setup.bash
    roslaunch ouster_ros driver.launch\
        sensor_hostname:=$LIDAR_ADDR\
        viz:=$RVIZ_ON
        # point_cloud_frame:=lidar_frame
        # else can use sensor_frame
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/ouster-sensor.sh"
fi