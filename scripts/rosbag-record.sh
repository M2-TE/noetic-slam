#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    source /root/repo/devel/setup.bash
    roslaunch ouster_ros record.launch\
        sensor_hostname:=$LIDAR_ADDR\
        bag_file:=$BAG_PATH
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/rosbag-record.sh"
fi