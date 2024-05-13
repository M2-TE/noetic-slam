#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then
    roslaunch ouster_ros driver.launch \
        sensor_hostname:=${LIDAR_ADDR} \
        viz:=${RVIZ_OUSTER} \
        imu_port:=7008 \
        lidar_port:=7009
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && source /root/repo/devel/setup.bash && /root/repo/scripts/ouster-stream.sh"
fi