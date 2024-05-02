#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    source /opt/ros/noetic/setup.bash
    roscore > /dev/null &
    catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-11 -DCMAKE_CXX_COMPILER=/usr/bin/g++-11
    catkin build
    
    if [ $AUTOSTART = "true" ]
    then
        source /root/repo/devel/setup.bash
        scripts/compound-launch.sh
    else
        bash
    fi
else
    echo "This script should not be used manually outside the container (or ever, really)"
    exit 1
fi