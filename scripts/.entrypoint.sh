#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]
then
    source /opt/ros/noetic/setup.bash
    roscore > /dev/null &
    catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_COMPILER=/usr/bin/gcc-11 -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache
    catkin build
    bash --rcfile <(echo "source /root/repo/devel/setup.bash")
else
    echo "This script should not be used manually outside the container (or ever, really)"
    exit 1
fi