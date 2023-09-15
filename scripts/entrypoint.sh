#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    source /opt/ros/noetic/setup.bash
    roscore > /dev/null &
    if [ $AUTOSTART = "true" ]
    then
        scripts/compound-launch.sh
    else
        bash
    fi
else
    echo "This script should not be used manually outside the container (or ever, really)"
    exit 1
fi