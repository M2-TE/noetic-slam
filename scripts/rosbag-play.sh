#!/bin/sh
if [ $# -eq 0 ]
then
    echo "Missing path argument (relative to workspace root)"
else
    if [ -f "$1" ]
    then
        if [ $NOETICSLAM_DOCKER ]
        then
            sh -c "\
            rosbag play $1"
        else
            docker exec -it noeticslam sh -c "\
            . /opt/ros/noetic/setup.sh &&\
            rosbag play $1"
        fi
    else
        if [ -f "bags/$1" ]
        then
            if [ $NOETICSLAM_DOCKER ]
            then
                sh -c "\
                rosbag play bags/$1"
            else
                docker exec -it noeticslam sh -c "\
                . /opt/ros/noetic/setup.sh &&\
                rosbag play bags/$1"
            fi
        else
            echo "Bag file could not be found"
        fi
    fi
fi