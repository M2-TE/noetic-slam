#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then

    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    source /opt/ros/noetic/setup.bash 
    source ${SCRIPT_DIR}/../devel/setup.bash

    # ensure roscore is up and running
    roscore &
    rostopic list &> /dev/null
    roscore_status=$?
    while [ $roscore_status -ne 0 ]; do
        rostopic list &> /dev/null
        roscore_status=$?
    done

    # let user override the mode
    if [ -z "$1" ]; then
        echo "usage: scripts/compound-launch.sh [replay, record, stream] [bags/*.bag]"
        exit -1
    fi

    # handle selected mode
    if [ $1 = "replay" ]; then
        if [ -z "$2" ]; then
            echo "usage: scripts/compound-launch.sh replay [bags/*.bag]"
            exit -1
        fi
        scripts/dlio-launch.sh &
        scripts/rosbag-replay.sh $2

    elif [ $1 = "record" ]; then
        if [ -z "$2" ]; then
            echo "usage: scripts/compound-launch.sh record [bags/*.bag]"
            exit -1
        fi
        # rosbag record -O bags/${FILENAME}_generic.bag ${PCL_TOPIC} ${IMU_TOPIC} &
        scripts/ouster-record.sh $2

    elif [ $1 = "stream" ]; then
        # stream the data and sic dlio on it
        scripts/dlio-launch.sh &
        scripts/ouster-stream.sh
    else
        echo "Valid modes are replay, record or stream"
    fi
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/compound-launch.sh"
fi
