#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    # ensure roscore is up and running
    rostopic list &> /dev/null
    roscore_status=$?
    while [ $roscore_status -ne 0 ]; do
        rostopic list &> /dev/null
        roscore_status=$?
    done

    # let user override the mode
    if [ -z "$1" ]; then
        echo "error: select a mode: replay, record, stream"
        exit -1
    fi
    selected_mode=$1

    # handle selected mode
    if [ $selected_mode = "replay" ]; then
        scripts/dlio-launch.sh &
        DLIO_PID=$!
        scripts/rosbag-replay.sh
        # scripts/ouster-replay.sh

        # give dlio some time to fully process the trauma
        # sleep 2 && scripts/dlio-save.sh
        # TODO: kill dlio here

    elif [ $selected_mode = "record" ]; then
        rosbag record -O bags/${FILENAME}_generic.bag ${PCL_TOPIC} ${IMU_TOPIC} &
        scripts/ouster-record.sh

    elif [ $selected_mode = "stream" ]; then
        # stream the data and sic dlio on it
        scripts/dlio-launch.sh &
        DLIO_PID=$!
        scripts/ouster-stream.sh
        # TODO: kill dlio here
    else
        echo "Valid modes are replay, record or stream"
    fi
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/compound-launch.sh"
fi
