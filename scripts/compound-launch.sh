#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    # ensure roscore is up and running
    rostopic list &> /dev/null
    roscore_status=$?
    while [ $roscore_status -ne 0 ]
    do
        rostopic list &> /dev/null
        roscore_status=$?
    done

    if [ $MODE = "replay" ]
    then
        # replay the bagfile and sic dlio on it
        scripts/dlio-launch.sh &
        DLIO_PID=$!
        scripts/rosbag-replay.sh

        # give dlio some time to fully process the trauma
        sleep 2 && scripts/map-save.sh

    elif [ $MODE = "record" ]
    then
        scripts/ouster-record.sh

    elif [ $MODE = "stream" ]
    then
        # stream the data and sic dlio on it
        scripts/dlio-launch.sh &
        DLIO_PID=$!
        scripts/ouster-stream.sh

    else
        echo "Set \$MODE to replay, record or stream"
    fi
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/compound-launch.sh"
fi
