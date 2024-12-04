#!/bin/bash

# Save lidar data and images in filestructure
if [ -z $ROS_DISTRO ]; then
    ROS_DISTRO="null"
fi
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

    scripts/ouster-stream.sh &
    scripts/startimagesaver.sh &
    scripts/startlidardatasaver.sh

else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/combined-record.sh"
fi
