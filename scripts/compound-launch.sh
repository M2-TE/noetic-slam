#!/bin/sh

# launch roscore in the background
rostopic list &> /dev/null
roscore_status=$?
if [ $roscore_status -ne 0 ]
then
    roscore > /dev/null &
fi

# build entire catkin workspace
catkin build

# ensure roscore is up and running
rostopic list &> /dev/null
roscore_status=$?
while [ $roscore_status -ne 0 ]
do
    rostopic list &> /dev/null
    roscore_status=$?
done

# replay the bagfile and sic dlio on it
trap 'kill $DLIO_PID; exit' SIGINT
scripts/dlio-launch.sh &
DLIO_PID=$!
scripts/rosbag-play.sh
# give dlio some time to fully process the trauma
sleep 2 && scripts/map-save.sh

# since dlio was started in background, stop it manually
kill -SIGINT $DLIO_PID