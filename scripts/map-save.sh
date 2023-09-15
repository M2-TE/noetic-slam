#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    rosservice call /robot/dlio_map/save_pcd $LEAF_SIZE /root/repo/maps/
    mv /root/repo/maps/dlio_map.pcd /root/repo/$MAP_PATH.pcd
    pcl_pcd2ply /root/repo/$MAP_PATH.pcd /root/repo/$MAP_PATH.ply
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/map-save.sh"
fi