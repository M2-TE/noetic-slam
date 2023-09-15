#!/bin/sh

if [ $NOETICSLAM_DOCKER ]
then
    bash -c "\
    . devel/setup.bash &&\
    rosservice call /robot/dlio_map/save_pcd \$LEAF_SIZE /root/repo/maps/ &&\
    mv /root/repo/maps/dlio_map.pcd /root/repo/\$MAP_PATH.pcd &&\
    pcl_pcd2ply /root/repo/\$MAP_PATH.pcd /root/repo/\$MAP_PATH.ply"
else
    if [ $# -eq 2 ]
    then
        docker exec -it noeticslam bash -c "\
        . devel/setup.bash &&\
        rosservice call /robot/dlio_map/save_pcd $1 /root/repo/maps/ &&\
        mv /root/repo/maps/dlio_map.pcd /root/repo/$2.pcd &&\
        pcl_pcd2ply /root/repo/$2.pcd /root/repo/$2.ply"
    else
        echo "Need two arguments: leaf_node_size file_name"
    fi
fi