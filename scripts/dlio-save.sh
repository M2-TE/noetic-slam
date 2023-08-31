#!/bin/sh
if [ $# -lt 2 ]
then
    echo "Need two arguments: leaf_node_size file_name"
else
    if [ $NOETICSLAM_DOCKER ]
    then
        bash -c "\
        . devel/setup.bash &&\
        rosservice call /robot/dlio_map/save_pcd $1 /root/repo/maps/ &&\
        mv /root/repo/maps/dlio_map.pcd /root/repo/maps/$2.pcd &&\
        pcl_pcd2ply /root/repo/maps/$2.pcd /root/repo/maps/$2.ply"
    else
        docker exec -it noeticslam bash -c "\
        . devel/setup.bash &&\
        rosservice call /robot/dlio_map/save_pcd $1 /root/repo/maps/ &&\
        mv /root/repo/maps/dlio_map.pcd /root/repo/maps/$2.pcd &&\
        pcl_pcd2ply /root/repo/maps/$2.pcd /root/repo/maps/$2.ply"
    fi
fi