#pragma once

struct TSDF_Point {
    float signedDistance;
};

class TSDF_Map {
public:
    void add_pointcloud(nav_msgs::PathConstPtr& pMsgPath, sensor_msgs::PointCloud2ConstPtr& pMsgPointcloud) {
        ROS_INFO("adding pointcloud to tsdf");

        // extract posetsdf
        poses.push_back(pMsgPath->poses.back().pose);
        // extract pcl
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(*pMsgPointcloud, pointcloud);

        // TODO: add points into tsdf map
        ROS_INFO("point count: %d", pointcloud.size());
    }
private:
    float maxSignedDistance = 3.0f;

    std::vector<geometry_msgs::Pose> poses;
    std::vector<TSDF_Point> points;
};