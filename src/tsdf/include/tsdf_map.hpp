#pragma once

#include <cstdint>
#include <limits>

struct TSDF_Point {
    TSDF_Point(int8_t val) : signedDistance(val) {}

    // simplified for performance (-127 to 127 instead of -128 to 127)
    inline float get() {
        static constexpr float div = 1.0f / 127.0f;
        float f = static_cast<float>(signedDistance) * div;
        // f = f*f*f; // higher accuracy in lower value ranges
        return f;
    }
    inline void set(float val) {
        // val = std::cbrt(val);
        signedDistance = static_cast<int8_t>(val * 127.0f);
    }

private:
    int8_t signedDistance;
};

class TSDF_Map {
public:
    void add_pointcloud(nav_msgs::PathConstPtr& pMsgPath, sensor_msgs::PointCloud2ConstPtr& pMsgPointcloud) {
        ROS_INFO("adding pointcloud to tsdf");

        // extract pose
        geometry_msgs::Pose pose = {};
        pose = pMsgPath->poses.back().pose;
        // extract pcl
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(*pMsgPointcloud, pointcloud);

        // TODO: add points into tsdf map
        ROS_INFO("point count: %ld", pointcloud.size());
    }
private:
    float maxSignedDistance = 3.0f;

};