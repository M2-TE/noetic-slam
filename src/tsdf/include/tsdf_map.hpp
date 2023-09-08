#pragma once

#include <cstdint>
#include <limits>
#include <parallel_hashmap/phmap.h>
#include "hash_spec.hpp"

struct TSDF_Point {
    TSDF_Point(int8_t val) : signedDistance(val) {}

    // simplified for performance (-127 to 127 instead of -128 to 127)
    inline float get() {
        static constexpr float div = 1.0f / 127.0f;
        float f = static_cast<float>(signedDistance) * div;
        return f;
    }
    inline void set(float val) {
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

    // typedef uint32_t DAG_Pointer;
    // struct DAG_Leaf {
    //     std::array<TSDF_Point, 8> leaves;
    // };
    // struct DAG_Node {
    //     uint8_t childMask;
    //     uint8_t padding; // blank
    //     uint16_t refcount; // number of parents
    //     std::array<DAG_Pointer, 8> childPointers;
    // };
    // struct DAG_Level {
    //     phmap::parallel_flat_hash_map<std::array<DAG_Pointer, 8>, DAG_Pointer> hashmap;
    //     std::vector<DAG_Node> nodes;
    // };
    // std::vector<DAG_Level> dagLevels;

    typedef uint32_t DAG_Pointer;
    struct DAG_Leaf {
        std::array<TSDF_Point, 8> leaves;
    };
    struct DAG_Node {
        uint8_t childMask;
        uint8_t padding; // blank
        uint16_t refcount; // number of parents
        std::array<DAG_Pointer, 8> childPointers;
    };
    struct DAG_Level {
        phmap::parallel_flat_hash_map<std::array<DAG_Pointer, 8>, DAG_Pointer> hashmap;
        std::vector<DAG_Node> nodes;
    };
    std::vector<DAG_Level> dagLevels;
};