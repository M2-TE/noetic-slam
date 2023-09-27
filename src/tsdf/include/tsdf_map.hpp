#pragma once

#include <cstdint>
#include <limits>
#include <parallel_hashmap/phmap.h>
#include <parallel_hashmap/btree.h>
#include <Eigen/Dense>
#include "hash_spec.hpp"

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

        ROS_INFO("point count: %ld", pointcloud.size());
        for (auto cur = pointcloud.begin(); cur != pointcloud.end(); cur++) {
            // TODO: transform points with pose

            insert_point(cur->getVector3fMap());
        }
    }

private:
    inline void insert_point(Eigen::Vector3f position) {
        // convert from real coordinate to voxel-local
        Eigen::Vector3i voxelPos = (position * coordToVoxelRatio).cast<int>();

        // figure out which root octant position the voxel will be in
        Eigen::Vector3i octantPos = voxelPos / rootDimSize;
        
        ROS_INFO_STREAM("Point " << position << " reduced to pos " << voxelPos << " at root pos " << octantPos);
    }

private:
    // compile time constants
    static constexpr float voxelToCoordRatio = 0.01f; // every voxel unit represents this distance
    static constexpr float coordToVoxelRatio = 1.0f / voxelToCoordRatio; // reciprocal
    static constexpr uint32_t dagLevelCount = 3;
    static constexpr uint32_t leafDimSize = 1; // leaves are cubic 1x1x1 voxels
    static constexpr uint32_t rootDimSize = leafDimSize << dagLevelCount; // roots are cubic voxels with dim size depending on the number of DAG levels
    static constexpr float rootDimSizeReal = (float)rootDimSize * voxelToCoordRatio;

    // some renaming for readability
    typedef int8_t TSDF_Point;
    typedef uint32_t DAG_Pointer;
    typedef uint64_t DAG_RootPos; // packing 3D position into single 64-bit unsigned integer
    static constexpr uint8_t xBits = 24;
    static constexpr uint8_t yBits = 24;
    static constexpr uint8_t zBits = 16;

    // prototype structures
    struct DAG_Leaves {
        std::array<TSDF_Point, 8> leaves;
    };
    struct DAG_Node {
        uint8_t childMask;
        uint8_t padding; // blank
        uint16_t refcount; // number of parents
        std::array<DAG_Pointer, 8> childPointers; // could make this sparse?
    };
    struct DAG_Root {
        DAG_RootPos pos; // key into ordered map
    };
    struct DAG_Level {
        phmap::flat_hash_map<std::array<DAG_Pointer, 8>, DAG_Pointer> hashmap;
        std::vector<DAG_Node> nodes;
    };

    // storage
    std::array<DAG_Level, dagLevelCount> dagLevels;
    phmap::btree_map<DAG_RootPos, DAG_Root> dagRoots;
};