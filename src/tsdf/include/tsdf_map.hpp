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
        // convert from real coordinate to voxel
        Eigen::Vector3i voxelPos = (position * coordToVoxelRatio).cast<int>();

        // figure out which root the voxel will be in
        Eigen::Vector3i rootParentPos = voxelPos / rootDimSize;
        // voxel position relative to root (why does eigen not have builtin modulo..)
        Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int x){ return x % rootDimSize; });
        
        // pack position into 64 bits
        DAG_RootPos rootPos = (DAG_RootPos)rootParentPos.x();
        rootPos |= (DAG_RootPos)rootParentPos.y() << xBits;
        rootPos |= (DAG_RootPos)rootParentPos.z() << (xBits + yBits);

        // look up root and set up iteration (creates new node if not already present)
        DAG_Node& currentNode = dagRoots[rootPos];

        // iterate through DAG levels to reach the correct leaf
        for (int i = 0; i < dagLevelCount; i++) {
            DAG_Level& level = dagLevels[i];

            // check which octant the voxel falls into (check each dimension individually)
            uint8_t maskBitOffset = 0;
            int32_t curDimThreshhold = rootDimSize / (2 << i);
            maskBitOffset |= (localPos.x() < curDimThreshhold) << 0;
            maskBitOffset |= (localPos.y() < curDimThreshhold) << 1;
            maskBitOffset |= (localPos.z() < curDimThreshhold) << 2;

            // 1 bit for each octant
            // max value of maskBitOffset here is 7
            uint8_t maskBit = 1 << maskBitOffset;

            // check if this child is already present in current node
            if (currentNode.childMask & maskBit) {
                // go one level deeper
                DAG_Pointer childPointer = currentNode.childPointers[maskBitOffset];
                currentNode = level.nodes[childPointer];
            }
            else {
                // create child node
                
            }
        }

        // TODO: access correct leaf node of currentNode
    }

private:
    // compile time constants
    static constexpr float voxelToCoordRatio = 0.01f; // every voxel unit represents this distance
    static constexpr float coordToVoxelRatio = 1.0f / voxelToCoordRatio; // simple reciprocal
    static constexpr int32_t totalDagLevelCount = 3; // number of levels including DAG_Root and DAG_Leaves
    static constexpr int32_t dagLevelCount = totalDagLevelCount - 2; // number of levels excluding DAG_Root and DAG_Leaves
    static constexpr int32_t leafDimSize = 1; // leaves are cubic 1x1x1 voxels
    static constexpr int32_t rootDimSize = leafDimSize << totalDagLevelCount; // roots are cubic voxels with dim size depending on the number of DAG levels
    static constexpr float rootDimSizeReal = (float)rootDimSize * voxelToCoordRatio;

    // some renaming for readability
    typedef int8_t TSDF_Point;
    typedef uint32_t DAG_Pointer;
    typedef uint64_t DAG_RootPos; // packing 3D position into single 64-bit unsigned integer
    static constexpr uint8_t xBits = 24;
    static constexpr uint8_t yBits = 24;
    static constexpr uint8_t zBits = 16;

    // prototype structures
    struct DAG_LeavesNode {
        std::array<TSDF_Point, 8> leaves;
    };
    struct DAG_Node {
        uint8_t childMask;
        uint8_t padding; // blank
        uint16_t refcount; // number of parents
        std::array<DAG_Pointer, 8> childPointers; // could make this sparse?
    };
    struct DAG_Level {
        phmap::flat_hash_map<std::array<DAG_Pointer, 8>, DAG_Pointer> hashmap;
        std::vector<DAG_Node> nodes;
    };

    // storage
    std::array<DAG_Level, dagLevelCount> dagLevels;
    phmap::btree_map<DAG_RootPos, DAG_Node> dagRoots;
};