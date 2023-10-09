#pragma once

#include <cstdint>
#include <limits>
#include <vector>
#include <bit>

#include <parallel_hashmap/phmap.h>
#include <parallel_hashmap/btree.h>
#include <Eigen/Dense>
#include "hash_spec.hpp"

class TSDF_Map {
public:
    TSDF_Map() {
        // create empty node at each level since index 0 will be the synonym used for invalid indices
        for (uint i = 0; i < dagLevelCount; i++) {
            dagLevels[i].nodes.emplace_back();
        }
    }

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

    void DEBUGGING_INSERT() {
        insert_point({ 5.7f, 2.5f, 8.6f });
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

        // look up root and set up iteration (creates new node pointer if not already present)
        DAG_Pointer& pCurrentNode = dagRoots[rootPos];
        if (pCurrentNode == 0) {
            pCurrentNode = dagLevels.size();
            dagLevels[0].nodes.emplace_back();
        }
        DAG_Node& currentNode = dagLevels[0].nodes[pCurrentNode];

        // TODO: iterate cleanly from root to leaf until hitting an invalid node
        // should also check the constexpr values since dagLevelCount changed
        return;

        // iterate through DAG levels to reach the correct leaf
        for (uint i = 1; i < dagLevelCount; i++) {
            DAG_Level& childLevel = dagLevels[i];

            // check which octant the voxel falls into (check each dimension individually)
            uint8_t childIndex = 0;
            int32_t curDimThreshhold = rootDimSize / (2 << i);
            childIndex |= (localPos.x() < curDimThreshhold) << 0;
            childIndex |= (localPos.y() < curDimThreshhold) << 1;
            childIndex |= (localPos.z() < curDimThreshhold) << 2;

            // 1 bit for each octant
            // max value of childIndex here is 7
            uint8_t maskBit = 1 << childIndex;

            // check if this child is already present in current node
            if (static_cast<uint8_t>(currentNode.childMask) & maskBit) {
                // go one level deeper
                DAG_Pointer childPointer = currentNode.childPointers[childIndex];
                currentNode = childLevel.nodes[childPointer];
            }
            else {
                // we know that the child node doesnt exist, so we have to create it and all its children
                // and then propagate changes towards the root

                //

                // create child node

                // create all children, starting from leaf node
                for (uint k = 0;;) {
                    uint32_t nodePointer = (uint32_t)childLevel.nodes.size();
                    childLevel.nodes.emplace_back(); // todo: still needs to be added to hashmap
                }

                // since we know all upcoming levels will not be present, cant just create them all?
                break;
            }
        }

        // access correct root
    }

private:
    // compile time constants
    static constexpr float voxelToCoordRatio = 0.01f; // every voxel unit represents this distance
    static constexpr float coordToVoxelRatio = 1.0f / voxelToCoordRatio; // simple reciprocal
    static constexpr int32_t dagLevelCount = 3; // number of levels including roots and leaves
    static constexpr int32_t leafDimSize = 1; // leaves are cubic 1x1x1 voxels
    static constexpr int32_t rootDimSize = leafDimSize << dagLevelCount; // roots are cubic voxels with dim size depending on the number of DAG levels
    static constexpr float rootDimSizeReal = (float)rootDimSize * voxelToCoordRatio;

    // some renaming for readability
    typedef int8_t TSDF_Point;
    typedef uint32_t DAG_Pointer;
    typedef uint64_t DAG_RootPos; // packing 3D position into single 64-bit unsigned integer
    static constexpr uint8_t xBits = 24;
    static constexpr uint8_t yBits = 24;
    static constexpr uint8_t zBits = 16;

    // prototype structures
    struct DAG_Node {
        uint32_t nParents : 24;
        uint32_t childMask : 8;
        std::array<DAG_Pointer, 8> childPointers; // could make this sparse
    };
    struct DAG_Level {
        phmap::flat_hash_map<std::array<DAG_Pointer, 8>, DAG_Pointer> hashmap;
        std::vector<DAG_Node> nodes;
    };

    // storage
    std::array<DAG_Level, dagLevelCount> dagLevels;
    phmap::btree_map<DAG_RootPos, DAG_Pointer> dagRoots;
};