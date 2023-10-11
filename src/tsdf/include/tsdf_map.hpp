#pragma once

#include <cstdint>
#include <limits>
#include <vector>
#include <bit>

#include <parallel_hashmap/phmap.h>
#include <parallel_hashmap/btree.h>
#include <Eigen/Dense>
#include "dag_structs.hpp"


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
    void DEBUGGING_INSERT() {
        insert_point({ 5.7f, 2.5f, 8.6f });
    }

private:
    inline void insert_point(Eigen::Vector3f realPos) {
        // convert from 1 to voxel position
        Eigen::Vector3i voxelPos = (realPos * coordToVoxelRatio).cast<int>();

        // figure out which root the voxel will be in, as well as its position local to that root
        Eigen::Vector3i voxelRootPos = voxelPos / rootDimSize;
        Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int x){ return x % rootDimSize; });
        ROS_INFO("Root Dimension = %d", rootDimSize);
        
        // pack root position into 64 bits
        DAG_RootPos rootPos = 0;
        rootPos |= (DAG_RootPos)voxelRootPos.x();
        rootPos |= (DAG_RootPos)voxelRootPos.y() << (xBits);
        rootPos |= (DAG_RootPos)voxelRootPos.z() << (xBits + yBits);

        // check if root is already present, create it and its pointer if not
        auto result = dagRootLevel.hashmap.try_emplace(rootPos);
        if (result.second) {
            result.first->second = dagRootLevel.roots.size();
            dagRootLevel.roots.emplace_back();
        }
        DAG_Node& currentNode = dagRootLevel.roots[result.first->second];

        // iterate through DAG levels to reach the correct leaf
        for (uint i = 0; i < dagLevels.size(); i++) {
            DAG_Level& childLevel = dagLevels[i];

            // check which octant the voxel falls into (check each dimension individually)
            int32_t nodeHalfDimSize = rootDimSize >> (i + 1);
            uint8_t childIndex = 0;
            childIndex |= (localPos.x() < nodeHalfDimSize) << 0;
            childIndex |= (localPos.y() < nodeHalfDimSize) << 1;
            childIndex |= (localPos.z() < nodeHalfDimSize) << 2;
            ROS_INFO("Node Dimension (level %d) = %d", i, nodeHalfDimSize);

            // 1 bit for each octant
            // max value of childIndex here is 7
            uint8_t maskBit = 1 << childIndex;

            // check if this child is already present in current node
            if (static_cast<uint8_t>(currentNode.childMask) & maskBit) {
                ROS_INFO("Found child!");
                // go one level deeper
                DAG_Pointer childPointer = currentNode.childPointers[childIndex];
                currentNode = childLevel.nodes[childPointer];
            }
            else {
                ROS_INFO("Didn't find child.");
                // we know that the child node doesnt exist, so we have to create it and all its children
                // and then propagate changes towards the root

                //

                // create child node

                // create all children, starting from leaf node
                for (uint k = 0;false;) {
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
    static constexpr int32_t dagLevelCount = 3; // number of levels including root and leaf levels
    static constexpr int32_t rootDimSize = 1 << dagLevelCount; // voxel size of each root node dimension, this*this*this is its size

    // constants for DAG_RootPos
    static constexpr uint8_t xBits = 24; // x bits for DAG_RootPos
    static constexpr uint8_t yBits = 24; // Y bits for DAG_RootPos
    static constexpr uint8_t zBits = 16; // z bits for DAG_RootPos

    // storage
    DAG_RootLevel dagRootLevel;
    DAG_LeafLevel dagLeafLevel;
    std::array<DAG_Level, dagLevelCount - 2> dagLevels;
    // std::vector<DAG_Level> dagLevels;
    // phmap::btree_map<DAG_RootPos, DAG_Pointer> dagRoots;
};