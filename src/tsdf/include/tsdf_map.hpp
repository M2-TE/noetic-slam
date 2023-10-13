#pragma once

#include <cstdint>
#include <limits>
#include <vector>
#include <bit>
#include <optional>

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

    // insert single point into DAG
    inline void insert_point(Eigen::Vector3f realPos){

        print_vec3f(realPos, "> Inserting new point with position");
        Eigen::Vector3i voxelPos = (realPos * coordToVoxelRatio).cast<int>();
        print_vec3i(voxelPos, "Voxel position of inserted point is");

        // figure out which root the voxel will be in, as well as its position local to that root
        Eigen::Vector3i voxelRootPos = voxelPos / rootDimSize;
        print_vec3i(voxelRootPos, "Root  position of inserted point is");
        Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int x){ return x & (rootDimSize - 1); }); // modulo
        print_vec3i(localPos, "Local position of inserted point is");

        // shenanigans
        auto currentNode = try_get_root(voxelRootPos);
        if (currentNode.has_value()) {
            // go and get all the child nodes recursively until a node doesnt exist
            for (uint32_t i = 0; i < nDagLevels; i++) {
                // try_get_node();
            }
        }
        else {
            // create nodes starting from leaf
        }

        // for later
        // this is just a glorified for-loop, calling try_get_node each iteration
        // [&]<size_t... indices>(std::index_sequence<indices...>){
        //     (try_get_node<indices>(localPos), ...);
        // }(std::make_index_sequence<nDagLevels>{});
    }

private:
    // template<size_t i> // iteration with compile-time constant i
    inline std::optional<DAG::Pointer> try_get_node(Eigen::Vector3i localPos) {
        // TODO
    }

    // returns pointer to root node, else returns nothing
    inline std::optional<DAG::Pointer> try_get_root(Eigen::Vector3i voxelRootPos) {
        DAG::RootPos rootPos;
        rootPos |= (DAG::RootPos)voxelRootPos.x();
        rootPos |= (DAG::RootPos)voxelRootPos.y() << (DAG::xRootBits);
        rootPos |= (DAG::RootPos)voxelRootPos.z() << (DAG::xRootBits + DAG::yRootBits);

        if (dagRootMap.contains(rootPos)) return dagRootMap[rootPos];
        return {};
    }


    inline void insert_point_old(Eigen::Vector3f realPos) {
        // // convert from 1 to voxel position
        // Eigen::Vector3i voxelPos = (realPos * coordToVoxelRatio).cast<int>();
        // ROS_INFO(" ");
        // ROS_INFO("-> Inserting new point with position (%.2f, %.2f, %.2f)", 
        //     realPos.x(), realPos.y(), realPos.z());
        // ROS_INFO("Voxel position of inserted point is (%d, %d, %d)", 
        //     voxelPos.x(), voxelPos.y(), voxelPos.z());

        // // figure out which root the voxel will be in, as well as its position local to that root
        // Eigen::Vector3i voxelRootPos = voxelPos / rootDimSize;
        // ROS_INFO("Root  position of inserted point is (%d, %d, %d)", 
        //     voxelRootPos.x(), voxelRootPos.y(), voxelRootPos.z());
        // Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int x){ return x & (rootDimSize - 1); }); // modulo
        // ROS_INFO("Local position of inserted point is (%d, %d, %d)\n", 
        //     localPos.x(), localPos.y(), localPos.z());
        
        // // pack root position into 64 bits
        // DAG_RootPos rootPos = 0;
        // rootPos |= (DAG_RootPos)voxelRootPos.x();
        // rootPos |= (DAG_RootPos)voxelRootPos.y() << (xBits);
        // rootPos |= (DAG_RootPos)voxelRootPos.z() << (xBits + yBits);
        // DAG_Node& currentNode = dagLevels.get_root_node(rootPos);

        // // iterate through DAG levels to reach the correct leaf
        // for (uint8_t i = 0; i < dagLevels.size() - 1; i++) {

        //     // 1 bit for each octant
        //     // max value of childIndex here is 7
        //     uint8_t childMaskIndex = get_child_mask_index(localPos, i);
        //     uint8_t maskBit = 1 << childMaskIndex;

        //     // check if this child is already present in current node
        //     if (static_cast<uint8_t>(currentNode.childMask) & maskBit) {
        //         ROS_INFO("Found child in depth %d", i + 1);
        //         DAG_Pointer childPointer = currentNode.childPointers[childMaskIndex];
        //         currentNode = dagLevels.get_node(i + 1, childPointer);
        //     }
        //     else {
        //         ROS_INFO("Didn't find child in depth %d, finding/creating children..", i + 1);
        //         // we know that the child node doesnt exist, so we have to create it and all its children
        //         // starting with the leaves node containing the requested leaf
        //         // and propagating changes up to the last valid node

        //         // check if leaves node exists
        //         uint32_t signedDistance = 6; // hardcoded for now
        //         uint32_t leafIndex = get_child_mask_index(localPos, dagLevels.size() - 1);
        //         uint32_t nBitsPerLeaf = 4;
        //         // leavesMask will contain all the signed distances of each leaf
        //         uint32_t leavesMask = signedDistance << (leafIndex * nBitsPerLeaf);
        //         DAG_Pointer pLastChild = dagLevels.get_leaves_node_addr(leavesMask);

        //         // create all children, starting from the last node before the leaf
        //         for (uint k = dagLevels.size() - 2; k > i; k--) {

        //             childMaskIndex = get_child_mask_index(localPos, k);
                    
        //             // prepare key for hashtable to look for existing node
        //             DAG_Node nodeKey;
        //             nodeKey.childPointers[childMaskIndex] = pLastChild;
        //             nodeKey.childMask = 1 << childMaskIndex;
                    
        //             // look it up
        //             dagLevels.get_node_addr(nodeKey, k);

        //         }

        //         // // since we know all upcoming levels will not be present, cant just create them all?
        //         break;
        //     }
        // }

        // // TODO: access correct root
    }

    // check which child octant the voxel falls into
    inline uint8_t get_child_mask_index(Eigen::Vector3i localPos, int32_t parentDepth) {
        // these should normally be obtainable at compile time! (with constexpr for loop)
        int32_t nodeDimSize = rootDimSize >> parentDepth;
        int32_t nodeDimSizeHalf = nodeDimSize >> 1;
        int32_t moduloMask = nodeDimSize - 1;

        ROS_INFO("nodeDimSize of get_child_mask_index: %d", nodeDimSize);
        // ROS_INFO("localPos %d modulo nodeDimSize %d (=%d) ge nodeDimSizeHalf %d equals %d", 
        //     localPos.x(), nodeDimSize, localPos.x() & moduloMask, nodeDimSizeHalf, 
        //     (localPos.x() & moduloMask) >= nodeDimSizeHalf);
            
        uint8_t childIndex = 0;
        childIndex |= ((localPos.x() & moduloMask) >= nodeDimSizeHalf) << 0;
        childIndex |= ((localPos.y() & moduloMask) >= nodeDimSizeHalf) << 1;
        childIndex |= ((localPos.z() & moduloMask) >= nodeDimSizeHalf) << 2;
        return childIndex;
    }

private:
    // compile time constants
    static constexpr float voxelToCoordRatio = 0.01f; // every voxel unit represents this distance
    static constexpr float coordToVoxelRatio = 1.0f / voxelToCoordRatio; // simple reciprocal for conversion
    static constexpr int32_t nDagLevels = 5; // number of DAG levels including roots and leaves
    static constexpr int32_t rootDimSize = 2 << nDagLevels; // voxel size of each root node dimension

    // storage
    phmap::flat_hash_map<DAG::RootPos, DAG::Pointer> dagRootMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};