#pragma once

#include <iostream>
#include <cstdint>
#include <array>
//
#include <Eigen/Dense>
#include <parallel_hashmap/phmap.h>
//
#include "constants.hpp"
#include "dag_structs.hpp"

class TSDF_Map {
public:
    TSDF_Map() {
        // set up hash and equality functors with proper references
        for (size_t i = 0; i < dagLevels.size(); i++) {
            DAG::Level& level = dagLevels[i];

            DAG::HashFunctor hashFnc = { &level.data };
            DAG::CompFunctor compFnc = { &level.data };
            level.pointerSet = decltype(level.pointerSet)(0, hashFnc, compFnc);
        }
    }
    void insert_pointcloud(sensor_msgs::PointCloud2ConstPtr& pMsgPointcloud) {
        ROS_INFO("adding pointcloud to tsdf");

        // extract pcl
        pcl::PointCloud<Point> pointcloud = {};
        pcl::fromROSMsg(*pMsgPointcloud, pointcloud);

        // ROS_INFO("point count: %ld", pointcloud.size());
        for (auto cur = pointcloud.begin(); cur != pointcloud.end(); cur++) {
            // TODO: transform points with pose
            insert_point(cur->getVector3fMap());
        }
    }
    inline void insert_point(const Eigen::Vector3f& realPos) {

        // figure out which root the voxel will be in, as well as its position local to that root
        Eigen::Vector3i voxelPos = (realPos * coordToVoxelRatio).cast<int32_t>();
        Eigen::Vector3f coordPos = voxelPos.cast<float>() * voxelToCoordRatio;
        Eigen::Vector3i voxelRootPos = voxelPos / rootDimSize;

        // some debugging output for now
        // print_vec3(realPos,  "Real  position");
        print_vec3(voxelPos, "Voxel position");
        // print_vec3(coordPos, "Coord position");

        DAG::NodePointer root = get_root_node(voxelRootPos);
        [&]<size_t... indices>(std::index_sequence<indices...>) {
            
            DAG::NodePointer lastNode = root;
            bool bContinue = true;
            // find nodes recursively until one no longer exists
            ((lastNode = get_child_node<indices + 1>(voxelPos, lastNode, bContinue)), ...);

        } (std::make_index_sequence<nDagLevels - 2>{});
    }

private:
    template<int32_t depth = 0> inline DAG::NodePointer get_root_node(const Eigen::Vector3i voxelRootPos) {
        DAG::RootPos rootPos = (DAG::RootPos)voxelRootPos.x();
        rootPos |= (DAG::RootPos)voxelRootPos.y() << (DAG::xRootBits);
        rootPos |= (DAG::RootPos)voxelRootPos.z() << (DAG::xRootBits + DAG::yRootBits);

        // check if a new node is emplaced
        auto [pRoot, bEmplacedNew] = dagRootMap.emplace(rootPos, dagLevels[depth].data.size());
        if (bEmplacedNew) {
            // always allocate all 8 children + child mask for a root
            std::array<DAG::NodePointer, DAG::nLeavesPerNode + 1> children;
            for (auto& child : children) child = 0;
            dagLevels[depth].data.insert(dagLevels[depth].data.end(), children.begin(), children.end());
            std::cout << "created new root node\n";
        }
        return pRoot->second;
    }
    template<int32_t depth> inline DAG::ChildMask get_child_bit(const Eigen::Vector3i& voxelPos) {
        static constexpr int32_t reverseDepth = nDagLevels - depth;
        static constexpr int32_t dimSize = 1 << reverseDepth;

        // calculate local position within current node
        // localPos will describe a 2x2x2 grid
        const Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int32_t x){ return x % dimSize / (dimSize / 2); });

        DAG::ChildMask childBit = 1;
        childBit = childBit << (localPos.x() << 0);
        childBit = childBit << (localPos.y() << 1);
        childBit = childBit << (localPos.z() << 2);
        return childBit;
    }
    template<int32_t depth> inline DAG::NodePointer get_child_node(Eigen::Vector3i& voxelPos, DAG::NodePointer parent, bool& bContinue) {
        if (!bContinue) return 0;
        std::cout << depth << '\n';

        // try to find child in parent
        DAG::Level& parentLevel = dagLevels[depth - 1];
        DAG::ChildMask childMask = (DAG::ChildMask)parentLevel.data[parent];
        DAG::ChildMask childBit = get_child_bit<depth>(voxelPos);
        DAG::ChildMask result = childMask & childBit;
        if (result) {
            std::cout << "found child at depth " << depth << '\n';
            // we need to know how many bits were set in front this child node (children are in order)
            // instead of looping over each bit, it is faster to count bits via popcount()
            // after applying a mask that only goes up to the childBit bit
            DAG::ChildMask submask = (childBit << 1) - 1;
            DAG::NodePointer childIndex = std::popcount<DAG::ChildMask>(result & submask);
            return parent + childIndex; // move pointer forward via child index
        }
        else bContinue = false;

        DAG::NodePointer childNode = create_children<depth>(voxelPos);
        if (depth == 1) {
            // at this depth its easy, because the parent (root) depth always has all 8 children allocated
            
            // add new child to mask
            parentLevel.data[parent] = (uint32_t)(childMask | childBit);
            // add child to corresponding position
            uint32_t childOffset = 0; // TODO: calc
            parentLevel.data[parent + childOffset] = childNode;
        }
        else {
            // at all other depths, the parent's parent needs to be accessed
        }
        return 0; // todo: remove the need for return val
    }
    template<int32_t depth> inline DAG::NodePointer create_children(const Eigen::Vector3i& voxelPos) {
        std::cout << "creating new children starting from depth " << depth << '\n';

        // create children all the way from leaves up until the <depth> given via template
        DAG::NodePointer lastNode = 0;
        [&]<size_t... indices>(std::index_sequence<indices...>) {
            // keep track of last node
            lastNode = create_node_leaf<nDagLevels - 1>(voxelPos);
            ((lastNode = create_node_generic<nDagLevels - indices - 2>(voxelPos, lastNode)), ...);
        } (std::make_index_sequence<nDagLevels - depth>{});
        return lastNode;
    }
    template<int32_t depth> inline DAG::NodePointer create_node_leaf(const Eigen::Vector3i& voxelPos) { // TODO: calc signed distance properly. also dont need childBit, fill all leaves at once
        DAG::ChildMask childBit = get_child_bit<depth>(voxelPos);
        DAG::Level& level = dagLevels[depth];

        float sd = voxelToCoordRatio * 0.2345f; // signed distance TODO: calc
        float sdNorm = sd * (1.0f / voxelToCoordRatio); // normalize signed distance between 0 and 1
        sd = sdNorm * static_cast<float>(DAG::maxSignedDistance); // scale up to uint4_t range

        // cast to uint and shift into position within leaves node
        // each leaf will take up DAG::nBitsPerLeaf bits
        DAG::Leaves leaves = static_cast<DAG::Leaves>(sd) << (childBit * DAG::nBitsPerLeaf);

        // check if a new node is emplaced
        auto [pLeaf, bEmplacedNew] = dagLeafMap.emplace(leaves, level.data.size());
        if (bEmplacedNew) {
            level.data.emplace_back(leaves);
        }
        return pLeaf->second;
    }
    template<int32_t depth> inline DAG::NodePointer create_node_generic(const Eigen::Vector3i& voxelPos, const DAG::NodePointer child) {
        DAG::ChildMask childBit = get_child_bit<depth>(voxelPos);
        DAG::Level& level = dagLevels[depth];

        // create a temporary node and only progress tracker if it was actually inserted
        DAG::NodePointer key = level.dataSize;
        level.data.insert(level.data.begin() + key, {
            childBit,   // mask
            child       // child pointer
        });

        // check if a new node is emplaced
        auto [pNode, bEmplacedNew] = level.pointerSet.emplace(key);
        if (bEmplacedNew) level.dataSize += 2;
        return *pNode;
    }
    
private:
    phmap::flat_hash_map<DAG::RootPos, DAG::NodePointer> dagRootMap;
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};