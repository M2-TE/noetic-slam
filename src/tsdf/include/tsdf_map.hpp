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
        print_vec3(voxelPos, "Voxel position");

        // retrieve leaf node from tree 
        // (parent will contain leaf, as multiple leaves are packed into single node)
        DAG::NodePointer parent = get_root_node(voxelPos / rootDimSize);
        DAG::NodePointer parentParent = 0;
        [&]<size_t... indices>(std::index_sequence<indices...>) {
            
            bool bContinue = true;
            // find nodes recursively until one no longer exists
            ((get_child_node<indices + 1>(voxelPos, parent, parentParent, bContinue)), ...);

        } (std::make_index_sequence<nDagLevels - 1>{});
        
        Eigen::Vector3f normal = realPos; // todo
        // overwrite leaf node contents based on value to be inserted
        // for tsdf: compare each leaflet!
        // if any changes need to be made, make leaf node copy and change as needed
        // then replace child in parentParent
        // note: parent will hold the current leaf node

        // TODO
    }

private:
    inline DAG::ChildMask get_child_index(DAG::ChildMask childMask, DAG::ChildMask childBit) {
        // mask that contains all bits before current childBit
        DAG::ChildMask submask = (childBit << 1) - 1;
        // by counting set bits before childBit, we get the index of the child
        return std::popcount<DAG::ChildMask>(childMask & submask);
    }
    template<int32_t depth> inline DAG::ChildMask get_child_bit(Eigen::Vector3i& voxelPos) { // position of <depth-1> child within <depth> childMask
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

    inline DAG::NodePointer get_root_node(Eigen::Vector3i voxelRootPos) {
        DAG::RootPos rootPos = (DAG::RootPos)voxelRootPos.x();
        rootPos |= (DAG::RootPos)voxelRootPos.y() << (DAG::xRootBits);
        rootPos |= (DAG::RootPos)voxelRootPos.z() << (DAG::xRootBits + DAG::yRootBits);

        // check if a new node is emplaced
        auto [pRoot, bEmplacedNew] = dagRootMap.emplace(rootPos, dagLevels[0].data.size());
        if (bEmplacedNew) {
            // always allocate all 8 children + child mask for a root
            std::array<DAG::NodePointer, DAG::nLeavesPerNode + 1> children;
            for (auto& child : children) child = 0;
            dagLevels[0].data.insert(dagLevels[0].data.end(), children.begin(), children.end());
            std::cout << "created new root node\n";
        }
        return pRoot->second;
    }
    inline DAG::NodePointer create_node_leaf(Eigen::Vector3i voxelPos) {
        DAG::ChildMask childBit = get_child_bit<nDagLevels - 1>(voxelPos);
        DAG::Level& level = dagLevels[nDagLevels - 1];
        
        Eigen::Vector3f f = {0.0f, 0.0f, 0.0f}; // temporary
        DAG::Leaves leaves = calculate_leaves(voxelPos, f);

        // check if a new node is emplaced
        auto [pLeaf, bEmplacedNew] = dagLeafMap.emplace(leaves, level.data.size());
        if (bEmplacedNew) {
            level.data.emplace_back(leaves);
        }
        return pLeaf->second;
    }
    inline DAG::Leaves calculate_leaves(Eigen::Vector3i voxelPos, Eigen::Vector3f realPos) {
        DAG::ChildMask childBit = 1; // temporary
        
        float sd = voxelToCoordRatio * 0.2345f; // signed distance TODO: calc
        float sdNorm = sd * (1.0f / voxelToCoordRatio); // normalize signed distance between 0 and 1
        sd = sdNorm * static_cast<float>(DAG::maxSignedDistance); // scale up to uint4_t range

        // cast to uint and shift into position within leaves node
        // each leaf will take up DAG::nBitsPerLeaf bits
        DAG::Leaves leaves = static_cast<DAG::Leaves>(sd) << (childBit * DAG::nBitsPerLeaf);

        // TODO: this whole thing should write a signed distance into EVERY leaf
        // that means, all 8 children of this leaf node should be written into, not just a single one
        // get_child_bit is therefore not really needed here
        // or rather, a different version should be made

        return leaves;
    }
    inline DAG::Leaves update_leaves() {
        // do i even want/need this?
        return 0;
    }

    template<int32_t depth> inline void get_child_node(Eigen::Vector3i& voxelPos, DAG::NodePointer& parent, DAG::NodePointer& parentParent, bool& bContinue) {
        if (!bContinue) return;

        // try to find child in parent
        DAG::Level& parentLevel = dagLevels[depth - 1];
        DAG::ChildMask childMask = (DAG::ChildMask)parentLevel.data[parent];
        DAG::ChildMask childBit = get_child_bit<depth - 1>(voxelPos);
        DAG::ChildMask result = childMask & childBit;
        if (result) {
            parentParent = parent; // save for later
            parent = parentLevel.data[parent + get_child_index(result, childBit)]; // return pointer to child
            return;
        }
        else bContinue = false;

        // we need to reroute the tree structure without changing the parent
        DAG::NodePointer childNode = create_children<depth>(voxelPos);
        if constexpr (depth == 1) {
            // at root depth its easy, because the root always has all 8 children allocated

            // add new child to mask
            parentLevel.data[parent] = (uint32_t)(childMask | childBit);
            // add child to corresponding position
            uint32_t childOffset = 0;
            // there are some cpu-/compiler-specific intrinsics for this
            // the goal is to get the position of the bit
            // this switch tree is very predictable, compiler might assist
            switch (childBit) {
                case   1: childOffset = 1; break;
                case   2: childOffset = 2; break;
                case   4: childOffset = 3; break;
                case   8: childOffset = 4; break;
                case  16: childOffset = 5; break;
                case  32: childOffset = 6; break;
                case  64: childOffset = 7; break;
                case 128: childOffset = 8; break;
            }
            parentLevel.data[parent + childOffset] = childNode;
        }
        else {
            // at all other depths, the parent's parent (parentParent) needs to be accessed
            std::cout << "NOT YET IMPLEMENTED\n";

            // 1. create new node with parent's content + childNode
            // 2. in parentParent children: replace pointer to parent with new parent
        }
        return;
    }
    template<int32_t depth> inline DAG::NodePointer create_children(Eigen::Vector3i& voxelPos) {
        std::cout << "creating children from leaf to depth " << depth << '\n';

        // create children all the way from leaves up until the <depth> given via template
        DAG::NodePointer lastNode = 0;
        [&]<size_t... indices>(std::index_sequence<indices...>) {
            // keep track of last node
            lastNode = create_node_leaf(voxelPos);
            ((lastNode = create_node_generic<nDagLevels - indices - 2>(voxelPos, lastNode)), ...);
        } (std::make_index_sequence<nDagLevels - depth - 1>{});
        return lastNode;
    }
    template<int32_t depth> inline DAG::NodePointer create_node_generic(Eigen::Vector3i& voxelPos, DAG::NodePointer child) {
        DAG::Level& level = dagLevels[depth];
        // get mask bit for the child at one depth below
        DAG::ChildMask childBit = get_child_bit<depth>(voxelPos);

        // create a temporary node and only progress tracker if it was actually inserted
        DAG::NodePointer key = level.dataSize;
        level.data.insert(level.data.begin() + key, {
            (uint32_t)childBit, // mask
            (uint32_t)child     // child pointer
        });

        // std::cout << depth << " depth. bit: " << (uint32_t)childBit << '\n';

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