#pragma once

#include <cstdint>
#include <array>

#include <Eigen/Dense>
#include <parallel_hashmap/phmap.h>

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
    inline void insert_point(Eigen::Vector3f realPos) {

        // figure out which root the voxel will be in, as well as its position local to that root
        Eigen::Vector3i voxelPos = (realPos * coordToVoxelRatio).cast<int32_t>();
        Eigen::Vector3f voxelPosF = voxelPos.cast<float>() * voxelToCoordRatio;
        Eigen::Vector3i voxelRootPos = voxelPos / rootDimSize;
        Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int32_t x){ return x % rootDimSize; });

        // some debugging output for now
        print_vec3(realPos, "> Inserting new point with position");
        print_vec3(voxelPos, "Voxel position   (int)");
        print_vec3(voxelPosF, "Voxel position (float)");
        print_vec3(voxelRootPos, "Root  position");
        print_vec3(localPos, "Local position within root is");

        // main insertion
        DAG::RootPos rootPos = pack_root_pos(voxelRootPos);
        if (dagRootMap.contains(rootPos)) {
            std::cout << "Root found\n";
            DAG::NodePointer rootNode = dagRootMap[rootPos];

            // go and get all the child nodes recursively until a node doesnt exist
            for (uint32_t i = 0; i < nDagLevels; i++) {
                // TODO
            }
        }
        else {
            std::cout << "Root missing\n";

            // create all nodes starting from the leaves
            [&]<size_t... indices>(std::index_sequence<indices...>) {
                // keep track of last node
                DAG::NodePointer lastNode = 0;
                lastNode = create_node_leaf<nDagLevels - 1>(voxelPos);
                ((lastNode = create_node_generic<nDagLevels - indices - 2>(voxelPos, lastNode)), ...);
                lastNode = create_node_root<0>(voxelPos, lastNode, rootPos);
            } (std::make_index_sequence<nDagLevels - 2>{});
        }
    }

private:
    inline DAG::RootPos pack_root_pos(Eigen::Vector3i voxelRootPos) {
        DAG::RootPos rootPos;
        rootPos |= (DAG::RootPos)voxelRootPos.x();
        rootPos |= (DAG::RootPos)voxelRootPos.y() << (DAG::xRootBits);
        rootPos |= (DAG::RootPos)voxelRootPos.z() << (DAG::xRootBits + DAG::yRootBits);
        return rootPos;
    }
    
    template<int32_t depth> inline DAG::ChildMask get_child_bit(Eigen::Vector3i& voxelPos) {
        static constexpr int32_t reverseDepth = nDagLevels - depth;
        static constexpr int32_t dimSize = 1 << reverseDepth;
        std::cout << "Depth: " << depth << " Size: " << dimSize << 'x' << dimSize << 'x' << dimSize << std::endl;

        // calculate local position within current node (each node has 8 children, so 2x2x2)
        const Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int32_t x){ return x % dimSize / (dimSize / 2); });
        // print_vec3(localPos, "node subbpos");

        DAG::ChildMask childBit = 1;
        childBit = childBit << (localPos.x() << 0);
        childBit = childBit << (localPos.y() << 1);
        childBit = childBit << (localPos.z() << 2);
        return childBit;
    }
    template<int32_t depth> inline DAG::NodePointer create_node_leaf(Eigen::Vector3i& voxelPos) { // TODO: calc signed distance properly. also dont need childBit, fill all leaves at once
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
    template<int32_t depth> inline DAG::NodePointer create_node_generic(Eigen::Vector3i& voxelPos, DAG::NodePointer child) {
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
    template<int32_t depth> inline DAG::NodePointer create_node_root(Eigen::Vector3i& voxelPos, DAG::NodePointer child, DAG::RootPos rootPos) {
        DAG::ChildMask childBit = get_child_bit<depth>(voxelPos);
        DAG::Level& level = dagLevels[depth];

        // check if a new node is emplaced
        auto [pRoot, bEmplacedNew] = dagRootMap.emplace(rootPos, level.data.size());
        if (bEmplacedNew) {
            // always allocate all 8 children for a root
            std::array<DAG::NodePointer, DAG::nLeavesPerNode + 1> children;
            children[0] = childBit;
            children[1] = child;
            level.data.insert(level.data.end(), children.begin(), children.end());
        }
        return pRoot->second;
    }
private:
    phmap::flat_hash_map<DAG::RootPos, DAG::NodePointer> dagRootMap;
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};