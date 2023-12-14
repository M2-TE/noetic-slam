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
        // set up cache (first 9 values in data array)
        for (size_t i = 0; i < dagLevels.size(); i++) {
            DAG::Level& level = dagLevels[i];

            DAG::HashFunctor hashFnc = { &level.data };
            DAG::CompFunctor compFnc = { &level.data };
            level.pointerSet = decltype(level.pointerSet)(0, hashFnc, compFnc);
            level.data.resize(9, 0); // cache
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
        Eigen::Vector3i voxelRootPos = voxelPos / rootDimSize;
        Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int32_t x){ return x % rootDimSize; });

        // some debugging output for now
        print_vec3(realPos, "> Inserting new point with position");
        print_vec3(voxelPos, "Voxel position");
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
            static constexpr int32_t nNodes = nDagLevels - 1;
            DAG::NodePointer lastNode = 0;

            // create all parent nodes (except roots)
            [&]<size_t... indices>(std::index_sequence<indices...>) {
                ((lastNode = create_node<nNodes - indices>(voxelPos, lastNode)), ...);
            } (std::make_index_sequence<nNodes>{});

            // TODO: create root here
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
    inline uint8_t get_child_mask_index(Eigen::Vector3i localPos, int32_t parentDepth) {
        // check which child octant the voxel falls into

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
    
    template<int32_t depth> // depth as index into the DAG tree level
    inline DAG::NodePointer create_node(Eigen::Vector3i& voxelPos, DAG::NodePointer child) {
        static constexpr int32_t reverseDepth = nDagLevels - depth;
        static constexpr int32_t dimSize = 1 << reverseDepth;
        std::cout << "Depth: " << depth << " Size: " << dimSize << 'x' << dimSize << 'x' << dimSize << std::endl;

        // calculate local position within current node (each node has 8 children, so 2x2x2)
        const Eigen::Vector3i localPos = voxelPos.unaryExpr([](const int32_t x){ return x % dimSize / (dimSize / 2); });
        print_vec3(localPos, "node subbpos");

        DAG::ChildMask childBit = 0;
        childBit |= localPos.x() << 0;
        childBit |= localPos.y() << 1;
        childBit |= localPos.z() << 2;
        // ROS_INFO_STREAM("child bit: " << static_cast<uint32_t>(childBit));
        DAG::Level level = dagLevels[depth];

        // if its a node containing leaves
        if constexpr (depth == nDagLevels - 1) {
            float sd = voxelToCoordRatio / 2.0f; // signed distance (TODO: calc, its some random value rn)

            float sdMaxRecip = 1.0f / voxelToCoordRatio; // voxel resolution as max distance (taking reciprocal)
            float sdNorm = sd * sdMaxRecip; // normalize signed distance between 0 and 1
            sd = sdNorm * static_cast<float>(DAG::maxSignedDistance); // scale to n-bit uint
            // std::cout << "scaled sdNorm: " << sd << std::endl;

            // cast to uint and shift into position within leaves node
            // each leaf will take up DAG::nBitsPerLeaf bits
            DAG::Leaves leaves = static_cast<DAG::Leaves>(sd) << (childBit * DAG::nBitsPerLeaf);
            DAG::NodePointer leavesPointer;
            
            // check if these leaves already exist
            if (dagLeafMap.contains(leaves)) {
                leavesPointer = dagLeafMap[leaves];
                std::cout << "identical leaf found: " << leavesPointer << std::endl;
            }
            else {
                // add leaf node to DAG level
                leavesPointer = level.data.size();
                dagLeafMap[leaves] = leavesPointer;
                level.data.push_back(leaves);
                std::cout << "created new leaf: " << leavesPointer << std::endl;
            }

            return leavesPointer;
        }
        else {
            // figure out which voxel the child occupies
            DAG::ChildMask childMask = childBit;

            // write current node information to cache
            // hashes and comparisons are independant of actual node position
            level.data[0] = childMask;
            level.data[1] = child;

            // start of data array is the cache
            size_t hash = level.pointerSet.hash(0);
            auto pNode = level.pointerSet.find(0, hash);

            // check if this node already exists
            DAG::NodePointer nodePointer;
            if (pNode != level.pointerSet.end()) {
                nodePointer = *pNode;
                std::cout << "identical node found: " << nodePointer << std::endl;
            }
            else {
                // insert the new node pointer
                nodePointer = level.data.size();
                level.pointerSet.emplace_with_hash(nodePointer, hash);
                // insert the new node data
                level.data.insert(level.data.end(), {
                    childMask,
                    child
                });
                std::cout << "created new node: " << nodePointer << std::endl;
            }
            return nodePointer;
        }
    }

private:
    phmap::flat_hash_map<DAG::RootPos, DAG::NodePointer> dagRootMap;
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};