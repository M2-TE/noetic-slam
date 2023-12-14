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

        DAG::ChildMask childBit = 1;
        childBit = childBit << (localPos.x() << 0);
        childBit = childBit << (localPos.y() << 1);
        childBit = childBit << (localPos.z() << 2);
        // ROS_INFO_STREAM("child bit: " << static_cast<uint32_t>(childBit));
        DAG::Level& level = dagLevels[depth];

        // if its a node containing leaves
        if constexpr (depth == nDagLevels - 1) return create_node_leaf(level, childBit);
        else return create_node_generic(level, child, childBit);
    }
    inline DAG::NodePointer create_node_leaf(DAG::Level& level, DAG::ChildMask childBit) { // TODO: calc signed distance properly
        float sd = voxelToCoordRatio / 2.0f; // signed distance TODO: calc

        float sdMaxRecip = 1.0f / voxelToCoordRatio; // voxel resolution as max distance (taking reciprocal)
        float sdNorm = sd * sdMaxRecip; // normalize signed distance between 0 and 1
        sd = sdNorm * static_cast<float>(DAG::maxSignedDistance); // scale to n-bit uint

        // cast to uint and shift into position within leaves node
        // each leaf will take up DAG::nBitsPerLeaf bits
        DAG::Leaves leaves = static_cast<DAG::Leaves>(sd) << (childBit * DAG::nBitsPerLeaf);
        // create a temporary node and only progress tracker if it was actually inserted
        DAG::NodePointer key = level.dataSize;
        level.data.insert(level.data.begin() + key, {
            leaves
        });

        // check if a new node is emplaced
        auto [pLeaf, bEmplacedNew] = dagLeafMap.emplace(leaves, key);
        if (bEmplacedNew) level.dataSize += 1;
        else std::cout << "leaf already exists" << std::endl;
        return pLeaf->second;
    }
    inline DAG::NodePointer create_node_generic(DAG::Level& level, DAG::NodePointer child, DAG::ChildMask childBit) {
        // create a temporary node and only progress tracker if it was actually inserted
        DAG::NodePointer key = level.dataSize;
        level.data.insert(level.data.begin() + key, {
            childBit,   // mask
            child       // child pointer
        });

        // check if a new node is emplaced
        auto [pNode, bEmplacedNew] = level.pointerSet.emplace(key);
        if (bEmplacedNew) level.dataSize += 2;
        else std::cout << "node already exists" << std::endl;
        return *pNode;
    }
private:
    phmap::flat_hash_map<DAG::RootPos, DAG::NodePointer> dagRootMap;
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};