#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <parallel_hashmap/phmap.h>

namespace DAG {
    typedef uint32_t NodePointer; // position of node in a level's data vector
    typedef uint8_t ChildMask; // contains 8 children

    // contains 8 leaves, each of which are 4 bits
    typedef uint32_t Leaves;
    static constexpr uint32_t nBitsPerLeaf = 4;
    static constexpr uint32_t nLeavesPerNode = 8;
    static constexpr uint32_t maxSignedDistance = (1 << nBitsPerLeaf) - 1;
    static_assert(nBitsPerLeaf * nLeavesPerNode <= sizeof(Leaves) * 8);

    // 3D position of a root, packed into 64 bits
    typedef uint64_t RootPos;
    static constexpr uint64_t xRootBits = 24;
    static constexpr uint64_t yRootBits = 24;
    static constexpr uint64_t zRootBits = 16;
    static_assert(xRootBits + yRootBits + zRootBits <= sizeof(RootPos) * 8);

    struct HashFunctor {
        inline size_t operator()(NodePointer key) const noexcept {
            std::vector<uint32_t>& data = *pData;
            ChildMask childMask = static_cast<ChildMask>(data[key]);
            uint8_t nChildren = std::popcount(childMask);
            // hash child mask
            size_t hash = phmap::HashState::combine(0, childMask);
            // hash all children
            for (uint8_t i = 0; i < nChildren; i++) {
                hash = phmap::HashState::combine(hash, data[++key]);
            }
            // std::cout << hash << std::endl;
            return hash;
        }
        std::vector<uint32_t>* pData; // pointer to raw data array
    };
    struct CompFunctor {
        inline bool operator()(NodePointer keyA, NodePointer keyB) const noexcept {
            std::vector<uint32_t>& data = *pData;
            // compare all children
            ChildMask childMaskA = static_cast<ChildMask>(data[keyA]);
            std::memcmp(&data[keyA + 1], &data[keyB + 1], std::popcount(childMaskA));
            return true;
        }
        std::vector<uint32_t>* pData; // pointer to raw data array
    };

    struct Level {
        // hash set of pointers to nodes
        phmap::flat_hash_set<NodePointer, HashFunctor, CompFunctor> pointerSet;
        // storage of raw node data
        std::vector<uint32_t> data;
        // size tracker for data vector (excluding temp allocations)
        size_t dataSize = 0;
    };
};

// helper for vectors 
static void print_vec3(Eigen::Vector3i pos, std::string text) {
    ROS_INFO_STREAM(text << " (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")");
}
static void print_vec3(Eigen::Vector3f pos, std::string text) {
    ROS_INFO_STREAM(text << " (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")");
}
