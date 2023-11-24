#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace DAG {
    typedef uint32_t NodePointer; // position of node in a level's data vector
    typedef uint32_t LeafMask; // contains 8 leaves, each of which are 4 bits
    typedef uint8_t ChildMask; // contains 8 children

    // 3D position of a root, packed into 64 bits
    typedef uint64_t RootPos;
    static constexpr uint64_t xRootBits = 24;
    static constexpr uint64_t yRootBits = 24;
    static constexpr uint64_t zRootBits = 16;
    static_assert(xRootBits + yRootBits + zRootBits <= sizeof(RootPos) * 8);

    // allows use of node as hash key
    struct KeyNode {
        inline size_t operator()(const KeyNode& key) const { // do we really need the parameter variable here?

            // the header will contain the child mask
            ChildMask childMask = static_cast<ChildMask>(data[p]);
            uint8_t nSetBits = std::popcount(childMask); // one bit per child

            // include all child pointers in the final hash
            size_t hash = 0;
            for (uint8_t i = 0; i < nSetBits; i++) {
                NodePointer iNP = static_cast<NodePointer>(i);
                hash = phmap::HashState().combine(hash, data[p + iNP + 1]);
            }
            return hash;
        }
        inline bool operator==(const KeyNode& key) const {
            
            // the header will contain the child mask
            ChildMask childMask = static_cast<ChildMask>(data[p]);
            uint8_t nSetBits = std::popcount(childMask); // one bit per child

            // compare header and pointers with other key
            for (uint8_t i = 0; i <= nSetBits; i++) {
                NodePointer iNP = static_cast<NodePointer>(i);
                if (data[p + iNP] != key.data[key.p + iNP]) return false;
            }
            // only return true when previous checks were passed
            return true;
        }

        std::vector<uint32_t>& data; // array that contains this node
        NodePointer p; // points to node in array
    };

    struct Level {
        // hashmap containing pointer into data vector
        phmap::flat_hash_map<uint64_t, NodePointer> pointerMap;
        std::vector<uint32_t> data; // could extract data
    };
};

// helper for integer vectors 
static void print_vec3i(Eigen::Vector3i pos, std::string text) {
    ROS_INFO_STREAM(text << " (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")");
}
// helper for floating vectors
static void print_vec3f(Eigen::Vector3f pos, std::string text) {
    ROS_INFO_STREAM(text << " (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")");
}
