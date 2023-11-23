#pragma once

namespace DAG {
    typedef uint32_t* PointerRaw; // raw pointer to the actual memory location of a node (dangerous, might face invalidation!)
    typedef uint32_t Pointer; // position of node in a level's data vector
    typedef uint32_t LeafMask; // contains 8 leaves, each of which are 4 bits
    typedef uint8_t ChildMask; // contains 8 children

    // 3D position of a root, packed into 64 bits
    typedef uint64_t RootPos;
    static constexpr uint64_t xRootBits = 24;
    static constexpr uint64_t yRootBits = 24;
    static constexpr uint64_t zRootBits = 16;
    static_assert(xRootBits + yRootBits + zRootBits <= sizeof(RootPos) * 8);

    struct Level {
        // hashmap containing pointer into data vector
        phmap::flat_hash_map<uint64_t, Pointer> pointerMap;
        std::vector<uint32_t> data;
    };

    struct HashKey {
        inline size_t operator()(const HashKey& key) const {

            // the header will contain the child mask
            ChildMask childMask = static_cast<ChildMask>(key.p[0]);
            // its bits indicate how many child pointer will follow
            uint8_t nSetBits = std::popcount(childMask);

            // include all child pointers in the final hash
            if constexpr (bUseLoop) {
                size_t hash = 0;
                for (uint8_t i = 0; i < nSetBits; i++) {
                    hash = phmap::HashState().combine(hash, p[i + 1]);
                }
                return hash;
            }
            else {
                switch (nSetBits) {
                    // default:
                    // case 0: return phmap::HashState().combine(0);
                    case 1: return phmap::HashState().combine(0, p[1]);
                    case 2: return phmap::HashState().combine(0, p[1], p[2]);
                    case 3: return phmap::HashState().combine(0, p[1], p[2], p[3]);
                    case 4: return phmap::HashState().combine(0, p[1], p[2], p[3], p[4]);
                    case 5: return phmap::HashState().combine(0, p[1], p[2], p[3], p[4], p[5]);
                    case 6: return phmap::HashState().combine(0, p[1], p[2], p[3], p[4], p[5], p[6]);
                    case 7: return phmap::HashState().combine(0, p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
                    case 8: return phmap::HashState().combine(0, p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]);
                }
            }

        }
        inline bool operator==(const HashKey& key) const {
            
            // the header will contain the child mask
            ChildMask childMask = static_cast<ChildMask>(p[0]);
            // its bits indicate how many child pointer will follow
            uint8_t nSetBits = std::popcount(childMask);

            // compare header and pointers with other key
            if constexpr (bUseLoop) {
                for (uint8_t i = 0; i <= nSetBits; i++) {
                    if (p[i] != key.p[i]) return false;
                }
                // only return true when previous checks were passed
                return true;
            }
            else {
                switch (nSetBits) {
                    // default:
                    // case 0: return (p[0] == key.p[0]);
                    case 1: return (p[0] == key.p[0]) && (p[1] == key.p[1]);
                    case 2: return (p[0] == key.p[0]) && (p[1] == key.p[1]) && (p[1] == key.p[1]);
                    case 3: return (p[0] == key.p[0]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]);
                    case 4: return (p[0] == key.p[0]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]);
                    case 5: return (p[0] == key.p[0]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]);
                    case 6: return (p[0] == key.p[0]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]);
                    case 7: return (p[0] == key.p[0]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]);
                    case 8: return (p[0] == key.p[0]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]) && (p[1] == key.p[1]);
                }
            }
        }

        PointerRaw p; // points to node header in memory
    private:
        // TODO: compare performance of the two variants
        static constexpr bool bUseLoop = false;
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
