#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>
#include <parallel_hashmap/phmap.h>
#include <morton-nd/mortonND_BMI2.h>
#include <Eigen/Eigen>

namespace DAG {
    typedef uint32_t NodeIndex; // position of node in a level's data vector
    typedef uint32_t ChildMask; // contains 8 children
    typedef uint32_t NodeData; // contains a mask or child as raw undefined data

    struct MortonIndex { Eigen::Vector3f point; uint32_t index; };
    struct Pose { Eigen::Vector3f pos; Eigen::Quaternionf rot; };
    struct Scan { Pose pose; std::vector<NodeIndex> roots; };

    static constexpr double leafResolution = 0.01; // real distance for each voxel step
    static constexpr uint32_t nDagLevels = 10; // number of DAG levels including root and leaf clusters (excluding leaves)
    static constexpr uint32_t nAllLevels = nDagLevels + 1; // nDagLevels including implicit leaf depth
    static constexpr std::array<uint32_t, nAllLevels> get_sizes() {
        std::array<uint32_t, nAllLevels> arr = {};
        uint32_t current = 1;
        for (size_t i = nDagLevels; i > 0; i--) {
            arr[i] = current;
            current *= 2;
        }
        arr[0] = current;
        return arr;
    }
    static constexpr std::array<double, nAllLevels> get_resolutions() {
        std::array<double, nAllLevels> arr = {};
        double current = leafResolution;
        for (size_t i = nDagLevels; i > 0; i--) {
            arr[i] = current;
            current *= 2.0;
        }
        arr[0] = current;
        return arr;
    }
    static constexpr std::array<uint32_t, nAllLevels> dagSizes = get_sizes();
    static constexpr std::array<double, nAllLevels> dagResolutions = get_resolutions();
    namespace idx { // static indices
        static constexpr size_t root = 0;
        static constexpr size_t leaf = nDagLevels;
        static constexpr size_t leafCluster = leaf - 1;
    }
    // Morton Code stuff
    static constexpr size_t mortonVolume = idx::leaf - 4;
    struct MortonCode {
        MortonCode(int x, int y, int z): MortonCode(Eigen::Vector3i(x, y, z)) {}
        MortonCode(uint64_t code): val(code) {}
        MortonCode(Eigen::Vector3i vec) {
            // use n-bit signed integral as morton code input with locality between -1, 0, +1
            auto res = vec.unaryExpr([](const int32_t i) {
                constexpr uint signBit = 1 << 31;
                constexpr uint signMask = signBit - 1;
                constexpr uint mask = (1 << 20) - 1;
                // invert sign
                uint32_t sign = (i & signBit) ^ signBit;
                // shift sign to 21st bit
                sign = sign >> 11;
                // combine sign with i
                uint32_t res = (i & mask) | sign;
                return (int32_t)res;
            });
            val = mortonnd::MortonNDBmi_3D_64::Encode(res.x(), res.y(), res.z());
        }
        inline bool operator==(const MortonCode& other) const {
            return val == other.val;
        }
        inline bool operator<(const MortonCode& other) const {
            return val < other.val;
        }
        inline bool operator>(const MortonCode& other) const {
            return val > other.val;
        }
        uint64_t val;
    };

    struct HashFunctor {
        inline size_t operator()(NodeIndex key) const noexcept {
            std::vector<uint32_t>& data = *pData;
            ChildMask childMask = data[key];
            uint8_t nChildren = std::popcount<uint8_t>(childMask);
            // hash child mask
            size_t hash = phmap::HashState::combine(0, childMask);
            // hash all children
            for (uint8_t i = 0; i < nChildren; i++) {
                hash = phmap::HashState::combine(hash, data[++key]);
            }
            return hash;
        }
        std::vector<uint32_t>* pData; // non-owning pointer to raw data array
    };
    struct CompFunctor {
        inline bool operator()(NodeIndex keyA, NodeIndex keyB) const noexcept {
            std::vector<uint32_t>& data = *pData;
            // compare all children
            ChildMask childMaskA = data[keyA];
            return std::memcmp(
                &data[keyA + 1],
                &data[keyB + 1],
                std::popcount<uint8_t>(childMaskA) * sizeof(uint32_t));
        }
        std::vector<uint32_t>* pData; // non-owning pointer to raw data array
    };

    // represents a node of theoretical max size in memory
    template<size_t nChildren = 8> struct Node {
        static inline Node* reinterpret(void* p) {
            return reinterpret_cast<Node*>(p);
        }
        inline uint8_t count_children() const {
            return std::popcount<uint8_t>(childMask);
        }
        ChildMask childMask = 0;
        std::array<NodeIndex, nChildren> children;
    };

    struct Level {
        Level(): pointers(0, HashFunctor(&data), CompFunctor(&data)), data(1), dataSize(1) {}
        phmap::parallel_flat_hash_set<NodeIndex, HashFunctor, CompFunctor> pointers;
        std::vector<NodeData> data;
        size_t dataSize = 0;
    };
};
