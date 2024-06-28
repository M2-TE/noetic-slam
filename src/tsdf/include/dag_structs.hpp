#pragma once
#include <algorithm>
#include <bitset>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <type_traits>
#include <vector>
//
#include <parallel_hashmap/phmap.h>
#include <morton-nd/mortonND_BMI2.h>
#include <Eigen/Eigen>
//
#include "dag_constants.hpp"
#include "leaf_cluster.hpp"

namespace DAG {
    struct MortonCode {
        MortonCode(int x, int y, int z): MortonCode(Eigen::Vector3i(x, y, z)) {}
        MortonCode(uint64_t code): val(code) {}
        MortonCode(Eigen::Vector3i vec) {
            encode(vec);
        }
        inline void encode(Eigen::Vector3i vec) {
            // truncate from two's complement 32-bit to 21-bit integer
            uint32_t x, y, z;
            x = (1 << 20) + (uint32_t)vec.x();
            y = (1 << 20) + (uint32_t)vec.y();
            z = (1 << 20) + (uint32_t)vec.z();
            val = mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);
        }
        inline Eigen::Vector3i decode() const {
            auto [x, y, z] = mortonnd::MortonNDBmi_3D_64::Decode(val);
            x -= 1 << 20;
            y -= 1 << 20;
            z -= 1 << 20;
            return { (int32_t)x, (int32_t)y, (int32_t)z };
        }
        inline bool operator==(const MortonCode& other) const { return val == other.val; }
        inline bool operator<(const MortonCode& other) const { return val < other.val; }
        inline bool operator>(const MortonCode& other) const { return val > other.val; }
        uint64_t val;
    };
    

    struct NodeLevel {
    private:
        struct HashFunctor {
            inline size_t operator()(NodeIndex key) const noexcept {
                std::vector<uint32_t>& data = *pData;
                // count children
                // uint8_t nChildren = std::popcount<uint8_t>(data[key]);
                uint32_t nChildren = data[key] >> 8;
                // hash entire node
                size_t hash = 0;
                // switch(nChildren) {
                //     case 1: hash = phmap::HashState::combine(0, data[key + 0], data[key + 1]); break;
                //     case 2: hash = phmap::HashState::combine(0, data[key + 0], data[key + 1], data[key + 2]); break;
                //     case 3: hash = phmap::HashState::combine(0, data[key + 0], data[key + 1], data[key + 2], data[key + 3]); break;
                //     case 4: hash = phmap::HashState::combine(0, data[key + 0], data[key + 1], data[key + 2], data[key + 3], data[key + 4]); break;
                //     case 5: hash = phmap::HashState::combine(0, data[key + 0], data[key + 1], data[key + 2], data[key + 3], data[key + 4], data[key + 5]); break;
                //     case 6: hash = phmap::HashState::combine(0, data[key + 0], data[key + 1], data[key + 2], data[key + 3], data[key + 4], data[key + 5], data[key + 6]); break;
                //     case 7: hash = phmap::HashState::combine(0, data[key + 0], data[key + 1], data[key + 2], data[key + 3], data[key + 4], data[key + 5], data[key + 6], data[key + 7]); break;
                //     case 8: hash = phmap::HashState::combine(0, data[key + 0], data[key + 1], data[key + 2], data[key + 3], data[key + 4], data[key + 5], data[key + 6], data[key + 7], data[key + 8]); break;
                // }
                for (uint8_t i = 0; i <= nChildren; i++) {
                    hash = phmap::HashState::combine(hash, data[key + i]);
                }
                return hash;
            }
            std::vector<uint32_t>* pData; // non-owning pointer to raw data array
        };
        struct CompFunctor {
            inline bool operator()(NodeIndex keyA, NodeIndex keyB) const noexcept {
                std::vector<uint32_t>& data = *pData;
                // count children
                // uint8_t nChildren = std::popcount<uint8_t>(data[keyB]);
                uint32_t nChildren = data[keyB] >> 8;
                // compare entire node
                int cmp = std::memcmp(
                    &data[keyA],
                    &data[keyB],
                    nChildren * sizeof(uint32_t) + sizeof(uint32_t));
                return cmp == 0;
            }
            std::vector<uint32_t>* pData; // non-owning pointer to raw data array
        };
    public:
        NodeLevel(): hashSet(0, HashFunctor(&data), CompFunctor(&data)), data(1), nOccupied(1) {}
        phmap::parallel_flat_hash_set<NodeIndex, HashFunctor, CompFunctor> hashSet;
        std::vector<uint32_t> data;
        uint32_t nOccupied;
    };
    struct LeafLevel {
        typedef uint32_t LeafIndex; // index into data array
        LeafLevel(): hashMap(), data(1, 0) {}
        phmap::parallel_flat_hash_map<LeafCluster::ClusterT, LeafIndex> hashMap;
        std::vector<uint32_t> data;
    };
};

namespace std {
    template<>
    struct hash<DAG::MortonCode> {
        inline size_t operator()(const DAG::MortonCode& x) const {
            return x.val;
        }
    };
}
