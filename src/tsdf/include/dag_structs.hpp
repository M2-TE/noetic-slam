#pragma once
#include <bitset>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <vector>
//
#include <parallel_hashmap/phmap.h>
#include <morton-nd/mortonND_BMI2.h>
#include <Eigen/Eigen>
//
#include "dag_constants.hpp"

namespace DAG {
    // Morton Code stuff
    static constexpr size_t mortonVolume = idx::leaf - 4;
    struct MortonCode {
        MortonCode(int x, int y, int z): MortonCode(Eigen::Vector3i(x, y, z)) {}
        MortonCode(uint64_t code): val(code) {}
        MortonCode(Eigen::Vector3i vec) {
            // truncate from two's complement 32-bit to 21-bit integer
            uint32_t x, y, z;
            x = (1 << 20) + (uint32_t)vec.x();
            y = (1 << 20) + (uint32_t)vec.y();
            z = (1 << 20) + (uint32_t)vec.z();
            val = mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);
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
        inline size_t hash() {
            return val;
        }
        uint64_t val;
    };
    
    struct LeafCluster {
        typedef uint64_t ClusterT;
        LeafCluster(ClusterT other): cluster(other) {}
        LeafCluster(std::array<float, 8>& leaves) {
            for (ClusterT i = 0; i < 8; i++) {
                // normalize sd to fit maxDist
                float sdNormalized = leaves[i] * (1.0 / maxDist);
                
                // scale up to fit into nBit integers
                float scale = (float)range;
                float sdScaled = sdNormalized * scale;
                
                // cast to 8-bit integer and clamp between given range
                int8_t sdScaledInt = (int8_t)sdScaled;
                sdScaledInt = std::clamp<int8_t>(sdScaledInt, -scale, scale);
                
                // add offset such that values are represented linearly from 0 to max
                // 0 = -range, 14 = +range (both of these should be seen as "too far away" and discarded)
                uint8_t sdScaledUint = (uint8_t)(sdScaledInt + (int8_t)scale);
                
                // pack the 4 bits of this value into the leaf cluster
                cluster |= (ClusterT)sdScaledUint << i*nBits;
            }
        }
        void merge(LeafCluster& other) {
            for (ClusterT i = 0; i < 8; i++) {
                // mask out bits for current leaf
                constexpr ClusterT mask = 0b1111;
                int8_t maskedA = (this->cluster >> i*nBits) & mask;
                int8_t maskedB = (other.cluster >> i*nBits) & mask;
                
                // convert back to standard readable int
                int8_t a = maskedA - range;
                int8_t b = maskedB - range;
                
                // ruleset (in order of priority):
                // 1. positive signed distance takes precedence over negative
                // 2. smaller value takes precedence over larger value
                bool bOverwrite = false;
                if (std::signbit(a) > std::signbit(b)) bOverwrite = true;
                else if (std::signbit(a) == std::signbit(b) && std::abs(a) > std::abs(b)) bOverwrite = true;
                
                if (bOverwrite) {
                    // mask out the relevant bits
                    ClusterT submask = mask << i*nBits;
                    submask = ~submask; // flip
                    // overwrite result bits with new value
                    cluster &= submask;
                    cluster |= (ClusterT)maskedB << i*nBits;
                }
            }
        }
        float get_sd(uint8_t index) {
            // 4 bits precision for each leaf
            ClusterT leaf = cluster >> index*nBits;
            leaf &= 0b1111;
            // convert back to standard signed
            leaf -= (std::make_signed_t<ClusterT>)range;
            // convert to floating signed distance
            float signedDistance = (float)leaf;
            signedDistance /= range; // normalize signed distance (sorta)
            signedDistance *= leafResolution; // scale signed distance to real size
            return signedDistance;
        }
        
        ClusterT cluster;
        static constexpr float maxDist = leafResolution;
        static constexpr ClusterT nBits = 4; // 1b sign, rest data
        static constexpr ClusterT range = (1 << (nBits-1)) - 1; // achievable range with data bits
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
