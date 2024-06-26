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
        typedef std::make_signed_t<ClusterT> ClusterS;
        typedef uint32_t PartT;
        LeafCluster(ClusterT cluster): cluster(cluster) {}
        LeafCluster(PartT part0, PartT part1): cluster((ClusterT)part0 | ((ClusterT)part1 << 32)) {}
        LeafCluster(std::array<float, 8>& leaves): cluster(0) {
            for (ClusterT i = 0; i < 8; i++) {
                // normalize sd to [-1, 1]
                float sdNormalized = leaves[i] * (1.0 / maxDist);
                
                // scale up to fit into nBit integers
                float scale = (float)range;
                float sdScaled = sdNormalized * scale;
                
                // cast to 8-bit integer and clamp between given range
                int8_t sdScaledInt = (int8_t)sdScaled;
                sdScaledInt = std::clamp<int8_t>(sdScaledInt, -scale, scale);
                
                // add offset such that values are represented linearly from 0 to max
                uint8_t sdScaledUint = (uint8_t)(sdScaledInt + (int8_t)scale);
                
                // pack the 4 bits of this value into the leaf cluster
                cluster |= (ClusterT)sdScaledUint << i*nBits;
            }
        }
        std::pair<PartT, PartT> get_parts() {
            PartT part0 = (PartT)cluster;
            PartT part1 = (PartT)(cluster >> 32);
            return { part0, part1 };
        }
        void merge(LeafCluster& other) {
            for (ClusterT i = 0; i < 8; i++) {
                // mask out bits for current leaf
                typedef std::make_signed_t<ClusterT> ClusterInt;
                int8_t maskedA = (this->cluster >> i*nBits) & leafMask;
                int8_t maskedB = (other.cluster >> i*nBits) & leafMask;
                
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
                    ClusterT submask = leafMask << i*nBits;
                    submask = ~submask; // flip
                    // overwrite result bits with new value
                    cluster &= submask;
                    cluster |= (ClusterT)maskedB << i*nBits;
                }
            }
        }
        float get_sd(uint8_t index) {
            // 4 bits precision for each leaf
            int8_t leaf = cluster >> index*nBits;
            leaf &= leafMask;
            // convert back to standard signed
            leaf -= (int8_t)range;
            // convert to floating signed distance
            float signedDistance = (float)leaf;
            signedDistance /= (float)range; // normalize signed distance
            signedDistance *= leafResolution; // scale signed distance to real size
            return signedDistance;
        }
        
        ClusterT cluster;
        static constexpr float maxDist = leafResolution;
        static constexpr ClusterT nBits = 8; // 1b sign, rest data
        static constexpr ClusterT leafMask = (1 << nBits) - 1; // mask for a single leaf
        static constexpr ClusterT range = leafMask / 2; // achievable range with data bits
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
