#pragma once
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
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
            // TODO: this can be massively simplified using simple addition (see hashgrid impl)
            
            // from two's completent to simple uint21_t
            Eigen::Matrix<uint32_t, 3, 1> res = vec.cast<uint32_t>();
            res = res.unaryExpr([](uint32_t i) {
                constexpr uint32_t mask = (1 << 20) - 1;
                uint32_t val = i + (1u << 31u);
                uint32_t sign = val & (1u << 31u);
                val &= mask; // mask out the first 20 bits
                val |= sign >> 11; // shift sign to bit 21
                return val;
            });
            // other method:
            // Eigen::Matrix<int32_t, 3, 1> res2 = vec.unaryExpr([](const int32_t i) {
            //     constexpr uint signBit = 1 << 31;
            //     constexpr uint signMask = signBit - 1;
            //     constexpr uint mask = (1 << 20) - 1;
            //     // invert sign
            //     uint32_t sign = (i & signBit) ^ signBit;
            //     // shift sign to 21st bit
            //     sign = sign >> 11;
            //     // combine sign with i
            //     uint32_t res = (i & mask) | sign;
            //     return (int32_t)res;
            // });

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
        inline size_t hash() {
            return val;
        }
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
    private:
        struct HashFunctor {
            inline size_t operator()(uint32_t key) const noexcept {
                const std::vector<uint32_t>& data = *pData;
                size_t hash = phmap::HashState::combine(0,
                    data[key + 0], data[key + 1], data[key + 2], data[key + 3],
                    data[key + 4], data[key + 5], data[key + 6], data[key + 7]);
                return hash;
            }
            std::vector<uint32_t>* pData; // non-owning pointer to raw data array
        };
        struct CompFunctor {
            inline bool operator()(uint32_t keyA, uint32_t keyB) const noexcept {
                const std::vector<uint32_t>& data = *pData;
                // compare entire node
                int cmp = std::memcmp(
                    &data[keyA],
                    &data[keyB],
                    8 * sizeof(uint32_t));
                return cmp == 0;
            }
            std::vector<uint32_t>* pData; // non-owning pointer to raw data array
        };
    public: // leaf nodes have no child mask
        LeafLevel(): hashSet(0, HashFunctor(&data), CompFunctor(&data)), data(1), nOccupied(1) {}
        phmap::parallel_flat_hash_set<uint32_t, HashFunctor, CompFunctor> hashSet;
        std::vector<uint32_t> data;
        uint32_t nOccupied;
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
