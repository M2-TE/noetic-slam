#pragma once
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
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

    struct HashFunctor {
        inline size_t operator()(NodeIndex key) const noexcept {
            std::vector<uint32_t>& data = *pData;
            // count children
            uint8_t nChildren = std::popcount<uint8_t>(data[key]);
            // hash entire node
            size_t hash = 0;
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
            uint8_t nChildren = std::popcount<uint8_t>(data[keyB]);
            // compare entire node
            int cmp = std::memcmp(
                &data[keyA],
                &data[keyB],
                nChildren * sizeof(uint32_t) + sizeof(uint32_t));
            return cmp == 0;
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
        size_t dataSize;
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
