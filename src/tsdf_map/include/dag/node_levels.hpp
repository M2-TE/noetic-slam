#pragma once
//
#include <cstdint>
//
#include <Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
//
#include "dag/leaf_cluster.hpp"

struct NodeLevel {
private:
    struct HashFunctor {
        HashFunctor(std::vector<uint32_t>* pData): pData(pData) {}
        inline size_t operator()(uint32_t key) const noexcept {
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
        CompFunctor(std::vector<uint32_t>* pData): pData(pData) {}
        inline bool operator()(uint32_t keyA, uint32_t keyB) const noexcept {
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
    phmap::parallel_flat_hash_set<uint32_t, HashFunctor, CompFunctor> hashSet;
    std::vector<uint32_t> data;
    uint32_t nOccupied;
};
struct LeafLevel {
    typedef uint32_t LeafIndex; // index into data array
    LeafLevel(): hashMap(), data(1, 0) {}
    phmap::parallel_flat_hash_map<LeafCluster::ClusterT, LeafIndex> hashMap;
    std::vector<uint32_t> data;
};