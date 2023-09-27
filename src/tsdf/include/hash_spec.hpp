#pragma once
#include <parallel_hashmap/phmap_utils.h>

// inject hashing specialization
namespace std {
    template<> struct hash<std::array<uint32_t, 8>> {
        inline std::size_t operator()(const std::array<uint32_t, 8>& key) const noexcept {
            return phmap::HashState().combine(0, 
                key[0], key[1], key[2], key[3],
                key[4], key[5], key[6], key[7]);
        }
    };
}