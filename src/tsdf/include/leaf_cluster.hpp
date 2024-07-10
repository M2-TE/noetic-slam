#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <type_traits>
//
#include "/root/repo/src/tsdf/include/dag_constants.hpp"
// #include "dag_structs.hpp"

namespace DAG
{
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
            sdNormalized = std::clamp(sdNormalized, -1.0f, 1.0f);
            
            // scale up to fit into nBit integers
            int32_t sdScaled = (int32_t)(sdNormalized * range);
            
            // add offset such that values are represented linearly from 0 to max
            uint8_t sdLinear = (uint8_t)(sdScaled + (int32_t)range);
            
            // pack the 4 bits of this value into the leaf cluster
            cluster |= (ClusterT)sdLinear << i*nBits;
        }
    }
    std::pair<PartT, PartT> get_parts() {
        PartT part0 = (PartT)cluster;
        PartT part1 = (PartT)(cluster >> 32);
        return { part0, part1 };
    }
    // merge by favoring positive numbers
    void merge_(LeafCluster& other) {
        for (ClusterT i = 0; i < 8; i++) {
            // mask out bits for current leaf
            typedef std::make_signed_t<ClusterT> ClusterInt;
            int32_t maskedA = (this->cluster >> i*nBits) & leafMask;
            int32_t maskedB = (other.cluster >> i*nBits) & leafMask;
            
            // convert back to standard readable int
            int32_t a = maskedA - range;
            int32_t b = maskedB - range;
            
            // ruleset (in order of priority):
            // 1. positive signed distance takes precedence over negative
            // 2. smaller value takes precedence over larger value
            bool bOverwrite = false;
            if (std::signbit(a) > std::signbit(b)) bOverwrite = true;
            else if (std::signbit(a) == std::signbit(b) && std::abs(a) > std::abs(b)) bOverwrite = true;
            // TODO: only prioritize sign when both values arent at their min/max
            
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
    // merge by favoring smaller numbers
    void merge(LeafCluster& other) {
        for (ClusterT i = 0; i < 8; i++) {
            // mask out bits for current leaf
            typedef std::make_signed_t<ClusterT> ClusterInt;
            int32_t maskedA = (this->cluster >> i*nBits) & leafMask;
            int32_t maskedB = (other.cluster >> i*nBits) & leafMask;
            
            // convert back to standard readable int
            int32_t a = maskedA - range;
            int32_t b = maskedB - range;
            
            bool overwrite = false;
            if (std::abs(a) == std::abs(b) && b > a) overwrite = true;
            if (std::abs(a) > std::abs(b)) overwrite = true;
            
            // overwrite if b has a smaller signed distance
            if (overwrite) {
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
        // n bits precision for each leaf
        int32_t leaf = cluster >> index*nBits;
        leaf &= leafMask;
        // convert back to standard signed
        leaf -= (int32_t)range;
        // convert to floating signed distance
        float signedDistance = (float)leaf;
        signedDistance /= (float)range; // normalize signed distance
        signedDistance *= maxDist; // scale signed distance to real size
        return signedDistance;
    }
    bool operator==(const LeafCluster& other) const {
        return cluster == other.cluster;
    }
    
    ClusterT cluster;
    static constexpr float maxDist = leafResolution;
    static constexpr ClusterT nBits = 8; // 1b sign, rest data
    static constexpr ClusterT leafMask = (1 << nBits) - 1; // mask for a single leaf
    static constexpr ClusterT range = leafMask / 2; // achievable range with data bits
    // cluster value when all leaves have the maximum positive signed distance
    static constexpr ClusterT max = 0b1111111011111110111111101111111011111110111111101111111011111110;
    // cluster value when all leaves have the maximum negative signed distance
    static constexpr ClusterT min = 0b0000000000000000000000000000000000000000000000000000000000000000;
};  
};