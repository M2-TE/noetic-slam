#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <sstream>
#include <type_traits>
//
#include "constants.hpp"

struct LeafCluster {
    typedef uint64_t ClusterT;
    typedef std::make_signed_t<ClusterT> ClusterS;
    typedef uint32_t PartT;
    LeafCluster(ClusterT cluster): cluster(cluster) {}
    LeafCluster(PartT part0, PartT part1): cluster((ClusterT)part0 | ((ClusterT)part1 << 32)) {}
    LeafCluster(std::array<std::pair<float, bool>, 8>& leaves): cluster(0) {
        for (ClusterT i = 0; i < 8; i++) {
            // check validity of leaf
            if (!leaves[i].second) {
                // store "invalid" leaf by setting all bits
                static_assert(nBits == 8);
                cluster |= (ClusterT)0b11111111 << i*nBits;
                continue;
            }
            
            // normalize sd to [-1, 1]
            float sdNormalized = leaves[i].first * (1.0 / leafResolution);
            sdNormalized = std::clamp(sdNormalized, -1.0f, 1.0f);
            
            // // convert from linear to quadratic
            // float sdQuad = std::sqrt(sdNormalized);
            // if (sdNormalized < 0) sdQuad *= -1.0f;
            // sdNormalized = sdQuad;
            
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
    // get signed distance of specified leaf, if it is valid
    std::optional<float> get_sd(ClusterT index) {
        // n bits precision for each leaf
        int32_t leaf = cluster >> index*nBits;
        leaf &= leafMask;
        // return early if leaf is invalid
        if (leaf == 0b11111111) return std::nullopt;
        
        // convert back to standard signed
        leaf -= (int32_t)range;
        // convert to floating signed distance
        float signedDistance = (float)leaf;
        signedDistance /= (float)range; // normalize signed distance
        
        // // convert from quadratic to linear
        // float sdLinear = signedDistance * signedDistance;
        // signedDistance = sdLinear;
        
        signedDistance *= leafResolution; // scale signed distance to real size
        return signedDistance;
    }
    bool operator==(const LeafCluster& other) const {
        return cluster == other.cluster;
    }
    
    ClusterT cluster;
    static constexpr ClusterT nBits = 8; // 1b sign, rest data
    static constexpr ClusterT leafMask = (1 << nBits) - 1; // mask for a single leaf
    static constexpr ClusterT range = leafMask / 2; // achievable range with data bits
};