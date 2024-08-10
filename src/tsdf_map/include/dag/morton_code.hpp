#pragma once
#include <chrono>
#include <Eigen/Eigen>
#include <morton-nd/mortonND_BMI2.h>
#include "dag/constants.hpp"

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
    inline bool operator!=(const MortonCode& other) const { return val != other.val; }
    inline bool operator<(const MortonCode& other) const { return val < other.val; }
    inline bool operator>(const MortonCode& other) const { return val > other.val; }
    uint64_t val;
};

namespace std {
    template<>
    struct hash<MortonCode> {
        inline size_t operator()(const MortonCode& x) const {
            return x.val;
        }
    };
}

static auto calc_morton(std::vector<Eigen::Vector3f>& points) -> std::vector<std::pair<MortonCode, Eigen::Vector3f>> {
    auto beg = std::chrono::steady_clock::now();
    
    // create a vector to hold sortable morton codes alongside position
    std::vector<std::pair<MortonCode, Eigen::Vector3f>> mortonCodes;
    mortonCodes.reserve(points.size());
    
    // iterate through all points to populate morton code vector
    for (auto it_points = points.cbegin(); it_points != points.cend(); it_points++) {
        Eigen::Vector3f position = *it_points;
        // transform into chunk position (leaf chunk in this case)
        Eigen::Vector3f chunkPosFloat = position * (1.0 / leafResolution);
        // properly floor instead of relying on float -> int conversion
        chunkPosFloat = chunkPosFloat.unaryExpr([](float f){ return std::floor(f); });
        Eigen::Vector3i chunkPos = chunkPosFloat.cast<int32_t>();
        // calculate morton code and insert into vector
        MortonCode mc(chunkPos);
        mortonCodes.emplace_back(mc.val, position);
    }
    
    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "mort calc " << dur << '\n';
    return mortonCodes;
}