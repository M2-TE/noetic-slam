#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
#include <bitset>
//
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/math/ccmath/ccmath.hpp>
#include <Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
#ifdef __INTELLISENSE__
#define __BMI2__
#endif
#include <morton-nd/mortonND_BMI2.h>
//
#include "constants.hpp"
#include "dag_structs.hpp"

namespace DAG {
static constexpr double leafResolution = 0.01; // real distance for each voxel step
static constexpr uint32_t nDagLevels = 3; // number of DAG levels including root and leaf clusters (excluding leaves)
static constexpr uint32_t nAllLevels = nDagLevels + 1; // nDagLevels including implicit leaf depth
static constexpr std::array<uint32_t, nAllLevels> get_sizes() {
    std::array<uint32_t, nAllLevels> arr = {};
    uint32_t current = 1;
    for (size_t i = nDagLevels; i > 0; i--) {
        arr[i] = current;
        current *= 2;
    }
    arr[0] = current;
    return arr;
}
static constexpr std::array<double, nAllLevels> get_resolutions() {
    std::array<double, nAllLevels> arr = {};
    double current = leafResolution;
    for (size_t i = nDagLevels; i > 0; i--) {
        arr[i] = current;
        current *= 2.0;
    }
    arr[0] = current;
    return arr;
};
static constexpr std::array<uint32_t, nAllLevels> dagSizes = get_sizes();
static constexpr std::array<double, nAllLevels> dagResolutions = get_resolutions();
struct Scan {
    Eigen::Vector3f position;
    Eigen::Vector3f rotation;
    std::vector<NodePointer> roots;
};
struct Map {
    Map() {
        // set up hash and equality functors with proper references
        for (size_t i = 0; i < dagLevels.size(); i++) {
            DAG::Level& level = dagLevels[i];

            DAG::HashFunctor hashFnc = { &level.data };
            DAG::CompFunctor compFnc = { &level.data };
            level.pointerSet = decltype(level.pointerSet)(0, hashFnc, compFnc);
        }
    }

    void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, const std::vector<Eigen::Vector3f>& scan) {
        for (uint32_t depth = 0; depth < nDagLevels + 1; depth++) {
            std::cout << "depth " << depth << ": " << dagSizes[depth] << " voxel(s), " << dagResolutions[depth] << " resolution" << std::endl;
        }

        std::vector<std::pair<uint_fast64_t, uint32_t>> mortonCodes;
        mortonCodes.reserve(scan.size());
        uint32_t currentLevel = 0;
        uint32_t index = 0;
        for (auto pCur = scan.cbegin(); pCur != scan.cend(); pCur++, index++) {
            // calculate voxel position in DAG tree
            typedef uint32_t uint21_t;
            Eigen::Matrix<uint21_t, 3, 1> vPos = (*pCur * (1.0 / dagResolutions[currentLevel])).cast<uint21_t>();
            // std::cout << "\n";
            // std::cout << (int)vPos.x() << " " << (int)vPos.y() << " " << (int)vPos.z() << std::endl;
            // std::cout << std::bitset<32>(vPos.x()) << " " << std::bitset<32>(vPos.y()) << " " << std::bitset<32>(vPos.z()) << std::endl;

            // map 32-bit int to 21-bits
            vPos = vPos.unaryExpr([](const uint21_t i) {
                constexpr uint21_t signBit32 = 1 << 31;
                constexpr uint21_t signBit21 = 1 << 20;
                constexpr uint21_t bitmask = signBit21 - 1;
                return
                    (i & bitmask) | // limit to 20 bits (note: mask may not be needed, mortonnd masks too)
                    ((i & signBit32 ^ signBit32) >> 11); // inverted sign bit for 21 bits total
            });
            // std::cout << (int)vPos.x() << " " << (int)vPos.y() << " " << (int)vPos.z() << std::endl;
            // std::cout << std::bitset<21>(vPos.x()) << " " << std::bitset<21>(vPos.y()) << " " << std::bitset<21>(vPos.z()) << std::endl;
            
            // create 63-bit morton code from 3x21-bit fields
            // z order priority will be: x -> y -> z
            static_assert(mortonnd::MortonNDBmi_3D_64::FieldBits == 21);
            mortonCodes.emplace_back(mortonnd::MortonNDBmi_3D_64::Encode<uint_fast32_t>(vPos.x(), vPos.z(), vPos.y()), index);
        }
        // sort via morton codes to obtain z-order
        std::sort(mortonCodes.begin(), mortonCodes.end(), [](
            const std::pair<uint_fast64_t, uint32_t>& a,
            const std::pair<uint_fast64_t, uint32_t>& b) {
                return a.first > b.first;
        });

        // testing z order
        for (auto pCur = mortonCodes.cbegin(); pCur != mortonCodes.cend(); pCur++) {
            auto index = pCur->second;
            // std::cout << scan[index].x() << " " << scan[index].y() << " " << scan[index].z() << std::endl;
        }


        // 1. calc morton codes
        // 2. sort (radix?) scan via morton code
        // 3? create binary radix tree from sorted data
        // 4. create DAG sub-tree from data
        // 5. calculate each point's normal via approximated nearest neighbour hyperplane
    }

private:
    std::vector<Scan> scans;

    // not really in use atm
    phmap::flat_hash_map<DAG::RootPos, DAG::NodePointer> dagRootMap;
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};
}