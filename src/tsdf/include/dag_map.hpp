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
// #ifdef __INTELLISENSE__
// #define __BMI2__
// #endif
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
        typedef uint_fast32_t uint21_t;
        typedef Eigen::Matrix<uint21_t, 3, 1> Vector3u;
        for (uint32_t depth = 0; depth < nDagLevels + 1; depth++) {
            std::cout << "depth " << depth << ": " << dagSizes[depth] << " voxel(s), " << dagResolutions[depth] << " resolution" << std::endl;
        }

        // calculate morton codes based on leaf voxel position
        uint32_t index = 0;
        std::vector<std::pair<uint_fast64_t, uint32_t>> mortonCodes;
        mortonCodes.reserve(scan.size());
        for (auto pCur = scan.cbegin(); pCur != scan.cend(); pCur++, index++) {
            // calculate leaf voxel position
            Vector3u vPos = (*pCur * (1.0 / dagResolutions[nDagLevels])).cast<uint21_t>();
            // map 32-bit int to 21-bits
            vPos = vPos.unaryExpr([](const uint21_t i) {
                constexpr uint21_t signBit32 = 1 << 31;
                constexpr uint21_t signBit21 = 1 << 20;
                constexpr uint21_t bitmask = signBit21 - 1;
                return
                    (i & bitmask) | // limit to 20 bits (note: mask may not be needed, mortonnd masks too)
                    ((i & signBit32 ^ signBit32) >> 11); // inverted sign bit for 21 bits total
            });
            // create 63-bit morton code from 3x21-bit fields
            // z order priority will be: x -> y -> z
            static_assert(mortonnd::MortonNDBmi_3D_64::FieldBits == 21);
            uint_fast64_t mortonCode = mortonnd::MortonNDBmi_3D_64::Encode(vPos.x(), vPos.z(), vPos.y());
            mortonCodes.emplace_back(mortonCode, index);
        }
        // sort via morton codes to obtain z-order
        std::sort(mortonCodes.begin(), mortonCodes.end(), [](
            const std::pair<uint_fast64_t, uint32_t>& a,
            const std::pair<uint_fast64_t, uint32_t>& b) {
                return a.first > b.first;
        });

        constexpr double maxRadius = leafResolution * 500.0;
        constexpr size_t maxNeighbours = 16;
        // nearest neighbours via z-order locality
        for (auto pCur = mortonCodes.cbegin(); pCur != mortonCodes.cend(); pCur++) {
            constexpr float maxRadiusSqr = maxRadius * maxRadius;
            Eigen::Vector3f point = scan[pCur->second];
            // A: store vector or index?
            // B: use sorted container like std::set? (sort by distSqr)
            std::vector<Eigen::Vector3d> neighbours;
            neighbours.reserve(maxNeighbours);

            // TODO: could leverage MatrixXd for SIMD acceleration
            // "point" as default value? would allow <50 neighbours
            // Eigen::Matrix<double, 4, Eigen::Dynamic> neighboursMat;
            // Eigen::Matrix<double, 4, maxNeighbours> neighboursMat;
            Eigen::Matrix<double, 4, Eigen::Dynamic, 0, 4, maxNeighbours> neighboursMat;

            // TODO: define END pointers here, so the for loops dont look as complex
            // can combine pointer with maxNeighbours that way
            
            // forward neighbours
            for (auto pFwd = pCur + 1; pFwd < mortonCodes.cend(); pFwd++) {
                Eigen::Vector3f candidate = scan[pFwd->second];
                float distSqr = (point - candidate).squaredNorm();
                if (distSqr < maxRadiusSqr) {
                    neighbours.push_back(candidate.cast<double>());
                    if (neighbours.size() >= maxNeighbours / 2) break;
                }
                else break;
            }
            // backward neighbours
            for (auto pBwd = pCur - 1; pBwd >= mortonCodes.cbegin(); pBwd--) {
                Eigen::Vector3f candidate = scan[pBwd->second];
                float distSqr = (point - candidate).squaredNorm();
                if (distSqr < maxRadiusSqr) {
                    neighbours.push_back(candidate.cast<double>());
                    if (neighbours.size() >= maxNeighbours) break;
                }
                else break;
            }
            
            // centroid for neighbourhood plane
            Eigen::Vector3d centroid = {0, 0, 0};
            for (auto pNei = neighbours.cbegin(); pNei != neighbours.cend(); pNei++) {
                centroid += *pNei;
            }
            centroid /= (double)neighbours.size();

            // calculate normal via covariance matrix
            Eigen::Matrix<double, 3, 3> covarianceMatrix = {};
            for (uint32_t i = 0; i < 3; i++) {
                for (uint32_t j = 0; j < 3; j++) {
                    for (auto pNei = neighbours.cbegin(); pNei != neighbours.cend(); pNei++) {
                        covarianceMatrix(i, j) += 
                            (centroid(i) - (*pNei)(i)) *
                            (centroid(j) - (*pNei)(j));
                    }
                    // covarianceMatrix(i, j) /= (double)neighbours.size();
                }
            }

            // compute eigenvalues and eigenvectors
            Eigen::EigenSolver<decltype(covarianceMatrix)> solver(covarianceMatrix, true);
            const auto& eigenvalues = solver.eigenvalues();
            // find lowest eigenvalue
            uint32_t iMin = 0;
            iMin = eigenvalues(1).real() < eigenvalues(iMin).real() ? 1 : iMin;
            iMin = eigenvalues(2).real() < eigenvalues(iMin).real() ? 2 : iMin;
            // eigenvector corresponding to lowest eigenvalue => surface normal
            Eigen::Vector3d norm = solver.eigenvectors().col(iMin).real();

            // std::cout << neighbours.size() << std::endl;
        }
    }

private:
    std::vector<Scan> scans;

    // not really in use atm
    phmap::flat_hash_map<DAG::RootPos, DAG::NodePointer> dagRootMap;
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};
}