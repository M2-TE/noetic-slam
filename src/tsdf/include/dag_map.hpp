#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
#include <bitset>
#include <concepts>
#include <type_traits>
//
#include <boost/math/ccmath/ccmath.hpp>
#include <Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
// help intellisense be not stupid for once
#ifndef __BMI2__
#define __BMI2__
#endif
#include <morton-nd/mortonND_BMI2.h>
//
#include "constants.hpp"
#include "dag_structs.hpp"

// https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
template<std::floating_point T, size_t N>
Eigen::Matrix<T, 4, 1> normal_from_neighbourhood(Eigen::Matrix<T, 4, Eigen::Dynamic, 0, 4, N>& points) {
    typedef Eigen::Matrix<T, 4, 1> Vec4;
    // calculate centroid by through coefficient average
    Vec4 centroid = {};
    for (const T* pCur = points.data(); pCur != points.data() + points.size(); pCur += 4) {
        centroid += *reinterpret_cast<const Vec4*>(pCur);
    }
    centroid /= (T)points.cols();

    // covariance matrix excluding symmetries
    T xx = 0.0;
    T xy = 0.0;
    T xz = 0.0;
    T yy = 0.0;
    T yz = 0.0;
    T zz = 0.0;
    for (const T* pCur = points.data(); pCur != points.data() + points.size(); pCur += 4) {
        auto r = *reinterpret_cast<const Vec4*>(pCur) - centroid;
        xx += r.x() * r.x();
        xy += r.x() * r.y();
        xz += r.x() * r.z();
        yy += r.y() * r.y();
        yz += r.y() * r.z();
        zz += r.z() * r.z();
    }
    xx /= (T)points.cols();
    xy /= (T)points.cols();
    xz /= (T)points.cols();
    yy /= (T)points.cols();
    yz /= (T)points.cols();
    zz /= (T)points.cols();

    // weighting linear regression based on square determinant
    Vec4 weighted_dir = {};
    Vec4 axis_dir = {};
    T weight = 0.0;

    // determinant x
    T det_x = yy*zz - yz*yz;
    axis_dir = {
        det_x,
        xz*yz - xy*zz,
        xy*yz - xz*yy,
        0.0
    };
    weight = det_x * det_x;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // determinant y
    T det_y = xx*zz - xz*xz;
    axis_dir = {
        xz*yz - xy*zz,
        det_y,
        xy*xz - yz*xx,
        0.0
    };
    weight = det_y * det_y;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // determinant z
    T det_z = xx*yy - xy*xy;
    axis_dir = {
        xy*yz - xz*yy,
        xy*xz - yz*xx,
        det_z,
        0.0
    };
    weight = det_z * det_z;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // return normalized weighted direction as surface normal
    return weighted_dir.normalized();
}

// use n-bit signed integral as morton code input with locality between -1, 0, +1
template<std::signed_integral T, size_t N_VEC>
static inline uint_fast64_t calc_morton_signed(Eigen::Matrix<T, N_VEC, 1> input) {
    auto res = input.unaryExpr([](const T i) {
        typedef std::make_unsigned_t<T> uint;
        constexpr uint shiftMask = mortonnd::MortonNDBmi_3D_64::FieldBits - 1;
        constexpr uint shiftSign = sizeof(T) * 8 - 1;
        constexpr uint shiftSign21 = shiftSign - shiftMask;
        constexpr T signBit = 1 << shiftSign;
        constexpr T bitmask = (1 << shiftMask) - 1;
        return
            (i & bitmask) | // limit to 20 bits (note: mask may not be needed, mortonnd masks too)
            ((i & signBit ^ signBit) >> shiftSign21); // inverted sign bit for 21 bits total
    });
    return mortonnd::MortonNDBmi_3D_64::Encode(res.x(), res.z(), res.y());
}

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
static constexpr uint32_t iRoot = 0;
static constexpr uint32_t iLeaf = nDagLevels;
static constexpr uint32_t iLeafCluster = iLeaf - 1;
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

    void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, const std::vector<Eigen::Vector3f>& points) {
        for (uint32_t depth = 0; depth < nDagLevels + 1; depth++) {
            std::cout << "depth " << depth << ": " 
                << dagSizes[depth] << " voxel(s), " 
                << dagResolutions[depth] << " resolution" << std::endl;
        }
        typedef std::pair<uint_fast64_t, uint32_t> MortonIndex;
        std::vector<MortonIndex> mortonCodes;

        // calculate morton codes based on leaf voxel position
        mortonCodes.reserve(points.size());
        for (auto pCur = points.cbegin(); pCur != points.cend(); pCur++) {
            // calculate leaf voxel position
            Eigen::Matrix<int32_t, 3, 1> vPos = (*pCur * (1.0 / leafResolution)).cast<int32_t>();
            // create 63-bit morton code from 3x21-bit fields
            // z order priority will be: x -> y -> z
            uint_fast64_t mortonCode = calc_morton_signed<int32_t, 3>(vPos);
            mortonCodes.emplace_back(mortonCode, mortonCodes.size());

            // todo: calc morton codes for all other DAG layers as well
        }
        // sort via morton codes to obtain z-order
        std::sort(
            mortonCodes.begin(), 
            mortonCodes.end(), 
            [](const MortonIndex& a, const MortonIndex& b) { return a.first > b.first; }
        );

        // adjust these two as needed (high radius bcs of random input data atm)
        constexpr double maxRadius = leafResolution * 500.0;
        constexpr size_t maxNeighbours = 64;
        // nearest neighbours via z-order locality
        for (auto pCur = mortonCodes.cbegin(); pCur != mortonCodes.cend(); pCur++) {
            constexpr float maxRadiusSqr = maxRadius * maxRadius;
            Eigen::Vector4d point = points[pCur->second].head<4>().cast<double>();
            point.w() = 0.0;

            // 4-point vectors to leverage SIMD instructions
            Eigen::Matrix<double, 4, Eigen::Dynamic, 0, 4, maxNeighbours> neighbours(4, maxNeighbours);
            // add current point to "neighbours" to include it in normal estimation
            neighbours.col(0) = point;
            uint_fast32_t nNeighbours = 1;

            // forward neighbours
            for (auto pFwd = pCur + 1; pFwd < mortonCodes.cend(); pFwd++) {
                Eigen::Vector4d candidate = points[pFwd->second].head<4>().cast<double>();
                candidate.w() = 0.0;
                double distSqr = (point - candidate).squaredNorm();
                if (distSqr < maxRadiusSqr) {
                    neighbours.col(nNeighbours) = candidate;
                    if (++nNeighbours >= maxNeighbours / 2) break;
                }
                else break;
            }
            // backward neighbours
            for (auto pBwd = pCur - 1; pBwd >= mortonCodes.cbegin(); pBwd--) {
                Eigen::Vector4d candidate = points[pBwd->second].head<4>().cast<double>();
                candidate.w() = 0.0;
                double distSqr = (point - candidate).squaredNorm();
                if (distSqr < maxRadiusSqr) {
                    neighbours.col(nNeighbours) = candidate;
                    if (++nNeighbours >= maxNeighbours) break;
                }
                else break;
            }
            neighbours.conservativeResize(Eigen::NoChange, nNeighbours);

            Eigen::Vector4d normal;
            // estimate normal via neighbourhood if enough neighbours are present
            if (nNeighbours >= 3) {
                normal = normal_from_neighbourhood<double, maxNeighbours>(neighbours);
            }
            // else use pose-to-point
            else {
                Eigen::Vector4d pos = position.cast<double>().head<4>();
                pos.w() = 0.0;
                normal = pos;
            }
            // std::cout << normal.x() << " " << normal.y() << " " << normal.z() << " " << normal.w() << std::endl;
            // std::cout << neighbours.cols() << std::endl;
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