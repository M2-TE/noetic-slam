#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
#include <bitset>
#include <concepts>
#include <type_traits>
#include <map>
//
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/math/ccmath/ccmath.hpp>
#include <Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
#include <parallel_hashmap/btree.h>
// help intellisense be not stupid for once
#ifndef __BMI2__
#define __BMI2__
#endif
#include <morton-nd/mortonND_BMI2.h>
//
#include "constants.hpp"
#include "dag_structs.hpp"

// https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
Eigen::Vector3f normal_from_neighbourhood(std::vector<Eigen::Vector3f>& points) {
    // calculate centroid by through coefficient average
    Eigen::Vector3f centroid = {};
    for (auto p = points.cbegin(); p != points.cend(); p++) {
        centroid += *p;
    }
    centroid /= (float)points.size();

    // covariance matrix excluding symmetries
    float xx = 0.0;
    float xy = 0.0;
    float xz = 0.0;
    float yy = 0.0;
    float yz = 0.0;
    float zz = 0.0;
    for (auto p = points.cbegin(); p != points.cend(); p++) {
        auto r = *p - centroid;
        xx += r.x() * r.x();
        xy += r.x() * r.y();
        xz += r.x() * r.z();
        yy += r.y() * r.y();
        yz += r.y() * r.z();
        zz += r.z() * r.z();
    }
    xx /= (float)points.size();
    xy /= (float)points.size();
    xz /= (float)points.size();
    yy /= (float)points.size();
    yz /= (float)points.size();
    zz /= (float)points.size();

    // weighting linear regression based on square determinant
    Eigen::Vector3f weighted_dir = {};
    Eigen::Vector3f axis_dir = {};
    float weight = 0.0;

    // determinant x
    float det_x = yy*zz - yz*yz;
    axis_dir = {
        det_x,
        xz*yz - xy*zz,
        xy*yz - xz*yy
    };
    weight = det_x * det_x;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // determinant y
    float det_y = xx*zz - xz*xz;
    axis_dir = {
        xz*yz - xy*zz,
        det_y,
        xy*xz - yz*xx
    };
    weight = det_y * det_y;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // determinant z
    float det_z = xx*yy - xy*xy;
    axis_dir = {
        xy*yz - xz*yy,
        xy*xz - yz*xx,
        det_z
    };
    weight = det_z * det_z;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // return normalized weighted direction as surface normal
    return weighted_dir.normalized();
}

namespace DAG {
static constexpr double leafResolution = 0.01; // real distance for each voxel step
static constexpr uint32_t nDagLevels = 10; // number of DAG levels including root and leaf clusters (excluding leaves)
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
namespace idx { // static indices
    static constexpr size_t root = 0;
    static constexpr size_t leaf = nDagLevels;
    static constexpr size_t leafCluster = leaf - 1;
}
static constexpr size_t mortonVolume = idx::leaf - 4;
typedef uint64_t MortonCode;
struct MortonIndex { Eigen::Vector3f point; uint32_t index; };
struct Pose { Eigen::Vector3f pos; Eigen::Quaternionf rot; };
struct Scan {
    Pose pose;
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

    void print_info() {
        for (uint32_t depth = 0; depth < nDagLevels + 1; depth++) {
            std::cout << "depth " << depth << ": " 
                << dagSizes[depth] << "^3" << " voxels, " 
                << dagResolutions[depth] << "^3" << " resolution";
            if (depth == idx::leaf) std::cout << " (implicit)";
            std::cout << std::endl;
        }
        std::cout << "morton volume (leaf NN) of " << dagSizes[mortonVolume] << "^3 voxels";
        std::cout << "(" << dagSizes[mortonVolume] * leafResolution << ")" << std::endl;
    }
    
    // use n-bit signed integral as morton code input with locality between -1, 0, +1
    template<std::signed_integral T>
    uint_fast64_t calc_morton_signed(Eigen::Matrix<T, 3, 1> input) {
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
    template<size_t depth> 
    auto get_morton_map(std::vector<Eigen::Vector3f>& points) {
        // multimap properties:
        // 1. sorted
        // 2. multiple values per key
        // 3. cache friendly due to 1. and 2.
        phmap::btree_multimap<MortonCode, MortonIndex> mortonMap;
        // std::multimap<MortonCode, MortonIndex> mortonMap;

        // calculate morton codes based on voxel position
        uint32_t i = 0;
        for (auto pCur = points.cbegin(); pCur != points.cend(); pCur++) {
            // calculate leaf voxel position
            Eigen::Matrix<int32_t, 3, 1> vPos = (*pCur * (1.0 / leafResolution)).cast<int32_t>();
            // assign to voxel chunk
            vPos /= (int32_t)dagSizes[depth];
            // create 63-bit morton code from 3x21-bit fields
            MortonCode mortonCode = calc_morton_signed<int32_t>(vPos);
            mortonMap.emplace(mortonCode, MortonIndex(*pCur, i++));
        }
        return mortonMap;
    }
    auto get_normals(Pose pose, std::vector<Eigen::Vector3f>& points) {
        auto mortonMap = get_morton_map<mortonVolume>(points);
        std::vector<Eigen::Vector3f> normals(points.size());
        // estimate normals based on kNN (nearest neighbours)
        for (auto pCur = mortonMap.cbegin(); pCur != mortonMap.cend(); pCur++) {
            Eigen::Vector3f point = pCur->second.point;
            
            std::vector<decltype(point)> neighbours;
            neighbours.push_back(point);

            // morton code components are 21-bit signed integers (stored in unsigned)
            // 21st bit is set for positive numbers to achieve smooth range from -max to +max
            // performing math operations on it works as expected (relative to other 21-bit values)
            // these values CANNOT be interpreted as real numbers without proper conversion
            auto [xC, yC, zC] = mortonnd::MortonNDBmi_3D_64::Decode(pCur->first);

            // traverse morton code neighbours for nearest neighbour search
            constexpr decltype(xC) off = 1; // offset (1 = 3x3x3 morton neighbourhood)
            for (auto x = xC - off; x <= xC + off; x++) {
                for (auto y = yC - off; y <= yC + off; y++) {
                    for (auto z = zC - off; z <= zC + off; z++) {
                        // generate morton code from new coordinates
                        MortonCode mc = mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);
                        // iterate over all values for current key
                        decltype(mortonMap)::const_iterator ptr = mortonMap.find(mc);
                        if (ptr == mortonMap.cend()) continue;
                        for (; ptr->first == mc; ptr++) {
                            neighbours.push_back(ptr->second.point);
                        }
                    }
                }
            }
            // std::cout << neighbours.size() << std::endl;
            
            // estimate normal via neighbourhood if enough neighbours are present
            // else use pose-to-point
            Eigen::Vector3f normal;
            if (neighbours.size() > 1) normal = normal_from_neighbourhood(neighbours);
            else normal = (pose.pos - point).normalized();
            normals[pCur->second.index] = normal;
        }
        return normals;
    }
    void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, std::vector<Eigen::Vector3f>& points) {
        // print_info();
        Pose pose = { position, rotation };
        auto normals = get_normals(pose, points);
        
        // scanpoints influence an area of 3x3x3 leaf clusters (TSDF values)
        auto test = get_morton_map<idx::leafCluster>(points);
        auto testcpy = test; // for insertion
        for (auto p = test.begin(); p != test.end(); p++) {
            auto point = p->second.point;

            auto [xC, yC, zC] = mortonnd::MortonNDBmi_3D_64::Decode(p->first);
            // traverse morton code neighbours
            constexpr decltype(xC) off = 1; // offset (1 = 3x3x3 morton neighbourhood)
            for (auto x = xC - off; x <= xC + off; x++) {
                for (auto y = yC - off; y <= yC + off; y++) {
                    for (auto z = zC - off; z <= zC + off; z++) {
                        // generate morton code from new coordinates
                        MortonCode mc = mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);
                        // insert current point into neighbour for TSDF calcs later on
                        testcpy.emplace(mc, MortonIndex(point, p->second.index));
                    }
                }
            }
        }

        // TODO: iterate over testcpy for leaf clusters

        // TODO: dont actually need to know normals in this scope
        // use the neighbours vector to determine which point is closest to the voxel
        // -> (store indices+distanceSqr per leaf cluster?)
        // after normal calc, write actual TSDF values into leaves (clusters?)
    }

private:
    std::vector<Scan> scans; // TODO
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};
}