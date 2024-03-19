#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <span>
//
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/math/ccmath/ccmath.hpp>
#include <eigen3/Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
#include <parallel_hashmap/btree.h>
#include <morton-nd/mortonND_BMI2.h>
#include "dag_structs.hpp"
#include "trie.hpp"

// https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
static Eigen::Vector3f normal_from_neighbourhood(std::span<Eigen::Vector3f> points) {
    typedef double prec;
    // calculate centroid by through coefficient average
    Eigen::Vector3d centroid = { 0, 0, 0 };
    for (auto p = points.begin(); p != points.end(); p++) {
        centroid += p->cast<prec>();
    }
    centroid /= (prec)points.size();

    // covariance matrix excluding symmetries
    prec xx = 0.0;
    prec xy = 0.0;
    prec xz = 0.0;
    prec yy = 0.0;
    prec yz = 0.0;
    prec zz = 0.0;
    for (auto p = points.begin(); p != points.end(); p++) {
        auto r = p->cast<prec>() - centroid;
        xx += r.x() * r.x();
        xy += r.x() * r.y();
        xz += r.x() * r.z();
        yy += r.y() * r.y();
        yz += r.y() * r.z();
        zz += r.z() * r.z();
    }
    xx /= (prec)points.size();
    xy /= (prec)points.size();
    xz /= (prec)points.size();
    yy /= (prec)points.size();
    yz /= (prec)points.size();
    zz /= (prec)points.size();

    // weighting linear regression based on square determinant
    Eigen::Vector3d weighted_dir = {};
    Eigen::Vector3d axis_dir = {};
    prec weight = 0.0;

    // determinant x
    prec det_x = yy*zz - yz*yz;
    axis_dir = {
        det_x,
        xz*yz - xy*zz,
        xy*yz - xz*yy
    };
    weight = det_x * det_x;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // determinant y
    prec det_y = xx*zz - xz*xz;
    axis_dir = {
        xz*yz - xy*zz,
        det_y,
        xy*xz - yz*xx
    };
    weight = det_y * det_y;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // determinant z
    prec det_z = xx*yy - xy*xy;
    axis_dir = {
        xy*yz - xz*yy,
        xy*xz - yz*xx,
        det_z
    };
    weight = det_z * det_z;
    if (weighted_dir.dot(axis_dir) < 0.0) weight = -weight;
    weighted_dir += axis_dir * weight;

    // return normalized weighted direction as surface normal
    return weighted_dir.normalized().cast<float>();
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
    struct MortonCode {
        MortonCode(int x, int y, int z): MortonCode(Eigen::Vector3i(x, y, z)) {}
        MortonCode(uint64_t code): val(code) {}
        MortonCode(Eigen::Vector3i vec) {
            // use n-bit signed integral as morton code input with locality between -1, 0, +1
            auto res = vec.unaryExpr([](const int32_t i) {
                constexpr uint signBit = 1 << 31;
                constexpr uint signMask = signBit - 1;
                constexpr uint mask = (1 << 20) - 1;
                // invert sign
                uint32_t sign = (i & signBit) ^ signBit;
                // shift sign to 21st bit
                sign = sign >> 11;
                // combine sign with i
                uint32_t res = (i & mask) | sign;
                return (int32_t)res;
            });
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
        uint64_t val;
    };
    struct MortonIndex { Eigen::Vector3f point; uint32_t index; };
    struct Pose { Eigen::Vector3f pos; Eigen::Quaternionf rot; };
    struct Scan { Pose pose; std::vector<NodePointer> roots; };

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

        auto get_normals(Pose pose, std::vector<Eigen::Vector3f>& points) {
            auto beg = std::chrono::steady_clock::now();
            phmap::btree_multimap<MortonCode, MortonIndex> mortonMap;

            // calculate morton codes based on voxel position
            uint32_t i = 0;
            // std::vector<MortonCode>
            for (auto pCur = points.begin(); pCur != points.end(); pCur++) {
                // calculate leaf voxel position
                Eigen::Matrix<int32_t, 3, 1> vPos = (*pCur * (1.0 / leafResolution)).cast<int32_t>();
                // assign to voxel chunk
                vPos /= (int32_t)dagSizes[mortonVolume];
                // create 63-bit morton code from 3x21-bit fields
                mortonMap.emplace(vPos, MortonIndex(*pCur, i++));
            }
            auto end = std::chrono::steady_clock::now();
            auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
            std::cout << "construction: " << dur << " ms" << std::endl;
            
            // estimate normals based on kNN (nearest neighbours)
            std::vector<Eigen::Vector3f> normals(points.size());
            beg = std::chrono::steady_clock::now();
            for (auto pCur = mortonMap.cbegin(); pCur != mortonMap.cend(); pCur++) {
                Eigen::Vector3f point = pCur->second.point;
                
                std::vector<Eigen::Vector3f> neighbours;
                neighbours.push_back(point);

                // traverse morton code neighbours for nearest neighbour search
                auto [xC, yC, zC] = mortonnd::MortonNDBmi_3D_64::Decode(pCur->first.val);
                constexpr decltype(xC) off = 1; // offset (1 = 3x3x3 morton neighbourhood)
                for (auto x = xC - off; x <= xC + off; x++) {
                    for (auto y = yC - off; y <= yC + off; y++) {
                        for (auto z = zC - off; z <= zC + off; z++) {
                            // generate morton code from new coordinates
                            MortonCode mc = mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);
                            // MortonCode mc = x + y + z;
                            // iterate over all values for current key
                            auto ptr = mortonMap.find(mc);
                            if (ptr == mortonMap.cend()) continue;
                            for (; ptr->first == mc && ptr != mortonMap.cend(); ptr++) {
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
            end = std::chrono::steady_clock::now();
            dur = std::chrono::duration<double, std::milli> (end - beg).count();
            std::cout << "normal est: " << dur << " ms" << std::endl;
            return normals;
        }
        auto get_trie(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) {
            auto beg = std::chrono::steady_clock::now();
            Trie trie;
            auto pNorm = normals.cbegin();
            for (auto p = points.cbegin(); p != points.cend(); p++, pNorm++) {
                // leaf cluster position that p belongs to
                Eigen::Vector3f clusterPos = *p * (0.5f / leafResolution);
                Eigen::Vector3i voxelPos = clusterPos.cast<int32_t>();
                
                // calculate signed distances for neighbours of current leaf cluster as well
                constexpr int32_t off = 1; // offset (1 = 3x3x3)
                for (auto x = -off; x <= +off; x++) {
                    for (auto y = -off; y <= +off; y++) {
                        for (auto z = -off; z <= +off; z++) {
                            // this is be a leaf cluster containing 8 individual leaves
                            Eigen::Vector3i cPos = voxelPos + Eigen::Vector3i(x, y, z);
                            // actual floating position of cluster
                            Eigen::Vector3f fPos = cPos.cast<float>() * 2.0f * leafResolution;

                            // offsets of leaves within
                            std::array<float, 8> leaves;
                            auto pLeaf = leaves.begin();
                            for (auto xl = 0; xl < 2; xl++) {
                                for (auto yl = 0; yl < 2; yl++) {
                                    for (auto zl = 0; zl < 2; zl++) {
                                        // leaf position
                                        Eigen::Vector3f offset = Eigen::Vector3f(xl, yl, zl) * leafResolution;
                                        Eigen::Vector3f lPos = fPos + offset;
                                        *pLeaf = pNorm->dot(*p - lPos);
                                        pLeaf++;
                                    }
                                }
                            }

                            // sd max should be turned into a parameter
                            constexpr double sdMax = boost::math::ccmath::sqrt(3.0*3.0*3.0) * leafResolution;
                            constexpr float sdMaxRecip = 1.0 / sdMax;

                            MortonCode code(cPos);
                            auto& cluster = trie.find(code.val);

                            // pack all leaves into 32 bits
                            typedef uint32_t pack;
                            pack packedLeaves = 0;
                            for (auto i = 0; i < leaves.size(); i++) {
                                float sd = leaves[i];
                                // normalize between -1.0 and 1.0 (not yet clamped)
                                sd = sd * sdMaxRecip;

                                constexpr pack sdBits = 4;
                                constexpr pack sdMask = (1 << (sdBits - 1)) - 1;
                                constexpr float sdConv = static_cast<float>(sdMask);
                                // expand for int conversion
                                sd = sd * sdConv;
                                // convert absolute value, copy sign manually
                                pack sdInt = static_cast<int>(std::abs(sd));
                                pack signBit = std::signbit(sd) << (sdBits - 1);
                                sdInt |= signBit;
                                
                                // check if signed distance is smaller than saved one
                                if (cluster != Trie::defVal) {
                                    constexpr pack mask = (1 << sdBits) - 1;
                                    // extract relevant portion
                                    pack part = (cluster >> i * sdBits) & mask;
                                    // < comparison works if treating sign bit as data
                                    if (part < sdInt) sdInt = part;
                                }
                                packedLeaves |= sdInt << i * sdBits;
                            }
                            cluster = packedLeaves;
                        }
                    }
                }
            }
            auto end = std::chrono::steady_clock::now();
            auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
            std::cout << "trie ctor: " << dur << " ms" << std::endl;
            return trie;
        }
        void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, std::vector<Eigen::Vector3f>& points) {
            Pose pose = { position, rotation };
            auto normals = get_normals(pose, points);
            auto trie = get_trie(points, normals);
            trie.printstuff();

            auto iter = trie.get_depth_first();
            for (auto leaf: iter.path[20].pNode->leafClusters) {
                std::cout << leaf << '\n';
            }
        }

    private:
        std::vector<Scan> scans; // TODO
        phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
        std::array<DAG::Level, nDagLevels> dagLevels;
    };
}