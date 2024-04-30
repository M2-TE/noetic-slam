#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <span>
#include <thread>
#include <execution>
#include <parallel/algorithm>
#include <cstdint>
//
#include <boost/math/ccmath/ccmath.hpp>
#include <eigen3/Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
#include <parallel_hashmap/btree.h>
#include <morton-nd/mortonND_BMI2.h>
//
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
    static constexpr bool bLog = true;
    static double gTimerA = 0.0;
    static double gTimerB = 0.0;
    static double gTimerC = 0.0;
    struct Map {
        auto get_normals(Pose pose, std::vector<Eigen::Vector3f>& points) {
            auto beg = std::chrono::steady_clock::now();
            // points to be sorted via morton code
            std::vector<std::tuple<MortonCode, Eigen::Vector3f, uint32_t>> sorted;
            sorted.reserve(points.size());
            uint32_t i = 0;
            for (auto pCur = points.begin(); pCur != points.end(); pCur++) {
                // calculate leaf voxel position
                Eigen::Matrix<int32_t, 3, 1> vPos = (*pCur * (1.0 / leafResolution)).cast<int32_t>();
                // assign to voxel chunk
                vPos /= (int32_t)dagSizes[mortonVolume];
                // create 63-bit morton code from 3x21-bit fields
                sorted.emplace_back(vPos, *pCur, i++);
            }
            // sort via morton codes
            // __gnu_parallel::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b){
            std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b){
                return std::get<0>(a) > std::get<0>(b);
            });
            // std::vector<int> nums(100000);
            // std::sort(std::execution::par, nums.begin(), nums.end());

            // sort original points via sorted vector (redundant for now, helpful later on)
            auto pOut = points.begin();
            for (auto pCur = sorted.begin(); pCur != sorted.end(); pCur++, pOut++) {
                *pOut = std::get<1>(*pCur);
            }

            // create map lookup map for unique morton codes
            phmap::flat_hash_map<MortonCode, uint32_t> map;
            MortonCode last = std::get<0>(sorted.front());
            map.emplace(last, 0);
            i = 0;
            for (auto pCur = sorted.begin(); pCur != sorted.end(); pCur++, i++) {
                if (std::get<0>(*pCur) == last) continue;
                last = std::get<0>(*pCur);
                map.emplace(last, i);
            }

            if (bLog) {
                auto end = std::chrono::steady_clock::now();
                auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
                std::cout << "lookup-ctor: " << dur << " ms" << std::endl;
            }
            beg = std::chrono::steady_clock::now();
            
            // set up threads
            std::vector<std::jthread> threads;
            size_t nThreads = std::jthread::hardware_concurrency();
            threads.reserve(nThreads);

            // estimate normals based on nearest morton code neighbours
            size_t progress = 0;
            std::vector<Eigen::Vector3f> normals(points.size());
            for (size_t i = 0; i < nThreads; i++) {
                size_t nElements = points.size() / std::jthread::hardware_concurrency();
                if (i == nThreads - 1) nElements = 0; // special value for final thread
                threads.emplace_back([&sorted, &normals, &map, progress, nElements, pose](){
                    auto pOut = normals.begin() + progress;
                    auto pCur = sorted.cbegin() + progress;
                    auto pEnd = (nElements == 0) ? (sorted.cend()) : (pCur + nElements);
                    for (; pCur != pEnd; pCur++) {
                        Eigen::Vector3f point = std::get<1>(*pCur);
                        std::vector<Eigen::Vector3f> neighbours;
                        neighbours.push_back(point);

                        // traverse morton code neighbours for nearest neighbour search
                        auto [xC, yC, zC] = mortonnd::MortonNDBmi_3D_64::Decode(std::get<0>(*pCur).val);
                        constexpr decltype(xC) off = 1; // offset (1 = 3x3x3 morton neighbourhood)
                        for (auto x = xC - off; x <= xC + off; x++) {
                            for (auto y = yC - off; y <= yC + off; y++) {
                                for (auto z = zC - off; z <= zC + off; z++) {
                                    // generate morton code from new coordinates
                                    MortonCode mc = mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);

                                    // look up index for the point corresponding to mc
                                    auto iter = map.find(mc);
                                    if (iter == map.end()) continue;
                                    uint32_t index = iter->second;
                                    auto pSorted = sorted.cbegin() + index;

                                    // iterate over all values for current key
                                    while (pSorted != sorted.cend() && std::get<0>(*pSorted) == mc) {
                                        neighbours.push_back(std::get<1>(*pSorted));
                                        pSorted++;
                                    }
                                }
                            }
                        }
                        // std::cout << neighbours.size() << std::endl;
                        Eigen::Vector3f normal;
                        if (neighbours.size() > 1) normal = normal_from_neighbourhood(neighbours);
                        else normal = (pose.pos - point).normalized();
                        normals[std::get<2>(*pCur)] = normal;
                    }
                });
                progress += nElements;
            }
            threads.clear();

            if (bLog) {
                auto end = std::chrono::steady_clock::now();
                auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
                std::cout << "normals: " << dur << " ms" << std::endl;
            }
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
            if (bLog) {
                auto end = std::chrono::steady_clock::now();
                auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
                std::cout << "trie ctor: " << dur << " ms" << std::endl;
            }
            return trie;
        }
        uint32_t create_leaf_node(Trie::Node* pNode) {
            // construct a DAG node
            Node<8> newNode = {};
            size_t nClusters = 0;
            // go over all children
            for (ChildMask i = 0; i < 8; i++) {
                auto cluster = pNode->leafClusters[i];
                if (cluster == Trie::defVal) continue;
                // add to node and insert into mask
                newNode.children[nClusters++] = cluster;
                newNode.childMask |= 1 << i;
            }
            // insert temporary node into data array
            auto& level = dagLevels.back();
            auto& data = level.data;
            level.data.resize(level.dataSize + nClusters + 1);
            std::memcpy(data.data() + level.dataSize, &newNode, nClusters + 1);

            // check if the same node existed previously, only then do we count up dataSize
            auto& pointers = level.pointers;
            auto [pIndex, bNew] = pointers.emplace(level.dataSize);
            if (bNew) {
                level.dataSize += 1 + nClusters; // mask + children
                uniques.back()++;
            }
            else dupes.back()++;
            
            return *pIndex;
        }
        uint32_t create_normal_node(std::array<uint32_t, 8>& children, size_t depth) {
            Node<8> newNode = {};
            size_t nChildren = 0;
            // go over all children
            for (ChildMask i = 0; i < children.size(); i++) {
                auto child = children[i];
                if (child == 0) continue;
                newNode.children[nChildren++] = child;
                newNode.childMask |= 1 << i;
            }
            // insert temporary node into data array
            auto& level = dagLevels[depth];
            auto& data = level.data;
            level.data.resize(level.dataSize + nChildren + 1);
            std::memcpy(data.data() + level.dataSize, &newNode, nChildren + 1);

            // check if the same node existed previously, only then do we count up dataSize
            auto& pointers = level.pointers;
            auto [pIndex, bNew] = pointers.emplace(level.dataSize);
            if (bNew) {
                level.dataSize += 1 + nChildren; // mask + children
                uniques[depth]++;
            }
            else dupes[depth]++;
            return *pIndex;
        }
        void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, std::vector<Eigen::Vector3f>& points) {
            Pose pose = { position, rotation };
            auto normals = get_normals(pose, points);
            auto trie = get_trie(points, normals);
            trie.printstuff();
            auto beg = std::chrono::steady_clock::now();

            // keep track of path
            struct Layer {
                std::array<NodeIndex, 8> nodeIndices = {0,0,0,0,0,0,0,0};
                size_t index = 0;
            };
            std::array<Layer, nDagLevels> cache;
            std::array<Trie::Node*, nDagLevels - 1> path;
            size_t depth = 0;
            gTimerA = gTimerB = gTimerC = 0.0;
            path[depth] = trie.get_root();
            do {
                auto& cacheIndex = cache[depth].index;
                auto& cacheNodes = cache[depth].nodeIndices;
                while (true) {
                    // retrace to parent when all children were checked
                    if (cacheIndex == 8) {
                        // create normal node
                        uint32_t node = create_normal_node(cacheNodes, depth);
                        gTimerA += 1.0;
                        // reset cache for this level
                        cacheIndex = 0;
                        // go up by one level
                        depth--;
                        // update parent level
                        auto& parentLevel = cache[depth];
                        parentLevel.nodeIndices[parentLevel.index++] = node;
                        break;
                    }

                    // child invalid: invalidate cache entry
                    auto* pChild = path[depth]->children[cacheIndex];
                    if (pChild == (Trie::Node*)Trie::defVal) {
                        cacheNodes[cacheIndex++] = 0;
                    }
                    // child leaf: create new leaf node
                    else if (depth == nDagLevels - 2) {
                        cacheNodes[cacheIndex++] = create_leaf_node(pChild);
                        gTimerB += 1.0;
                    }
                    // child normal: go to child node
                    else {
                        path[++depth] = pChild;
                        break;
                    }
                }
            }
            while (depth > 0);
            // std::cout << "global timer a: " << gTimerA << '\n';
            // std::cout << "global timer b: " << gTimerB << '\n';
            // std::cout << "global timer c: " << gTimerC << '\n';
            if (bLog) {
                auto end = std::chrono::steady_clock::now();
                auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
                std::cout << "trie iter: " << dur << " ms" << std::endl;
            }

            size_t nUniques = 0;
            size_t nDupes = 0;
            size_t nBytes = 0;
            for (size_t i = 0; i < uniques.size(); i++) {
                // std::cout << "Level " << i << ": "
                //     << dagLevels[i].data.size() * sizeof(uint32_t) << " bytes, "
                //     << uniques[i] << " uniques, " << dupes[i] << " dupes\n";
                nUniques += uniques[i];
                nDupes += dupes[i];
                nBytes += dagLevels[i].data.size() * sizeof(uint32_t);
            }
            std::cout << "Memory footprint: " << nBytes << " bytes (" << (double)nBytes / 1'000'000 << " MB)\n";

            if (bLog) {
                std::cout << "Total: " << nUniques << " uniques, " << nDupes << " dupes\n";
                size_t pointsBytes = points.size() * sizeof(Eigen::Vector3f);
                std::cout << "Pointcloud footprint: " << pointsBytes << " bytes (" << (double)pointsBytes / 1'000'000 << "MB)\n";
            }

            // Eigen::Vector3d acc = {0, 0, 0};
            // for (auto& point: points) {
            //     acc += point.cast<double>();
            // }
            // acc /= (double)points.size();
            // std::cout << acc << std::endl;
        }

    private:
        std::array<DAG::Level, nDagLevels> dagLevels;
        std::array<uint32_t, nDagLevels> uniques = {};
        std::array<uint32_t, nDagLevels> dupes = {};
    };
}