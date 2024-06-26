/**
 * Copyright (c) 2018, University Osnabrück
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University Osnabrück nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL University Osnabrück BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * HashGrid.cpp
 *
 *  Created on: 16.02.2011
 *      Author: Thomas Wiemann
 */

#include "lvr2/geometry/BaseMesh.hpp"
#include "lvr2/io/ChunkIO.hpp"
#include "lvr2/util/Progress.hpp"
#include "lvr2/util/Timestamp.hpp"
#include "lvr2/reconstruction/FastReconstructionTables.hpp"
#include "lvr2/reconstruction/HashGrid.hpp"

#include <bitset>
#include <fstream>
#include <iostream>

#include "/root/repo/src/tsdf/ext/morton-nd/include/morton-nd/mortonND_BMI2.h"
// #include "/root/repo/src/tsdf/include/placeholder/HashGridDag.tcc"

namespace lvr2
{

struct LeafCluster {
    typedef uint64_t ClusterT;
    typedef std::make_signed_t<ClusterT> ClusterS;
    typedef uint32_t PartT;
    LeafCluster(ClusterT cluster): cluster(cluster) {}
    LeafCluster(PartT part0, PartT part1): cluster((ClusterT)part0 | ((ClusterT)part1 << 32)) {}
    // LeafCluster(std::array<float, 8>& leaves): cluster(0) {
    //     for (ClusterT i = 0; i < 8; i++) {
    //         // normalize sd to [-1, 1]
    //         float sdNormalized = leaves[i] * (1.0 / maxDist);
            
    //         // scale up to fit into nBit integers
    //         float scale = (float)range;
    //         float sdScaled = sdNormalized * scale;
            
    //         // cast to 8-bit integer and clamp between given range
    //         int8_t sdScaledInt = (int8_t)sdScaled;
    //         sdScaledInt = std::clamp<int8_t>(sdScaledInt, -scale, scale);
            
    //         // add offset such that values are represented linearly from 0 to max
    //         uint8_t sdScaledUint = (uint8_t)(sdScaledInt + (int8_t)scale);
            
    //         // pack the 4 bits of this value into the leaf cluster
    //         cluster |= (ClusterT)sdScaledUint << i*nBits;
    //     }
    // }
    std::pair<PartT, PartT> get_parts() {
        PartT part0 = (PartT)cluster;
        PartT part1 = (PartT)(cluster >> 32);
        return { part0, part1 };
    }
    void merge(LeafCluster& other) {
        for (ClusterT i = 0; i < 8; i++) {
            // mask out bits for current leaf
            typedef std::make_signed_t<ClusterT> ClusterInt;
            int8_t maskedA = (this->cluster >> i*nBits) & leafMask;
            int8_t maskedB = (other.cluster >> i*nBits) & leafMask;
            
            // convert back to standard readable int
            int8_t a = maskedA - range;
            int8_t b = maskedB - range;
            
            // ruleset (in order of priority):
            // 1. positive signed distance takes precedence over negative
            // 2. smaller value takes precedence over larger value
            bool bOverwrite = false;
            if (std::signbit(a) > std::signbit(b)) bOverwrite = true;
            else if (std::signbit(a) == std::signbit(b) && std::abs(a) > std::abs(b)) bOverwrite = true;
            
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
    float get_sd(uint8_t index, float leafResolution) {
        // 4 bits precision for each leaf
        int8_t leaf = cluster >> index*nBits;
        leaf &= leafMask;
        // convert back to standard signed
        leaf -= (int8_t)range;
        // convert to floating signed distance
        float signedDistance = (float)leaf;
        signedDistance /= (float)range; // normalize signed distance
        signedDistance *= leafResolution; // scale signed distance to real size
        return signedDistance;
    }
    
    ClusterT cluster;
    // static constexpr float maxDist = leafResolution;
    static constexpr ClusterT nBits = 8; // 1b sign, rest data
    static constexpr ClusterT leafMask = (1 << nBits) - 1; // mask for a single leaf
    static constexpr ClusterT range = leafMask / 2; // achievable range with data bits
};

template <typename BaseVecT, typename BoxT>
HashGrid<BaseVecT, BoxT>::HashGrid(BoundingBox<BaseVecT> boundingBox, std::vector<std::vector<uint32_t>*>& nodeLevels, float voxelsize)
{
    m_boundingBox = boundingBox;
    m_globalIndex = 0;
    m_coordinateScales.x = 1.0;
    m_coordinateScales.y = 1.0;
    m_coordinateScales.z = 1.0;
    m_voxelsize = voxelsize;
    BoxT::m_voxelsize = m_voxelsize;
    // calcIndices();

    std::cout << "-> Constructing HashGrid from hashDAG" << std::endl;

    // init iteration thingies
    std::array<uint8_t, 63/3> path;
    std::array<uint32_t*, 63/3> nodes;
    path.fill(0);
    nodes.fill(0);
    uint32_t depth = 0;
    uint32_t iRoot = 1; // the first index is reserved
    nodes[0] = nodeLevels[0]->data() + iRoot;

    // begin traversal
    std::cout << timestamp << "Creating Grid..." << std::endl;
    while (true) {
        auto iChild = path[depth]++;
        if (iChild >= 8) { // every node has a max of 8 children
            // move to next root
            if (depth == 0) {
                // count how many children this root had
                uint32_t rootMask = *(nodeLevels[0]->data() + iRoot);
                uint32_t nChildren = rootMask >> 8; // mask contains child count further up
                std::cout << "root had " << (uint32_t)nChildren << " children\n";

                // go to next root
                iRoot += nChildren + 1;
                if (iRoot < nodeLevels[0]->size()) {
                    path.fill(0);
                    nodes.fill(0);
                    nodes[0] = nodeLevels[0]->data() + iRoot;
                }
                else break;
            }
            // go back up to parent
            else {
                depth--;
                continue;
            }
        }
        
        // read current parent node
        uint32_t* pParentNode = nodes[depth];
        // retrieve child mask from parent
        // nodes are e.g.: childMask, child4, child7
        // -> being 3x uint32 in total
        uint32_t childBit = 1 << iChild;
        
        // child will be a leaf cluster
        if (depth == 63/3 - 1) {
            // reconstruct morton code from path
            uint64_t mortonCode = 0;
            for (uint64_t k = 0; k < 63/3; k++) {
                uint64_t part = path[k] - 1;
                mortonCode |= part << (60 - k*3);
            }
            // std::cout << std::bitset<63>(mortonCode) << ' ' << (uint32_t)iChild << '\n';
            
            // morton codes were inserted via cluster pos, not leaf pos
            auto [x, y, z] = mortonnd::MortonNDBmi_3D_64::Decode(mortonCode);
            // convert from 21-bit inverted to 32-bit integer
            x -= 1 << 20;
            y -= 1 << 20;
            z -= 1 << 20;
            Eigen::Vector3i veci { (int32_t)x, (int32_t)y, (int32_t)z };
            veci /= 2;
            Eigen::Vector3f vecf = (veci).cast<float>() * m_voxelsize; // convert back to real position
            // std::cout << vecf.x() << ' ' << vecf.y() << ' ' << vecf.z() << '\n';
            
            // construct helper class for leaf cluster data
            LeafCluster leafCluster(*pParentNode, *(pParentNode+1));
            // std::cout << std::bitset<64>(leafCluster.cluster) << '\n';
            
            // iterate over packed leaves within leaf cluster
            uint32_t iLeaf = 0;
            for (auto xl = 0; xl < 2; xl++) {
                for (auto yl = 0; yl < 2; yl++) {
                    for (auto zl = 0; zl < 2; zl++) {
                        // leaf position
                        Eigen::Vector3f leafOffset = Eigen::Vector3f(xl, yl, zl) * m_voxelsize;
                        Eigen::Vector3f pos = vecf + leafOffset;
                        
                        float signedDistance = leafCluster.get_sd(iLeaf, m_voxelsize);
                        
                        // DEBUG
                        {
                            // std::cout << std::bitset<32>(leafCluster) << '\n';
                            // float magn = pos.norm();
                            // signedDistance = magn - 5.0f;
                            // std::cout << pos.x() << ' ' << pos.y() << ' ' << pos.z() << ": " << pos.norm() << ' ' << signedDistance << '\n';
                        }
                        
                        // create query point
                        size_t qIndex = m_queryPoints.size();
                        m_queryPoints.emplace_back(BaseVecT(pos.x(), pos.y(), pos.z()), signedDistance);

                        // create a cell for each query point (query point will be q0 of cell)
                        Eigen::Vector3f cell_centerf = pos + Eigen::Vector3f(m_voxelsize/2, m_voxelsize/2, m_voxelsize/2);
                        BoxT* box = new BoxT(BaseVecT(cell_centerf.x(), cell_centerf.y(), cell_centerf.z()));
                        box->setVertex(0, qIndex); // set only lower bottom left vertex

                        // hash cell as morton code
                        float voxelsPerUnit = 1.0 / m_voxelsize;
                        Eigen::Vector3i leafPosition = (cell_centerf * voxelsPerUnit).cast<int32_t>();
                        
                        // std::cout << leafPosition.x() << ' ' << leafPosition.y() << ' ' << leafPosition.z() << '\n';
                        uint32_t xCell = (1 << 20) + (uint32_t)leafPosition.x();
                        uint32_t yCell = (1 << 20) + (uint32_t)leafPosition.y();
                        uint32_t zCell = (1 << 20) + (uint32_t)leafPosition.z();
                        uint64_t mc = mortonnd::MortonNDBmi_3D_64::Encode(xCell, yCell, zCell);
                        m_cells.emplace(mc, box);
                        iLeaf++;
                    }
                }
            }
            depth--;
        }
        // see if child mask contains current iChild
        else if (*pParentNode & childBit) {
            // popcount lookup table: https://stackoverflow.com/a/51388543
            constexpr uint8_t bitcount[] = {
                0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
            };
            // count how many children come before this one
            uint8_t masked = *pParentNode & (childBit - 1);
            uint8_t nChildren = bitcount[masked];

            // fetch tree index of child and use it to access child node
            uint32_t index = *(pParentNode + nChildren + 1);
            
            // go down one level
            depth++;
            // fetch child
            uint32_t* pChild = nodeLevels[depth]->data() + index;

            // set child to node path and reset child tracker
            nodes[depth] = pChild;
            path[depth] = 0;
        }
    }

    // iterate over cells to set neighbours and remaining cell vertices
    std::cout << timestamp << "processing " << m_cells.size() << " cells..." << std::endl;
    for (auto it = m_cells.cbegin(); it != m_cells.cend(); it++) {
        BoxT* p_cell = it->second;
        
        auto center_v = p_cell->getCenter();
        Eigen::Vector3f center_f { center_v.x, center_v.y, center_v.z };
        float voxelsPerUnit = 1.0 / m_voxelsize;
        Eigen::Vector3i center_i = (center_f * voxelsPerUnit).cast<int32_t>();

        // neighbour vertex lookup tables (forbidden technique)
        // indices in current cell
        
#       if true
            // version A
            constexpr uint8_t lookup_A[] = {
                            //  x  y  z (neighbor pos)
                0, 0, 0, 0, // -1 -1 -1
                0, 4, 0, 4, // -1 -1 +0
                4, 4, 4, 4, // -1 -1 +1
                0, 3, 0, 3, // -1 +0 -1
                0, 3, 4, 7, // -1 +0 +0
                4, 7, 4, 7, // -1 +0 -1
                3, 3, 3, 3, // -1 +1 -1
                3, 7, 3, 7, // -1 +1 +0
                7, 7, 7, 7, // -1 +1 -1
                //
                0, 1, 0, 1, // +0 -1 -1
                0, 1, 4, 5, // +0 -1 +0
                4, 5, 4, 5, // +0 -1 +1
                0, 1, 3, 2, // +0 +0 -1
                0, 0, 0, 0, // +0 +0 +0
                4, 5, 7, 6, // +0 +0 -1
                3, 2, 3, 2, // +0 +1 -1
                3, 2, 7, 6, // +0 +1 +0
                7, 6, 7, 6, // +0 +1 -1
                //
                1, 1, 1, 1, // +1 -1 -1
                1, 5, 1, 5, // +1 -1 +0
                5, 5, 5, 5, // +1 -1 +1
                1, 2, 1, 2, // +1 +0 -1
                1, 2, 5, 6, // +1 +0 +0
                5, 6, 5, 6, // +1 +0 -1
                2, 2, 2, 2, // +1 +1 -1
                2, 6, 2, 6, // +1 +1 +0
                6, 6, 6, 6, // +1 +1 -1
            };
            // indices in neighbor cell
            constexpr uint8_t lookup_B[] = {
                            //  x  y  z
                6, 6, 6, 6, // -1 -1 -1
                2, 6, 2, 6, // -1 -1 +0
                2, 2, 2, 2, // -1 -1 +1
                5, 6, 5, 6, // -1 +0 -1
                1, 2, 5, 6, // -1 +0 +0
                1, 2, 1, 2, // -1 +0 -1
                5, 5, 5, 5, // -1 +1 -1
                1, 5, 1, 5, // -1 +1 +0
                1, 1, 1, 1, // -1 +1 -1
                //
                7, 6, 7, 6, // +0 -1 -1
                3, 2, 7, 6, // +0 -1 +0
                3, 2, 3, 2, // +0 -1 +1
                4, 5, 7, 6, // +0 +0 -1
                0, 0, 0, 0, // +0 +0 +0
                0, 1, 3, 2, // +0 +0 -1
                4, 5, 4, 5, // +0 +1 -1
                0, 1, 4, 5, // +0 +1 +0
                0, 1, 0, 1, // +0 +1 -1
                //
                7, 7, 7, 7, // +1 -1 -1
                3, 7, 3, 7, // +1 -1 +0
                3, 3, 3, 3, // +1 -1 +1
                4, 7, 4, 7, // +1 +0 -1
                0, 3, 5, 7, // +1 +0 +0
                0, 3, 0, 3, // +1 +0 -1
                4, 4, 4, 4, // +1 +1 -1
                0, 4, 0, 4, // +1 +1 +0
                0, 0, 0, 0, // +1 +1 -1
            };
#       else
            // version B with swapped z order
            constexpr uint8_t lookup_A[] = {
                            //  x  y  z (neighbor pos)
                4, 4, 4, 4, // -1 -1 -1 // todo: swap + and - on z (comment only)
                0, 4, 0, 4, // -1 -1 +0
                0, 0, 0, 0, // -1 -1 +1
                4, 7, 4, 7, // -1 +0 -1
                0, 3, 4, 7, // -1 +0 +0
                0, 3, 0, 3, // -1 +0 +1
                7, 7, 7, 7, // -1 +1 -1
                3, 7, 3, 7, // -1 +1 +0
                3, 3, 3, 3, // -1 +1 +1
                //
                4, 5, 4, 5, // +0 -1 -1
                0, 1, 4, 5, // +0 -1 +0
                0, 1, 0, 1, // +0 -1 +1
                4, 5, 7, 6, // +0 +0 -1
                0, 0, 0, 0, // +0 +0 +0
                0, 1, 3, 2, // +0 +0 +1
                7, 6, 7, 6, // +0 +1 -1
                3, 2, 7, 6, // +0 +1 +0
                3, 2, 3, 2, // +0 +1 +1
                //
                5, 5, 5, 5, // +1 -1 -1
                1, 5, 1, 5, // +1 -1 +0
                1, 1, 1, 1, // +1 -1 +1
                5, 6, 5, 6, // +1 +0 -1
                1, 2, 5, 6, // +1 +0 +0
                1, 2, 1, 2, // +1 +0 +1
                6, 6, 6, 6, // +1 +1 -1
                2, 6, 2, 6, // +1 +1 +0
                2, 2, 2, 2, // +1 +1 +1
            };
            // indices in neighbor cell
            constexpr uint8_t lookup_B[] = {
                            //  x  y  z
                2, 2, 2, 2, // -1 -1 -1
                2, 6, 2, 6, // -1 -1 +0
                6, 6, 6, 6, // -1 -1 +1
                1, 2, 1, 2, // -1 +0 -1
                1, 2, 5, 6, // -1 +0 +0
                5, 6, 5, 6, // -1 +0 +1
                1, 1, 1, 1, // -1 +1 -1
                1, 5, 1, 5, // -1 +1 +0
                5, 5, 5, 5, // -1 +1 +1
                //
                3, 2, 3, 2, // +0 -1 -1
                3, 2, 7, 6, // +0 -1 +0
                7, 6, 7, 6, // +0 -1 +1
                0, 1, 3, 2, // +0 +0 -1
                0, 0, 0, 0, // +0 +0 +0
                4, 5, 7, 6, // +0 +0 +1
                0, 1, 0, 1, // +0 +1 -1
                0, 1, 4, 5, // +0 +1 +0
                4, 5, 4, 5, // +0 +1 +1
                //
                3, 3, 3, 3, // +1 -1 -1
                3, 7, 3, 7, // +1 -1 +0
                7, 7, 7, 7, // +1 -1 +1
                0, 3, 0, 3, // +1 +0 -1
                0, 3, 5, 7, // +1 +0 +0
                4, 7, 4, 7, // +1 +0 +1
                0, 0, 0, 0, // +1 +1 -1
                0, 4, 0, 4, // +1 +1 +0
                4, 4, 4, 4, // +1 +1 +1
            };
#       endif

        size_t i_neighbour = 0;
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                for (int z = -1; z <= 1; z++) {
                    Eigen::Vector3i neigh_i = center_i + Eigen::Vector3i(x, y, z);
                    uint32_t xCell = (1 << 20) + (uint32_t)neigh_i.x();
                    uint32_t yCell = (1 << 20) + (uint32_t)neigh_i.y();
                    uint32_t zCell = (1 << 20) + (uint32_t)neigh_i.z();
                    uint64_t morton_code = mortonnd::MortonNDBmi_3D_64::Encode(xCell, yCell, zCell);
                    
                    // find neighbor and exchange phone numbers
                    auto it_neighbour = m_cells.find(morton_code);
                    if (it_neighbour != nullptr) {
                        auto p_neig = it_neighbour->second;
                        p_cell->setNeighbor(i_neighbour, p_neig);
                        p_neig->setNeighbor(26 - i_neighbour, p_cell);

                        // update aliasing vertices
                        auto i_lookup = i_neighbour * 4;
                        for (auto i = 0; i < 4; i++) {
                            uint8_t iv_cell = lookup_A[i_lookup];
                            uint8_t iv_neig = lookup_B[i_lookup];
                            // get vertex from both cells
                            auto v_cell = p_cell->getVertex(iv_cell);
                            auto v_neig = p_neig->getVertex(iv_neig);
                            // select the one that isnt invalid
                            auto vertex = 0;
                            if (v_cell != BoxT::INVALID_INDEX) vertex = v_cell;
                            else vertex = v_neig;
                            // update vertices
                            p_cell->setVertex(iv_cell, vertex);
                            p_neig->setVertex(iv_neig, vertex);
                        }
                    }
                    i_neighbour++;
                }
            }
        }
    }

    std::cout << timestamp << "flagging cells..." << std::endl;
    std::vector<size_t> invalid_cells;
    for (auto it = m_cells.cbegin(); it != m_cells.cend(); it++) {
        // iterate over all vertices and flag cell for removal if even one is invalid
        bool b_flagged = false;
        for (auto i = 0; i < 8; i++) {
            auto v = it->second->getVertex(i);
            if (v == BoxT::INVALID_INDEX) b_flagged = true;

            // also flag boxes with only negative or only positive signed distances?
            // if (m_queryPoints[v].m_distance < 0.0f) b_all_positive = false;
            // if (m_queryPoints[v].m_distance > 0.0f) b_all_negative = false;
        }
        if (b_flagged) invalid_cells.push_back(it->first);
    }
    // erase the cells that were flagged for removal
    std::cout << timestamp << "culling "<< invalid_cells.size() << " flagged cells..." << std::endl;
    for (auto it = invalid_cells.cbegin(); it != invalid_cells.cend(); it++) {
        auto it_cell = m_cells.find(*it);
        BoxT* p_cell = it_cell->second;

        // clear references to this cell from neigbors
        for (auto i = 0; i < 27; i++) {
            BoxT* p_neig = p_cell->getNeighbor(i);
            if (p_neig == nullptr) continue;
            for (auto k = 0; k < 27; k++) {
                if (p_neig->getNeighbor(k) == p_cell) {
                    p_neig->setNeighbor(k, nullptr);
                }
            }
        }

        delete p_cell;
        m_cells.erase(it_cell);
    }
    std::cout << timestamp << "Grid Construction Complete" << std::endl;
}
} // namespace lvr2
