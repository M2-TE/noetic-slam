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
#include <cstddef>
#include <fstream>
#include <iostream>

#include "/root/repo/src/tsdf_map/ext/mortonnd/include/morton-nd/mortonND_BMI2.h"
#include "/root/repo/src/tsdf_map/include/dag/leaf_cluster.hpp"
#include "/root/repo/src/tsdf_map/include/dag/node.hpp"

namespace lvr2
{

template <typename BaseVecT, typename BoxT>
HashGrid<BaseVecT, BoxT>::HashGrid(std::vector<std::vector<uint32_t>*>& nodeLevels, float voxelsize)
{
    m_globalIndex = 0;
    m_coordinateScales.x = 1.0;
    m_coordinateScales.y = 1.0;
    m_coordinateScales.z = 1.0;
    m_voxelsize = voxelsize;
    BoxT::m_voxelsize = m_voxelsize;

    std::cout << "-> Constructing HashGrid from hashDAG" << std::endl;

    // create iteration thingies
    std::array<uint8_t, 63/3> path;
    std::array<Node*, 63/3> nodes;
    path.fill(0);
    nodes.fill(nullptr);
    // initialize
    int64_t depth = 0;
    nodes[0] = Node::conv(*nodeLevels[0], 1);

    // begin traversal
    std::cout << timestamp << "Creating Grid..." << std::endl;
    while(depth >= 0) {
        auto iChild = path[depth]++;
        if (iChild >= 8) {
            depth--;
        }
        else if (depth < 63/3 - 1) {
            // read current parent node
            auto* pNode = nodes[depth];
            if (pNode->contains_child(iChild)) {
                uint32_t childAddr = pNode->get_child_addr(iChild);
                depth++;
                path[depth] = 0;
                nodes[depth] = Node::conv(*nodeLevels[depth], childAddr);
            }
        }
        else {
            // construct helper class for leaf cluster data
            auto* pNode = nodes[depth];
            if (!pNode->contains_child(iChild)) continue;
            uint32_t childAddr = pNode->get_child_addr(iChild);
            auto& leafLevel = *nodeLevels.back();
            LeafCluster leafCluster(leafLevel[childAddr + 0], leafLevel[childAddr + 1]);
            
            // reconstruct morton code from path
            uint64_t mortonCode = 0;
            for (uint64_t k = 0; k < 63/3; k++) {
                // TODO: verify that path[k] is never 0 before subtracting from it!
                uint64_t part = path[k] - 1;
                mortonCode |= part << (60 - k*3);
            }
            
            // revert shift on insertion
            mortonCode = mortonCode << 3;
            
            // morton codes were inserted via cluster pos, not leaf pos
            Eigen::Vector3i cluster_chunk;
            std::tie(cluster_chunk.x(), cluster_chunk.y(), cluster_chunk.z()) = mortonnd::MortonNDBmi_3D_64::Decode(mortonCode);
            // convert from 21-bit inverted to 32-bit integer
            cluster_chunk = cluster_chunk.unaryExpr([](auto i){ return i - (1 << 20); });
            
            uint32_t iLeaf = 0;
            for (int32_t z = 0; z < 2; z++) {
            for (int32_t y = 0; y < 2; y++) {
            for (int32_t x = 0; x < 2; x++, iLeaf++) {
                // leaf position
                Eigen::Vector3i leaf_chunk = cluster_chunk + Eigen::Vector3i(x, y, z);
                Eigen::Vector3f leaf_pos = leaf_chunk.cast<float>() * m_voxelsize;
                
                auto sdOpt = leafCluster.get_sd(iLeaf);
                if (!sdOpt) continue; // skip invalid leaves
                float signedDistance = *sdOpt;
                // float signedDistancePerfect = leaf_pos.cast<double>().norm() - 5.0f;
                // signedDistance = signedDistancePerfect;
                // std::cout << signedDistance << '\t' << signedDistancePerfect << '\n';
                
                // create query point
                size_t qIndex = m_queryPoints.size();
                m_queryPoints.emplace_back(BaseVecT(leaf_pos.x(), leaf_pos.y(), leaf_pos.z()), signedDistance);
                
                // // 1 cell for the query point
                // {
                //     Eigen::Vector3f cell_centerf = leaf_pos + Eigen::Vector3f(.5, .5, .5) * m_voxelsize;
                //     BoxT* pBox = new BoxT(BaseVecT(cell_centerf.x(), cell_centerf.y(), cell_centerf.z()));
                //     // create morton code of cell
                //     float recip = 1.0 / m_voxelsize;
                //     Eigen::Vector3i leafPosition = (cell_centerf * recip).cast<int32_t>();
                //     uint32_t xCell = (1 << 20) + (uint32_t)leafPosition.x();
                //     uint32_t yCell = (1 << 20) + (uint32_t)leafPosition.y();
                //     uint32_t zCell = (1 << 20) + (uint32_t)leafPosition.z();
                //     uint64_t mc = mortonnd::MortonNDBmi_3D_64::Encode(xCell, yCell, zCell);
                //     // emplace cell into map, check if it already existed
                //     auto [iter, _] = m_cells.emplace(mc, pBox);
                //     // place query point at the correct cell index
                //     iter->second->setVertex(0, qIndex);
                // }
                
                // 8 cells around the query point
                std::array<Eigen::Vector3f, 8> cellOffsets = {
                    Eigen::Vector3f(+0.5, +0.5, +0.5), 
                    Eigen::Vector3f(-0.5, +0.5, +0.5),
                    Eigen::Vector3f(-0.5, -0.5, +0.5), 
                    Eigen::Vector3f(+0.5, -0.5, +0.5),
                    Eigen::Vector3f(+0.5, +0.5, -0.5), 
                    Eigen::Vector3f(-0.5, +0.5, -0.5),
                    Eigen::Vector3f(-0.5, -0.5, -0.5), 
                    Eigen::Vector3f(+0.5, -0.5, -0.5),
                };
                for (size_t i = 0; i < 8; i++) {
                    // create cell
                    Eigen::Vector3f cell_centerf = leaf_pos + cellOffsets[i] * m_voxelsize;
                    // create morton code of cell
                    float recip = 1.0 / m_voxelsize;
                    // convert position back to chunk index
                    Eigen::Vector3f leafPosF = cell_centerf * recip;
                    leafPosF = leafPosF.unaryExpr([](float f){ return std::floor(f); });
                    Eigen::Vector3i leafPosition = leafPosF.cast<int32_t>();
                    // prepare for morton code conversion
                    uint32_t xCell = (1 << 20) + (uint32_t)leafPosition.x();
                    uint32_t yCell = (1 << 20) + (uint32_t)leafPosition.y();
                    uint32_t zCell = (1 << 20) + (uint32_t)leafPosition.z();
                    uint64_t mc = mortonnd::MortonNDBmi_3D_64::Encode(xCell, yCell, zCell);
                    // emplace cell into map, check if it already existed
                    BoxT* pBox = new BoxT(BaseVecT(cell_centerf.x(), cell_centerf.y(), cell_centerf.z()));
                    auto [iter, bEmplaced] = m_cells.emplace(mc, pBox);
                    if (!bEmplaced) delete pBox;
                    // place query point at the correct cell index
                    iter->second->setVertex(i, qIndex);
                }
            }}}
        }
    }
    
    // // iterate over cells to set neighbours and remaining cell vertices
    // std::cout << timestamp << "processing " << m_cells.size() << " cells..." << std::endl;
    // for (auto it = m_cells.cbegin(); it != m_cells.cend(); it++) {
    //     BoxT* p_cell = it->second;
    //     auto center_v = p_cell->getCenter();
    //     Eigen::Vector3f center_f { center_v.x, center_v.y, center_v.z };
    //     float voxelsPerUnit = 1.0 / m_voxelsize;
    //     Eigen::Vector3i center_i = (center_f * voxelsPerUnit).cast<int32_t>();
    //     // neighbour vertex lookup tables (forbidden technique)
    //     // indices in current cell
    //     constexpr uint8_t lookup_A[] = {
    //                     //  x  y  z (neighbor pos)
    //         0, 0, 0, 0, // -1 -1 -1
    //         0, 4, 0, 4, // -1 -1 +0
    //         4, 4, 4, 4, // -1 -1 +1
    //         0, 3, 0, 3, // -1 +0 -1
    //         0, 3, 4, 7, // -1 +0 +0
    //         4, 7, 4, 7, // -1 +0 +1
    //         3, 3, 3, 3, // -1 +1 -1
    //         3, 7, 3, 7, // -1 +1 +0
    //         7, 7, 7, 7, // -1 +1 +1
    //         //
    //         0, 1, 0, 1, // +0 -1 -1
    //         0, 1, 4, 5, // +0 -1 +0
    //         4, 5, 4, 5, // +0 -1 +1
    //         0, 1, 3, 2, // +0 +0 -1
    //         0, 0, 0, 0, // +0 +0 +0
    //         4, 5, 7, 6, // +0 +0 +1
    //         3, 2, 3, 2, // +0 +1 -1
    //         3, 2, 7, 6, // +0 +1 +0
    //         7, 6, 7, 6, // +0 +1 +1
    //         //
    //         1, 1, 1, 1, // +1 -1 -1
    //         1, 5, 1, 5, // +1 -1 +0
    //         5, 5, 5, 5, // +1 -1 +1
    //         1, 2, 1, 2, // +1 +0 -1
    //         1, 2, 5, 6, // +1 +0 +0
    //         5, 6, 5, 6, // +1 +0 +1
    //         2, 2, 2, 2, // +1 +1 -1
    //         2, 6, 2, 6, // +1 +1 +0
    //         6, 6, 6, 6, // +1 +1 +1
    //     };
    //     // indices in neighbor cell
    //     constexpr uint8_t lookup_B[] = {
    //                     //  x  y  z (neighbor pos)
    //         6, 6, 6, 6, // -1 -1 -1
    //         2, 6, 2, 6, // -1 -1 +0
    //         2, 2, 2, 2, // -1 -1 +1
    //         5, 6, 5, 6, // -1 +0 -1
    //         1, 2, 5, 6, // -1 +0 +0
    //         1, 2, 1, 2, // -1 +0 +1
    //         5, 5, 5, 5, // -1 +1 -1
    //         1, 5, 1, 5, // -1 +1 +0
    //         1, 1, 1, 1, // -1 +1 +1
    //         //
    //         7, 6, 7, 6, // +0 -1 -1
    //         3, 2, 7, 6, // +0 -1 +0
    //         3, 2, 3, 2, // +0 -1 +1
    //         4, 5, 7, 6, // +0 +0 -1
    //         0, 0, 0, 0, // +0 +0 +0
    //         0, 1, 3, 2, // +0 +0 +1
    //         4, 5, 4, 5, // +0 +1 -1
    //         0, 1, 4, 5, // +0 +1 +0
    //         0, 1, 0, 1, // +0 +1 +1
    //         //
    //         7, 7, 7, 7, // +1 -1 -1
    //         3, 7, 3, 7, // +1 -1 +0
    //         3, 3, 3, 3, // +1 -1 +1
    //         4, 7, 4, 7, // +1 +0 -1
    //         0, 3, 4, 7, // +1 +0 +0
    //         0, 3, 0, 3, // +1 +0 +1
    //         4, 4, 4, 4, // +1 +1 -1
    //         0, 4, 0, 4, // +1 +1 +0
    //         0, 0, 0, 0, // +1 +1 +1
    //     };
    //     size_t i_neighbour = 0;
    //     for (int x = -1; x <= 1; x++) {
    //         for (int y = -1; y <= 1; y++) {
    //             for (int z = -1; z <= 1; z++) {
    //                 // only need to find neighbour if there isnt one already
    //                 if (p_cell->getNeighbor(i_neighbour) != nullptr) {
    //                     i_neighbour++;
    //                     continue;
    //                 }
    //                 Eigen::Vector3i neigh_i = center_i + Eigen::Vector3i(x, y, z);
    //                 uint32_t xCell = (1 << 20) + (uint32_t)neigh_i.x();
    //                 uint32_t yCell = (1 << 20) + (uint32_t)neigh_i.y();
    //                 uint32_t zCell = (1 << 20) + (uint32_t)neigh_i.z();
    //                 uint64_t morton_code = mortonnd::MortonNDBmi_3D_64::Encode(xCell, yCell, zCell);
    //                 // find neighbor and exchange phone numbers
    //                 auto it_neighbour = m_cells.find(morton_code);
    //                 if (it_neighbour != nullptr) {
    //                     auto p_neig = it_neighbour->second;
    //                     p_cell->setNeighbor(i_neighbour, p_neig);
    //                     p_neig->setNeighbor(26 - i_neighbour, p_cell);
    //                     // update aliasing vertices
    //                     auto i_lookup = i_neighbour * 4;
    //                     for (auto i = 0; i < 4; i++) {
    //                         uint8_t iv_cell = lookup_A[i_lookup + i];
    //                         uint8_t iv_neig = lookup_B[i_lookup + i];
    //                         // get vertex from both cells
    //                         size_t v_cell = p_cell->getVertex(iv_cell);
    //                         size_t v_neig = p_neig->getVertex(iv_neig);
    //                         // select the one that isnt invalid
    //                         size_t vertex = 0;
    //                         if (v_cell != BoxT::INVALID_INDEX) vertex = v_cell;
    //                         else vertex = v_neig;
    //                         // update vertices
    //                         p_cell->setVertex(iv_cell, vertex);
    //                         p_neig->setVertex(iv_neig, vertex);
    //                     }
    //                 }
    //                 i_neighbour++;
    //             }
    //         }
    //     }
    // }
    
    // cull incomplete cells
    std::vector<uint64_t> incompleteCells;
    for (auto it = m_cells.cbegin(); it != m_cells.cend(); it++) {
        bool incomplete = false;
        for (auto i = 0; i < 8; i++) {
            auto vertIndex = it->second->getVertex(i);
            if (vertIndex == BoxT::INVALID_INDEX) incomplete = true;
        }
        if (incomplete) incompleteCells.push_back(it->first);
    }
    std::cout << timestamp << "culling " << incompleteCells.size() << " cells..." << std::endl;
    for (auto it = incompleteCells.cbegin(); it != incompleteCells.cend(); it++) {
        auto cell = m_cells.find(*it);
        delete cell->second;
        m_cells.erase(cell);
    }
    std::cout << timestamp << "Grid Construction Complete" << std::endl;
}

} // namespace lvr2
