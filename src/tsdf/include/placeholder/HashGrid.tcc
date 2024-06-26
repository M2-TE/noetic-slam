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

// temporary
#include "/root/repo/src/tsdf/ext/morton-nd/include/morton-nd/mortonND_BMI2.h"

namespace lvr2
{

template <typename BaseVecT, typename BoxT>
HashGrid<BaseVecT, BoxT>::HashGrid(float cellSize,
                                   BoundingBox<BaseVecT> boundingBox,
                                   bool isVoxelsize,
                                   bool extrude)
    : GridBase(extrude), m_boundingBox(boundingBox), m_globalIndex(0)
{
    m_coordinateScales = BaseVecT(1, 1, 1);

    auto newMax = m_boundingBox.getMax();
    auto newMin = m_boundingBox.getMin();
    if (m_boundingBox.getXSize() < 3 * cellSize)
    {
        newMax.x += cellSize;
        newMin.x -= cellSize;
    }
    if (m_boundingBox.getYSize() < 3 * cellSize)
    {
        newMax.y += cellSize;
        newMin.y -= cellSize;
    }
    if (m_boundingBox.getZSize() < 3 * cellSize)
    {
        newMax.z += cellSize;
        newMin.z -= cellSize;
    }
    m_boundingBox.expand(newMax);
    m_boundingBox.expand(newMin);

    if (!m_boundingBox.isValid())
    {
        cout << timestamp << "Warning: Malformed BoundingBox." << endl;
    }

    if (!isVoxelsize)
    {
        m_voxelsize = (float)m_boundingBox.getLongestSide() / cellSize;
    }
    else
    {
        m_voxelsize = cellSize;
    }

    cout << timestamp << "Used voxelsize is " << m_voxelsize << endl;

    if (!m_extrude)
    {
        cout << timestamp << "Grid is not extruded." << endl;
    }

    BoxT::m_voxelsize = m_voxelsize;
    calcIndices();
}

template <typename BaseVecT, typename BoxT>
HashGrid<BaseVecT, BoxT>::HashGrid(string file)
{
    ifstream ifs(file.c_str());
    float minx, miny, minz, maxx, maxy, maxz, vsize;
    size_t qsize, csize;

    ifs >> m_extrude;
    m_extrude = false;
    ifs >> minx >> miny >> minz >> maxx >> maxy >> maxz >> qsize >> vsize >> csize;

    m_boundingBox = BoundingBox<BaseVecT>(BaseVecT(minx, miny, minz), BaseVecT(maxx, maxy, maxz));
    m_globalIndex = 0;
    m_coordinateScales.x = 1.0;
    m_coordinateScales.y = 1.0;
    m_coordinateScales.z = 1.0;
    m_voxelsize = vsize;
    BoxT::m_voxelsize = m_voxelsize;
    calcIndices();

    float pdist;
    BaseVecT v;
    // cout << timestamp << "Creating Grid..." << endl;

    // Iterator over all points, calc lattice indices and add lattice points to the grid
    for (size_t i = 0; i < qsize; i++)
    {

        ifs >> v.x >> v.y >> v.z >> pdist;

        QueryPoint<BaseVecT> qp(v, pdist);
        m_queryPoints.push_back(qp);
    }
    // cout << timestamp << "read qpoints.. csize: " << csize << endl;
    size_t h;
    unsigned int cell[8];
    BaseVecT cell_center;
    bool fusion = false;
    for (size_t k = 0; k < csize; k++)
    {
        // cout << "i: " << k << endl;
        ifs >> h >> cell[0] >> cell[1] >> cell[2] >> cell[3] >> cell[4] >> cell[5] >> cell[6] >>
            cell[7] >> cell_center.x >> cell_center.y >> cell_center.z >> fusion;
        BoxT* box = new BoxT(cell_center);
        box->m_extruded = fusion;
        for (int j = 0; j < 8; j++)
        {
            box->setVertex(j, cell[j]);
        }

        m_cells[h] = box;
    }
    cout << timestamp << "Reading cells.." << endl;
    typename HashGrid<BaseVecT, BoxT>::box_map_it it;
    typename HashGrid<BaseVecT, BoxT>::box_map_it neighbor_it;

    cout << "c size: " << m_cells.size() << endl;
    for (it = m_cells.begin(); it != m_cells.end(); it++)
    {
        // cout << "asdfsdfgsdfgdsfgdfg" << endl;
        BoxT* currentBox = it->second;
        int neighbor_index = 0;
        size_t neighbor_hash = 0;

        for (int a = -1; a < 2; a++)
        {
            for (int b = -1; b < 2; b++)
            {
                for (int c = -1; c < 2; c++)
                {

                    // Calculate hash value for current neighbor cell
                    int idx = calcIndex((it->second->getCenter()[0] - m_boundingBox.getMin()[0]) /
                                        m_voxelsize);
                    int idy = calcIndex((it->second->getCenter()[1] - m_boundingBox.getMin()[1]) /
                                        m_voxelsize);
                    int idz = calcIndex((it->second->getCenter()[2] - m_boundingBox.getMin()[2]) /
                                        m_voxelsize);
                    neighbor_hash = this->hashValue(idx + a, idy + b, idz + c);
                    // cout << "n hash: " << neighbor_hash  << endl;
                    // cout << " id: " << neighbor_index << endl;

                    // Try to find this cell in the grid
                    neighbor_it = this->m_cells.find(neighbor_hash);

                    // If it exists, save pointer in box
                    if (neighbor_it != this->m_cells.end())
                    {
                        currentBox->setNeighbor(neighbor_index, (*neighbor_it).second);
                        (*neighbor_it).second->setNeighbor(26 - neighbor_index, currentBox);
                    }

                    neighbor_index++;
                }
            }
        }
    }
    cout << "Finished reading grid" << endl;
}

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

template <typename BaseVecT, typename BoxT>
HashGrid<BaseVecT, BoxT>::HashGrid(std::vector<string>& files,
                                   BoundingBox<BaseVecT>& boundingBox,
                                   float voxelsize)
    : m_boundingBox(boundingBox), m_voxelsize(voxelsize), m_globalIndex(0)
{
    unsigned int INVALID = BoxT::INVALID_INDEX;
    calcIndices();
    float distances[8];
    BaseVecT box_center;
    bool extruded;
    float vsh = 0.5 * this->m_voxelsize;
    for (int numFiles = 0; numFiles < files.size(); numFiles++)
    {
        unsigned int current_index = 0;
        cout << "Loading grid: " << numFiles << "/" << files.size() << endl;

        FILE* pFile = fopen(files[numFiles].c_str(), "rb");
        size_t numCells;
        size_t r = fread(&numCells, sizeof(size_t), 1, pFile);

        for (size_t cellCount = 0; cellCount < numCells; cellCount++)
        {
            r = fread(&(box_center[0]), sizeof(float), 1, pFile);
            r = fread(&(box_center[1]), sizeof(float), 1, pFile);
            r = fread(&(box_center[2]), sizeof(float), 1, pFile);

            r = fread(&extruded, sizeof(bool), 1, pFile);

            r = fread(&(distances[0]), sizeof(float), 8, pFile);

            size_t idx = calcIndex((box_center[0] - m_boundingBox.getMin()[0]) / m_voxelsize);
            size_t idy = calcIndex((box_center[1] - m_boundingBox.getMin()[1]) / m_voxelsize);
            size_t idz = calcIndex((box_center[2] - m_boundingBox.getMin()[2]) / m_voxelsize);
            size_t hash = hashValue(idx, idy, idz);
            auto cell_it = this->m_cells.find(hash);
            if (cell_it == this->m_cells.end() && !extruded)
            {
                BoxT* box = new BoxT(box_center);
                for (int i = 0; i < 8; i++)
                {
                    current_index = this->findQueryPoint(i, idx, idy, idz);
                    if (current_index != INVALID)
                        box->setVertex(i, current_index);
                    else
                    {
                        BaseVecT position(box_center[0] + box_creation_table[i][0] * vsh,
                                          box_center[1] + box_creation_table[i][1] * vsh,
                                          box_center[2] + box_creation_table[i][2] * vsh);
                        this->m_queryPoints.push_back(QueryPoint<BaseVecT>(position, distances[i]));
                        box->setVertex(i, this->m_globalIndex);
                        this->m_globalIndex++;
                    }
                }
                // Set pointers to the neighbors of the current box
                int neighbor_index = 0;
                size_t neighbor_hash = 0;

                for (int a = -1; a < 2; a++)
                {
                    for (int b = -1; b < 2; b++)
                    {
                        for (int c = -1; c < 2; c++)
                        {

                            // Calculate hash value for current neighbor cell
                            neighbor_hash = this->hashValue(idx + a, idy + b, idz + c);

                            // Try to find this cell in the grid
                            auto neighbor_it = this->m_cells.find(neighbor_hash);

                            // If it exists, save pointer in box
                            if (neighbor_it != this->m_cells.end())
                            {
                                box->setNeighbor(neighbor_index, (*neighbor_it).second);
                                (*neighbor_it).second->setNeighbor(26 - neighbor_index, box);
                            }

                            neighbor_index++;
                        }
                    }
                }

                this->m_cells[hash] = box;
            }
        }
        fclose(pFile);
    }
}

template <typename BaseVecT, typename BoxT>
HashGrid<BaseVecT, BoxT>::HashGrid(std::vector<string>& files,
                                   std::vector<BoundingBox<BaseVecT>> innerBoxes,
                                   BoundingBox<BaseVecT>& boundingBox,
                                   float voxelsize)
        : m_boundingBox(boundingBox), m_voxelsize(voxelsize), m_globalIndex(0)
{
    unsigned int INVALID = BoxT::INVALID_INDEX;
    calcIndices();
    float distances[8];
    BaseVecT box_center;
    bool extruded;
    float vsh = 0.5 * this->m_voxelsize;
    for (int numFiles = 0; numFiles < files.size(); numFiles++)
    {
        // get the min and max vector of the inner chunk bounding box
        BaseVecT innerChunkMin = innerBoxes.at(numFiles).getMin();
        BaseVecT innerChunkMax = innerBoxes.at(numFiles).getMax();

        unsigned int current_index = 0;
        cout << "Loading grid: " << numFiles << "/" << files.size() << endl;

        FILE* pFile = fopen(files[numFiles].c_str(), "rb");
        size_t numCells;
        size_t r = fread(&numCells, sizeof(size_t), 1, pFile);

        for (size_t cellCount = 0; cellCount < numCells; cellCount++)
        {
            r = fread(&(box_center[0]), sizeof(float), 1, pFile);
            r = fread(&(box_center[1]), sizeof(float), 1, pFile);
            r = fread(&(box_center[2]), sizeof(float), 1, pFile);

            r = fread(&extruded, sizeof(bool), 1, pFile);

            r = fread(&(distances[0]), sizeof(float), 8, pFile);

            // Check if the voxel is inside of our bounding box.
            // If not, we skip it, because some other chunk is responsible for the voxel.
            if(box_center.x < innerChunkMin.x || box_center.y < innerChunkMin.y || box_center.z < innerChunkMin.z ||
                    box_center.x > innerChunkMax.x || box_center.y > innerChunkMax.y || box_center.z > innerChunkMax.z )
            {
                continue;
            }

            size_t idx = calcIndex((box_center[0] - m_boundingBox.getMin()[0]) / m_voxelsize);
            size_t idy = calcIndex((box_center[1] - m_boundingBox.getMin()[1]) / m_voxelsize);
            size_t idz = calcIndex((box_center[2] - m_boundingBox.getMin()[2]) / m_voxelsize);
            size_t hash = hashValue(idx, idy, idz);
            auto cell_it = this->m_cells.find(hash);
            if (cell_it == this->m_cells.end() && !extruded)
            {
                BoxT* box = new BoxT(box_center);
                for (int i = 0; i < 8; i++)
                {
                    current_index = this->findQueryPoint(i, idx, idy, idz);
                    if (current_index != INVALID)
                        box->setVertex(i, current_index);
                    else
                    {
                        BaseVecT position(box_center[0] + box_creation_table[i][0] * vsh,
                                          box_center[1] + box_creation_table[i][1] * vsh,
                                          box_center[2] + box_creation_table[i][2] * vsh);
                        this->m_queryPoints.push_back(QueryPoint<BaseVecT>(position, distances[i]));
                        box->setVertex(i, this->m_globalIndex);
                        this->m_globalIndex++;
                    }
                }
                // Set pointers to the neighbors of the current box
                int neighbor_index = 0;
                size_t neighbor_hash = 0;

                for (int a = -1; a < 2; a++)
                {
                    for (int b = -1; b < 2; b++)
                    {
                        for (int c = -1; c < 2; c++)
                        {

                            // Calculate hash value for current neighbor cell
                            neighbor_hash = this->hashValue(idx + a, idy + b, idz + c);

                            // Try to find this cell in the grid
                            auto neighbor_it = this->m_cells.find(neighbor_hash);

                            // If it exists, save pointer in box
                            if (neighbor_it != this->m_cells.end())
                            {
                                box->setNeighbor(neighbor_index, (*neighbor_it).second);
                                (*neighbor_it).second->setNeighbor(26 - neighbor_index, box);
                            }

                            neighbor_index++;
                        }
                    }
                }

                this->m_cells[hash] = box;
            }
        }
        fclose(pFile);
    }
}

template<typename BaseVecT, typename BoxT>
HashGrid<BaseVecT, BoxT>::HashGrid(std::vector<PointBufferPtr> chunks,
                                   std::vector<BoundingBox<BaseVecT>> innerBoxes,
                                   BoundingBox<BaseVecT>& boundingBox,
                                   float voxelSize)
        : m_boundingBox(boundingBox), m_globalIndex(0)
{
    unsigned int INVALID = BoxT::INVALID_INDEX;
    m_voxelsize = voxelSize;
    calcIndices();
    size_t numCells;
    boost::shared_array<float> centers;
    boost::shared_array<float> queryPoints;
    boost::shared_array<int> extruded;
    float vsh = 0.5 * this->m_voxelsize;
    size_t counter = 1;
    std::cout << timestamp.getElapsedTime() << "Number of Chunks: "<< chunks.size()<< std::endl;
    std::string comment = timestamp.getElapsedTime() + "Loading grid ";
    lvr2::ProgressBar progress(chunks.size(), comment);
    for (PointBufferPtr chunk : chunks)
    {
        unsigned int current_index = 0;
        boost::optional<unsigned int> optNumCells =  chunk->getAtomic<unsigned int>("num_voxel");
        boost::optional<Channel<int>> optExtruded = chunk->getChannel<int>("extruded");
        boost::optional<Channel<float>> optTSDF = chunk->getFloatChannel("tsdf_values");
        centers = chunk->getPointArray();

        if(optNumCells && optExtruded && optTSDF)
        {
            queryPoints = optTSDF.get().dataPtr();
            extruded = optExtruded.get().dataPtr();
            numCells = optNumCells.get();

            // get the min and max vector of the inner chunk bounding box
            BaseVecT innerChunkMin = innerBoxes.at(counter - 1).getMin();
            BaseVecT innerChunkMax = innerBoxes.at(counter - 1).getMax();

            for(size_t cellCount = 0; cellCount < numCells; cellCount++)
            {
                // Check if the voxel is inside of our bounding box.
                // If not, we skip it, because some other chunk is responsible for the voxel.
                BaseVecT voxCenter = BaseVecT(centers[cellCount * 3 + 0], centers[cellCount * 3 + 1], centers[cellCount * 3 + 2]);
                if(voxCenter.x < innerChunkMin.x || voxCenter.y < innerChunkMin.y || voxCenter.z < innerChunkMin.z ||
                    voxCenter.x > innerChunkMax.x || voxCenter.y > innerChunkMax.y || voxCenter.z > innerChunkMax.z )
                {
                    continue;
                }

                size_t idx = calcIndex((centers[cellCount * 3 + 0] - m_boundingBox.getMin()[0]) / m_voxelsize);
                size_t idy = calcIndex((centers[cellCount * 3 + 1] - m_boundingBox.getMin()[1]) / m_voxelsize);
                size_t idz = calcIndex((centers[cellCount * 3 + 2] - m_boundingBox.getMin()[2]) / m_voxelsize);
                size_t hash = hashValue(idx, idy, idz);
                auto cell_it = this->m_cells.find(hash);
                if (cell_it == this->m_cells.end() && !extruded.get()[cellCount])
                {
                    BoxT* box = new BoxT(BaseVecT(centers[cellCount * 3 + 0], centers[cellCount * 3 + 1], centers[cellCount * 3 + 2]));
                    for (int i = 0; i < 8; i++)
                    {
                        current_index = this->findQueryPoint(i, idx, idy, idz);
                        if (current_index != INVALID)
                            box->setVertex(i, current_index);
                        else
                        {
                            BaseVecT position(centers[cellCount * 3 + 0] + box_creation_table[i][0] * vsh,
                                              centers[cellCount * 3 + 1] + box_creation_table[i][1] * vsh,
                                              centers[cellCount * 3 + 2] + box_creation_table[i][2] * vsh);
                            this->m_queryPoints.push_back(QueryPoint<BaseVecT>(position, queryPoints[cellCount * 8 + i]));
                            box->setVertex(i, this->m_globalIndex);
                            this->m_globalIndex++;
                        }
                    }
                    // Set pointers to the neighbors of the current box
                    int neighbor_index = 0;
                    size_t neighbor_hash = 0;

                    for (int a = -1; a < 2; a++)
                    {
                        for (int b = -1; b < 2; b++)
                        {
                            for (int c = -1; c < 2; c++)
                            {

                                // Calculate hash value for current neighbor cell
                                neighbor_hash = this->hashValue(idx + a, idy + b, idz + c);

                                // Try to find this cell in the grid
                                auto neighbor_it = this->m_cells.find(neighbor_hash);

                                // If it exists, save pointer in box
                                if (neighbor_it != this->m_cells.end())
                                {
                                    box->setNeighbor(neighbor_index, (*neighbor_it).second);
                                    (*neighbor_it).second->setNeighbor(26 - neighbor_index, box);
                                }

                                neighbor_index++;
                            }
                        }
                    }

                    this->m_cells[hash] = box;
                }
            }
        }
        else
        {
            std::cout << "WARNING: something went wrong while reconstructing multiple chunks. Please check if all channels are available." << std::endl;
        }
        ++counter;
        if(!timestamp.isQuiet())
            ++progress;
    }
    if(!timestamp.isQuiet())
        cout << endl;
}

template <typename BaseVecT, typename BoxT>
void HashGrid<BaseVecT, BoxT>::addLatticePoint(int index_x,
                                               int index_y,
                                               int index_z,
                                               float distance)
{
    size_t hash_value;

    unsigned int INVALID = BoxT::INVALID_INDEX;

    float vsh = 0.5 * this->m_voxelsize;

    // Some iterators for hash map accesses
    typename HashGrid<BaseVecT, BoxT>::box_map_it it;
    typename HashGrid<BaseVecT, BoxT>::box_map_it neighbor_it;

    // Values for current and global indices. Current refers to a
    // already present query point, global index is id that the next
    // created query point will get
    unsigned int current_index = 0;

    // Get min and max vertex of the point clouds bounding box
    auto v_min = this->m_boundingBox.getMin();
    auto v_max = this->m_boundingBox.getMax();

    int limit = this->m_extrude ? 1 : 0;
    for (int dx = -limit; dx <= limit; dx++)
    {
        for (int dy = -limit; dy <= limit; dy++)
        {
            for (int dz = -limit; dz <= limit; dz++)
            {
                hash_value = this->hashValue(index_x + dx, index_y + dy, index_z + dz);

                it = this->m_cells.find(hash_value);
                if (it == this->m_cells.end())
                {
                    // Calculate box center
                    BaseVecT box_center((index_x + dx) * this->m_voxelsize + v_min.x,
                                        (index_y + dy) * this->m_voxelsize + v_min.y,
                                        (index_z + dz) * this->m_voxelsize + v_min.z);

                    // Create new box
                    BoxT* box = new BoxT(box_center);

                    if (box_center[0] <= m_boundingBox.getMin().x + m_voxelsize * 5 ||
                        box_center[1] <= m_boundingBox.getMin().y + m_voxelsize * 5 ||
                        box_center[2] <= m_boundingBox.getMin().z + m_voxelsize * 5)
                    {
                        box->m_duplicate = true;
                    }
                    else if (box_center[0] >= m_boundingBox.getMax().x - m_voxelsize * 5 ||
                             box_center[1] >= m_boundingBox.getMax().y - m_voxelsize * 5 ||
                             box_center[2] >= m_boundingBox.getMax().z - m_voxelsize * 5)
                    {
                        box->m_duplicate = true;
                    }

          
                    // Setup the box itself
                    for (int k = 0; k < 8; k++)
                    {

                        // Find point in Grid
                        current_index =
                            this->findQueryPoint(k, index_x + dx, index_y + dy, index_z + dz);
                        // If point exist, save index in box
                        if (current_index != INVALID)
                            box->setVertex(k, current_index);

                        // Otherwise create new grid point and associate it with the current box
                        else
                        {
                            BaseVecT position(box_center.x + box_creation_table[k][0] * vsh,
                                              box_center.y + box_creation_table[k][1] * vsh,
                                              box_center.z + box_creation_table[k][2] * vsh);

                            qp_bb.expand(position);

                            this->m_queryPoints.push_back(QueryPoint<BaseVecT>(position, distance));
                            box->setVertex(k, this->m_globalIndex);
                            this->m_globalIndex++;
                        }
                    }
                    

                    // Set pointers to the neighbors of the current box
                    int neighbor_index = 0;
                    size_t neighbor_hash = 0;

                    for (int a = -1; a < 2; a++)
                    {
                        for (int b = -1; b < 2; b++)
                        {
                            for (int c = -1; c < 2; c++)
                            {

                                // Calculate hash value for current neighbor cell
                                neighbor_hash = this->hashValue(
                                    index_x + dx + a, index_y + dy + b, index_z + dz + c);

                                // Try to find this cell in the grid
                                neighbor_it = this->m_cells.find(neighbor_hash);

                                // If it exists, save pointer in box
                                if (neighbor_it != this->m_cells.end())
                                {
                                    box->setNeighbor(neighbor_index, (*neighbor_it).second);
                                    (*neighbor_it).second->setNeighbor(26 - neighbor_index, box);   
                                }

                                neighbor_index++;
                            }
                        }
                    }
                    this->m_cells[hash_value] = box;
                }
            }
        }
    }
}

template <typename BaseVecT, typename BoxT>
void HashGrid<BaseVecT, BoxT>::setCoordinateScaling(float x, float y, float z)
{
    m_coordinateScales.x = x;
    m_coordinateScales.y = y;
    m_coordinateScales.z = z;
}

template <typename BaseVecT, typename BoxT>
HashGrid<BaseVecT, BoxT>::~HashGrid()
{
    box_map_it iter;
    for (iter = m_cells.begin(); iter != m_cells.end(); iter++)
    {
        if (iter->second != NULL)
        {
            delete (iter->second);
            iter->second = NULL;
        }
    }

    m_cells.clear();
}

template <typename BaseVecT, typename BoxT>
void HashGrid<BaseVecT, BoxT>::calcIndices()
{
    float max_size = m_boundingBox.getLongestSide();

    // Save needed grid parameters
    m_maxIndex = (int)ceil((max_size + 5 * m_voxelsize) / m_voxelsize);
    m_maxIndexSquare = m_maxIndex * m_maxIndex;

    m_maxIndexX = (int)ceil(m_boundingBox.getXSize() / m_voxelsize) + 1;
    m_maxIndexY = (int)ceil(m_boundingBox.getYSize() / m_voxelsize) + 2;
    m_maxIndexZ = (int)ceil(m_boundingBox.getZSize() / m_voxelsize) + 3;
}

template <typename BaseVecT, typename BoxT>
unsigned int HashGrid<BaseVecT, BoxT>::findQueryPoint(int position, int x, int y, int z)
{
    int n_x, n_y, n_z, q_v, offset;
    box_map_it it;

    for (int i = 0; i < 7; i++)
    {
        offset = i * 4;
        n_x = x + shared_vertex_table[position][offset];
        n_y = y + shared_vertex_table[position][offset + 1];
        n_z = z + shared_vertex_table[position][offset + 2];
        q_v = shared_vertex_table[position][offset + 3];

        size_t hash = hashValue(n_x, n_y, n_z);
        // cout << "i=" << i << " looking for hash: " << hash << endl;
        it = m_cells.find(hash);
        if (it != m_cells.end())
        {
            //  cout << "found hash" << endl;
            BoxT* b = it->second;
            if (b->getVertex(q_v) != BoxT::INVALID_INDEX)
                return b->getVertex(q_v);
        }
        // cout << "did not find hash" << endl;
    }

    return BoxT::INVALID_INDEX;
}

template <typename BaseVecT, typename BoxT>
void HashGrid<BaseVecT, BoxT>::saveGrid(string filename)
{
    std::cout << timestamp << "Writing grid..." << std::endl;

    // Open file for writing
    std::ofstream out(filename.c_str());

    // Write data
    if (out.good())
    {
        // Write header
        out << m_queryPoints.size() << " " << m_voxelsize << " " << m_cells.size() << endl;

        // Write query points and distances
        for (size_t i = 0; i < m_queryPoints.size(); i++)
        {
            out << m_queryPoints[i].m_position.x << " " << m_queryPoints[i].m_position.y << " "
                << m_queryPoints[i].m_position.z << " ";

            if (!isnan(m_queryPoints[i].m_distance))
            {
                out << m_queryPoints[i].m_distance << std::endl;
            }
            else
            {
                out << 0 << std::endl;
            }
        }

        // Write box definitions
        typename unordered_map<size_t, BoxT*>::iterator it;
        BoxT* box;
        for (it = m_cells.begin(); it != m_cells.end(); it++)
        {
            box = it->second;
            for (int i = 0; i < 8; i++)
            {
                out << box->getVertex(i) << " ";
            }
            out << std::endl;
        }
    }
}

template <typename BaseVecT, typename BoxT>
void HashGrid<BaseVecT, BoxT>::saveCells(string file)
{
    FILE* pFile = fopen(file.c_str(), "wb");
    size_t csize = m_cells.size();
    fwrite(&csize, sizeof(size_t), 1, pFile);
    for (auto it = this->firstCell(); it != this->lastCell(); it++)
    {
        fwrite(&it->second->getCenter()[0], sizeof(float), 1, pFile);
        fwrite(&it->second->getCenter()[1], sizeof(float), 1, pFile);
        fwrite(&it->second->getCenter()[2], sizeof(float), 1, pFile);

        fwrite(&it->second->m_extruded, sizeof(bool), 1, pFile);

        fwrite(&m_queryPoints[it->second->getVertex(0)].m_distance, sizeof(float), 1, pFile);
        fwrite(&m_queryPoints[it->second->getVertex(1)].m_distance, sizeof(float), 1, pFile);
        fwrite(&m_queryPoints[it->second->getVertex(2)].m_distance, sizeof(float), 1, pFile);
        fwrite(&m_queryPoints[it->second->getVertex(3)].m_distance, sizeof(float), 1, pFile);
        fwrite(&m_queryPoints[it->second->getVertex(4)].m_distance, sizeof(float), 1, pFile);
        fwrite(&m_queryPoints[it->second->getVertex(5)].m_distance, sizeof(float), 1, pFile);
        fwrite(&m_queryPoints[it->second->getVertex(6)].m_distance, sizeof(float), 1, pFile);
        fwrite(&m_queryPoints[it->second->getVertex(7)].m_distance, sizeof(float), 1, pFile);
    }
    fclose(pFile);
}
// <<<<<<< HEAD
// =======
// template <typename BaseVecT, typename BoxT>
// void HashGrid<BaseVecT, BoxT>::saveCellsHDF5(string file, string groupName)
// {
//     lvr2::ChunkIO chunkIo = ChunkIO(file);

//     chunkIo.writeVoxelSize(m_voxelsize);
//     size_t csize = getNumberOfCells();

//     boost::shared_array<float> centers(new float[3 * csize]);
//     boost::shared_array<bool> extruded(new bool[csize]);
//     boost::shared_array<float> queryPoints(new float[8 * csize]);

//     int counter = 0;
//     for (auto it = firstCell(); it != lastCell(); it++)
//     {
//         for (int j = 0; j < 3; ++j)
//         {
//             centers[3 * counter + j] = it->second->getCenter()[j];
//         }

//         extruded[counter] = it->second->m_extruded;

//         for (int k = 0; k < 8; ++k)
//         {
//             queryPoints[8 * counter + k] = m_queryPoints[it->second->getVertex(k)].m_distance;
//         }
//         ++counter;
//     }
//     chunkIo.writeTSDF(groupName, csize, centers, extruded, queryPoints);
// }
// >>>>>>> feature/scan_project_io_fix

template <typename BaseVecT, typename BoxT>
void HashGrid<BaseVecT, BoxT>::serialize(string file)
{
    std::cout << timestamp << "saving grid: " << file << std::endl;
    std::ofstream out(file.c_str());

    // Write data
    if (out.good())
    {
        out << m_extrude << std::endl;
        out << m_boundingBox.getMin().x << " " << m_boundingBox.getMin().y << " "
            << m_boundingBox.getMin().z << " " << m_boundingBox.getMax().x << " "
            << m_boundingBox.getMax().y << " " << m_boundingBox.getMax().z << std::endl;

        out << m_queryPoints.size() << " " << m_voxelsize << " " << m_cells.size() << endl;

        // Write query points and distances
        for (size_t i = 0; i < m_queryPoints.size(); i++)
        {
            out << m_queryPoints[i].m_position.x << " " << m_queryPoints[i].m_position.y << " "
                << m_queryPoints[i].m_position.z << " ";

            if (!isnan(m_queryPoints[i].m_distance))
            {
                out << m_queryPoints[i].m_distance << std::endl;
            }
            else
            {
                out << 0 << endl;
            }
        }

        // Write box definitions
        typename unordered_map<size_t, BoxT*>::iterator it;
        BoxT* box;
        for (it = m_cells.begin(); it != m_cells.end(); it++)
        {
            box = it->second;
            out << it->first << " ";
            for (int i = 0; i < 8; i++)
            {
                out << box->getVertex(i) << " ";
            }
            out << box->getCenter().x << " " << box->getCenter().y << " " << box->getCenter().z
                << " " << box->m_extruded << endl;
        }
    }
    out.close();
    std::cout << timestamp << "finished saving grid: " << file << std::endl;
}

template <typename BaseVecT, typename BoxT>
void HashGrid<BaseVecT, BoxT>::setBB(BoundingBox<BaseVecT>& bb)
{
    m_boundingBox = bb;
    calcIndices();
}

} // namespace lvr2
