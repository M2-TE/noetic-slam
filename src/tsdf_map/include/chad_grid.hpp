#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include <unordered_map>
#include <lvr2/geometry/PMPMesh.hpp>
#include <lvr2/geometry/BoundingBox.hpp>
#include <lvr2/reconstruction/HashGrid.hpp>
#include <lvr2/reconstruction/FastBox.hpp>
#include <lvr2/reconstruction/QueryPoint.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include <morton-nd/mortonND_BMI2.h>
#include "chad/leaf_cluster.hpp"
#include "chad/node.hpp"

template<typename BaseVecT, typename BoxT>
struct ChadGrid: public lvr2::GridBase {
    ChadGrid(std::array<std::vector<uint32_t>*, 63/3 - 1> node_levels, std::vector<LeafCluster::ClusterValue>& leaf_level, uint32_t root_addr, double voxel_res): lvr2::GridBase(false) {
        m_globalIndex = 0;
        m_coordinateScales.x = 1.0;
        m_coordinateScales.y = 1.0;
        m_coordinateScales.z = 1.0;
        m_voxelsize = voxel_res;
        BoxT::m_voxelsize = voxel_res;

        // trackers that will be updated during traversal
        static constexpr std::size_t max_depth = 63/3 - 1;
        std::array<uint8_t, max_depth> path;
        std::array<Node*, max_depth> nodes;
        path.fill(0);
        nodes.fill(nullptr);
        // initialize
        nodes[0] = Node::from_addr(*node_levels[0], root_addr);

        uint_fast32_t depth = 0;
        while(true) {
            auto child_i = path[depth]++;
            if (child_i == 8) {
                if (depth > 0) depth--;
                else break; // exit main loop
            }

            // normal node
            else if (depth < max_depth - 1) {
                // read current parent node
                auto* node_p = nodes[depth];
                if (node_p->contains_child(child_i)) {
                    uint32_t child_addr = node_p->get_child_addr(child_i);
                    depth++;
                    path[depth] = 0;
                    nodes[depth] = Node::from_addr(*node_levels[depth], child_addr);
                }
            }
            // leaf cluster node
            else {
                // skip if child does not exist
                auto* node_p = nodes[depth];
                if (!node_p->contains_child(child_i)) continue;
                uint32_t child_addr = node_p->get_child_addr(child_i);
                // construct helper class for leaf cluster data
                LeafCluster leaf_cluster{ leaf_level[child_addr] };

                // reconstruct morton code from path
                uint64_t code = 0;
                for (uint64_t k = 0; k < 63/3 - 1; k++) {
                    uint64_t part = path[k] - 1;
                    code |= part << (60 - k*3);
                }
                // convert into chunk position of leaf cluster
                Eigen::Vector3i cluster_chunk;
                std::tie(cluster_chunk.x(), cluster_chunk.y(), cluster_chunk.z()) = mortonnd::MortonNDBmi_3D_64::Decode(code);
                // convert from 21-bit inverted to 32-bit integer
                cluster_chunk = cluster_chunk.unaryExpr([](auto i){ return i - (1 << 20); });
                
                uint32_t leaf_i = 0;
                for (int32_t z = 0; z <= 1; z++) {
                for (int32_t y = 0; y <= 1; y++) {
                for (int32_t x = 0; x <= 1; x++, leaf_i++) {
                    // leaf position
                    Eigen::Vector3i leaf_chunk = cluster_chunk + Eigen::Vector3i(x, y, z);
                    Eigen::Vector3f leaf_pos = leaf_chunk.cast<float>() * m_voxelsize;
                    
                    auto sd_opt = leaf_cluster.get_leaf(leaf_i);
                    if (!sd_opt) continue; // skip invalid leaves
                    // signed distance for this leaf
                    float sd = sd_opt.value();
                    // float sd_perfect = leaf_pos.cast<double>().norm() - 5.0;
                    // sd_perfect = std::clamp(sd_perfect, -m_voxelsize, +m_voxelsize);
                    // sd = sd_perfect;
                    // std::cout << "perfect: " << sd_perfect << " actual: " << sd_opt.value() << '\n';
                    
                    // create query point
                    size_t querypoint_i = m_queryPoints.size();
                    m_queryPoints.emplace_back(BaseVecT(leaf_pos.x(), leaf_pos.y(), leaf_pos.z()), sd);
                    
                    // 8 cells around the query point
                    std::array<Eigen::Vector3f, 8> cell_offsets = {
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
                        Eigen::Vector3f cell_center = leaf_pos + cell_offsets[i] * m_voxelsize;
                        // create morton code of cell
                        const float recip = 1.0 / m_voxelsize;
                        // convert position back to chunk index
                        Eigen::Vector3f cell_pos = cell_center * recip;
                        cell_pos = cell_pos.unaryExpr([](float f){ return std::floor(f); });
                        Eigen::Vector3i cell_chunk = cell_pos.cast<int32_t>();
                        // convert to 21-bit ints
                        cell_chunk = cell_chunk.unaryExpr([](auto i){ return i + (1 << 20); });
                        uint64_t mc = mortonnd::MortonNDBmi_3D_64::Encode(cell_chunk.x(), cell_chunk.y(), cell_chunk.z());
                        // emplace cell into map, check if it already existed
                        auto [box_it, emplaced] = m_cells.emplace(mc, nullptr);
                        if (emplaced) {
                            box_it->second = new BoxT(BaseVecT(cell_center.x(), cell_center.y(), cell_center.z()));
                        }
                        // place query point at the correct cell index
                        box_it->second->setVertex(i, querypoint_i);
                    }
                }}}
            }
        }

        // cull incomplete cells
        std::vector<uint64_t> incomplete_cells;
        for (auto it = m_cells.cbegin(); it != m_cells.cend(); it++) {
            bool incomplete = false;
            for (auto i = 0; i < 8; i++) {
                auto vertIndex = it->second->getVertex(i);
                if (vertIndex == BoxT::INVALID_INDEX) incomplete = true;
            }
            if (incomplete) incomplete_cells.push_back(it->first);
        }
        for (auto it = incomplete_cells.cbegin(); it != incomplete_cells.cend(); it++) {
            auto cell = m_cells.find(*it);
            delete cell->second;
            m_cells.erase(cell);
        }
        std::cout << "ChadGrid created" << std::endl;
    }
    ~ChadGrid() {
        lvr2::GridBase::~GridBase();
    }

    auto getNumberOfCells() -> std::size_t { 
        return m_cells.size();
    }
    auto firstCell() -> typename std::unordered_map<size_t, BoxT*>::iterator { 
        return m_cells.begin(); 
    }
    auto lastCell() -> typename unordered_map<size_t, BoxT*>::iterator {
        return m_cells.end();
    }
    auto firstQueryPoint() -> typename std::vector<lvr2::QueryPoint<BaseVecT>>::iterator {
        return m_queryPoints.begin();
    }
    auto lastQueryPoint() -> typename std::vector<lvr2::QueryPoint<BaseVecT>>::iterator {
        return m_queryPoints.end();
    }
    auto getQueryPoints() -> vector<lvr2::QueryPoint<BaseVecT>>& {
        return m_queryPoints;
    }
    auto getCells() -> unordered_map<size_t, BoxT*>& {
        return m_cells;
    }

    // implement pure virtual functions
    void addLatticePoint(int i, int j, int k, float distance = 0.0) override {
        std::cout << "addLatticePoint called" << std::endl;
    }
    void saveGrid(std::string file) override {
        // store all the points into .grid file
        std::ofstream output;
        output.open(file, std::ofstream::trunc | std::ofstream::binary);
        // store header data
        float voxel_res = LEAF_RESOLUTION;
        output.write(reinterpret_cast<char*>(&voxel_res), sizeof(float));
        size_t query_points_n = getQueryPoints().size();
        output.write(reinterpret_cast<char*>(&query_points_n), sizeof(size_t));
        size_t cells_n = getNumberOfCells();
        output.write(reinterpret_cast<char*>(&cells_n), sizeof(size_t));
        // store query points (vec3 + float)
        auto& query_points = getQueryPoints();
        for (auto cur = query_points.cbegin(); cur != query_points.cend(); cur++) {
            Eigen::Vector3f pos { cur->m_position.x, cur->m_position.y, cur->m_position.z };
            float signed_distance = cur->m_distance;
            output.write(reinterpret_cast<const char*>(&pos), sizeof(Eigen::Vector3f));
            output.write(reinterpret_cast<const char*>(&signed_distance), sizeof(float));
        }
        // store cells (8x uint32_t)
        for (auto cur = firstCell(); cur != lastCell(); cur++) {
            auto* cell = cur->second;
            for (size_t i = 0; i < 8; i++) {
                uint32_t i_query_point = cell->getVertex(i);
                output.write(reinterpret_cast<const char*>(&i_query_point), sizeof(uint32_t));
            }
        }
        output.close();
        std::cout << "Saved grid as " << file << '\n';
    }

private:
    vector<lvr2::QueryPoint<BaseVecT>> m_queryPoints;
    unordered_map<size_t, BoxT*> m_cells;
    unordered_map<size_t, size_t> m_qpIndices;
    // DEPRECATED, check if bounding box is even needed
    lvr2::BoundingBox<BaseVecT> qp_bb;
    lvr2::BoundingBox<BaseVecT> m_boundingBox;
    float m_voxelsize;
    std::size_t m_maxIndex;
    std::size_t m_maxIndexSquare;
    std::size_t m_maxIndexX;
    std::size_t m_maxIndexY;
    std::size_t m_maxIndexZ;
    std::string m_boxType;
    unsigned int m_globalIndex;
    BaseVecT m_coordinateScales;
};