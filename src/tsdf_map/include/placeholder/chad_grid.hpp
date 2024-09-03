#pragma once
#include <unordered_map>
#include <lvr2/geometry/PMPMesh.hpp>
#include <lvr2/geometry/BoundingBox.hpp>
#include <lvr2/reconstruction/HashGrid.hpp>
#include <lvr2/reconstruction/FastBox.hpp>
#include <lvr2/reconstruction/QueryPoint.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>

template<typename BaseVecT, typename BoxT>
struct ChadGrid: public lvr2::GridBase {
    ChadGrid(): lvr2::GridBase(false) {
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
    void saveGrid(string file) override {
        std::cout << "saveGrid called" << std::endl;
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