#pragma once
#include <eigen3/Eigen/Eigen>
//
#include "dag/structs.hpp" // TODO: remove

struct Dag {
    Dag();
    void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, std::vector<Eigen::Vector3f>& points);
    void reconstruct();
    void print_stats();
private:
// TODO: try trailing return type
    auto insert_octree(struct Octree& octree, std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) -> uint32_t;
    void merge_dag(uint32_t srcAddr);

// todo: remove some of these as they are unnecessary
private:
    std::array<uint32_t, 63/3+1> uniques = {};
    std::array<uint32_t, 63/3+1> dupes = {};

    // new //
    std::array<NodeLevel, 63/3> nodeLevels; // TODO: intern NodeLevel decl
    LeafLevel leafLevel;
    struct Scan {
        uint32_t root;
        Eigen::Vector3f pos;
        Eigen::Quaternionf rot;
        // Eigen::Vector3f lowerLeft = {max, max, max};
        // Eigen::Vector3f upperRight = {min, min, min};
    };
    std::vector<Scan> scans;
};