#pragma once
#include <eigen3/Eigen/Eigen>

struct Dag {
    Dag();
    ~Dag();
    void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, std::vector<Eigen::Vector3f>& points);
    void reconstruct();
    void print_stats();
private:
    auto insert_octree(struct Octree& octree, std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) -> uint32_t;
    void merge_dag(uint32_t srcAddr);

private:
    struct NodeLevel* node_levels;
    struct LeafLevel* leaf_level;
    // work in progress:
    struct Scan {
        uint32_t root;
        Eigen::Vector3f pos;
        Eigen::Quaternionf rot;
    };
    std::vector<Scan> scans;
    // meta info about tree levels
    std::array<uint32_t, 63/3+1> uniques = {};
    std::array<uint32_t, 63/3+1> dupes = {};
};