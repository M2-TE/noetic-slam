#pragma once
#include <cstdint>
#include <Eigen/Eigen>

typedef uint32_t NodeIndex; // position of node in a level's data vector
typedef uint32_t ChildMask; // contains 8 children
typedef uint32_t NodeData; // contains a mask or child as raw undefined data

struct MortonIndex { Eigen::Vector3f point; uint32_t index; };
struct Pose { Eigen::Vector3f pos; Eigen::Quaternionf rot; };
struct Scan { Pose pose; std::vector<NodeIndex> roots; };

static constexpr double leafResolution = 0.1; // voxel size
static constexpr uint32_t nDagLevels = 63/3; // number of DAG levels including root and leaf clusters (excluding leaves)
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
}
static constexpr std::array<uint32_t, nAllLevels> dagSizes = get_sizes();
static constexpr std::array<double, nAllLevels> dagResolutions = get_resolutions();
namespace idx { // static indices
    static constexpr size_t root = 0;
    static constexpr size_t leaf = nDagLevels;
    static constexpr size_t leafCluster = leaf - 1;
}