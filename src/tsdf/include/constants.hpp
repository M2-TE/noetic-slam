#pragma once
// compile time constants
static constexpr float voxelToCoordRatio = 0.01f; // every voxel unit represents this distance
static constexpr float coordToVoxelRatio = 1.0f / voxelToCoordRatio; // simple reciprocal for conversion
static constexpr int32_t nDagLevels = 5; // number of DAG levels including roots and leaf nodes
static constexpr int32_t rootDimSize = 1 << nDagLevels; // voxel size of each root node dimension