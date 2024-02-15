#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
//
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/math/ccmath/ccmath.hpp>
#include <Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
//
#include "constants.hpp"
#include "dag_structs.hpp"

namespace DAG {


static constexpr float distPerVoxel = 0.01f; // real distance for each voxel step
static constexpr uint32_t nDagLevels = 3; // number of DAG levels including root and leaf nodes
static constexpr std::array<size_t, nDagLevels> voxelSizes = {
    2, // root
    2, // 1
    2, // leaves
};

struct Scan {
    Eigen::Vector3f position;
    Eigen::Vector3f rotation;
    std::vector<NodePointer> roots;
};
struct Map {
    Map() {
        // set up hash and equality functors with proper references
        for (size_t i = 0; i < dagLevels.size(); i++) {
            DAG::Level& level = dagLevels[i];

            DAG::HashFunctor hashFnc = { &level.data };
            DAG::CompFunctor compFnc = { &level.data };
            level.pointerSet = decltype(level.pointerSet)(0, hashFnc, compFnc);
        }
    }

    void insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, const std::vector<Eigen::Vector3f>& scan) {

    }
    void insert_scan_old(Eigen::Vector3f position, Eigen::Vector3f rotation, const std::vector<Eigen::Vector3f>& scan) {
        // normal estimation
        std::vector<Eigen::Vector3f> normals;
        normals.reserve(scan.size());
        for (auto pCur = scan.cbegin(); pCur != scan.cend(); pCur++) {
            // two closest neighbours
            std::pair<uint32_t, float> directNeighbourA = { {}, std::numeric_limits<float>::max() };
            std::pair<uint32_t, float> directNeighbourB = directNeighbourA;
            
            // iterate through all point to get close neighbours for current point
            for (auto pComp = scan.cbegin(); pComp != scan.cend(); pComp++) {
                // squared distance is sufficient for comparison
                float distSqr = (*pCur - *pComp).squaredNorm();
                uint32_t index = pComp - scan.cbegin();

                // update 2 direct neighbours
                if (distSqr < directNeighbourA.second) directNeighbourA = { index, distSqr };
                else if (distSqr < directNeighbourB.second) directNeighbourB = { index, distSqr };
            }
            // std::cout << influencers.size() << std::endl;
            
            // calc normal using two neighbours
            auto vecA = scan[directNeighbourA.first] - *pCur;
            auto vecB = scan[directNeighbourB.first] - *pCur;
            auto normal = vecA.cross(vecB).normalized();

            // invert normal if it points in the wrong direction
            if (normal.dot(*pCur - position) >= 0.0f) normal = -normal;
            normals.push_back(normal);
        }

        // todo:
        // create temporary tree by inserting these scanned points
        //// for every point:
        //// 1. create hyperplane
        //// 2. get leaf cluster pos (and 8 surrounding leaf cluster positions)
        //// 3. get each leaf cluster from hash table
        //// 4. calculate real pos of each leaf in leaf cluster
        //// 5. insert signed distance into leaves
        static constexpr size_t dim = voxelSizes.back();
        struct LeafPosition { uint32_t x, y, z; };
        struct LeafCluster { std::array<float, dim*dim*dim> leaves; };
        phmap::flat_hash_map<LeafPosition, LeafCluster> tempMap;
        
    }

private:
    std::vector<Scan> scans;

    // not really in use atm
    phmap::flat_hash_map<DAG::RootPos, DAG::NodePointer> dagRootMap;
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};
}