#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
//
#include <boost/math/ccmath/ccmath.hpp>
#include <Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
//
#include "constants.hpp"
#include "dag_structs.hpp"

namespace DAG {
static constexpr float distPerVoxel = 0.01f;

// max squared (real) distance for another scan point to be able to contribute to nearby voxels
template<size_t dimSize> static constexpr double get_influence_radius_sqr() {
    constexpr double a = static_cast<double>(dimSize), b = static_cast<double>(dimSize);
    constexpr double c = boost::math::ccmath::sqrt(a*a + b*b);
    constexpr double res = 2*c * 2*c;
    return static_cast<float>(res * distPerVoxel);
}
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

    void insert_scan(Eigen::Vector3f position, Eigen::Vector3f rotation, const std::vector<Eigen::Vector3f>& scan) {
        std::vector<Eigen::Vector3f> normals;
        normals.reserve(scan.size());


        // normal calculation
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
            auto normal = vecA.cross(vecB);

            // invert normal if it points in the wrong direction
            if (normal.dot(*pCur - position) >= 0.0f) normal = -normal;
            normals.emplace_back(normal);
        }

        // describe neighbour grid to write signed distance to
        static constexpr uint32_t dim = 3; // can be 1, 3, 5, 7, [...]. describes voxel volume to calc SD contribution for
        std::vector<uint32_t> influencers;

        // signed distance neighbourhood evaluation
        for (auto pCur = scan.cbegin(); pCur != scan.cend(); pCur++) {
            
            // iterate through all point to get close neighbours for current point
            influencers.clear();
            for (auto pComp = scan.cbegin(); pComp != scan.cend(); pComp++) {
                // todo: calc signed distance via hyperplane

                // todo: if distance is < something, add index to influencers
            }
            
            // todo: fill signed distances with influencer candidates
        }
    }

private:
    std::vector<Scan> scans;

    phmap::flat_hash_map<DAG::RootPos, DAG::NodePointer> dagRootMap;
    phmap::flat_hash_map<DAG::Leaves, DAG::NodePointer> dagLeafMap;
    std::array<DAG::Level, nDagLevels> dagLevels;
};
}