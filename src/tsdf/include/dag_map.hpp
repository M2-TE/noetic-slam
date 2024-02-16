#pragma once
#include <iostream>
#include <vector>
#include <array>
#include <limits>
#include <cmath>
#include <bitset>
#include <random>
//
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/math/ccmath/ccmath.hpp>
#include <Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
#include <libmorton/morton.h> // morton lib A
#define __BMI2__
#include <morton-nd/mortonND_BMI2.h> // morton lib B
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
        
        // get sign bit
        constexpr int16_t offset = 1 << 15;
        static_assert(sizeof(int16_t) == 2);
        static_assert(sizeof(uint16_t) == 2);
        static_assert(static_cast<int16_t>(0) == 0b0);
        static_assert(std::numeric_limits<int16_t>::min() - offset == 0b0);

        // std::cout << std::numeric_limits<int16_t>::min() << " " << std::bitset<16>(std::numeric_limits<int16_t>::min()) << std::endl;
        // std::cout << std::numeric_limits<int16_t>::min() << " " << std::bitset<16>(std::numeric_limits<int16_t>::min() - offset) << std::endl;

        // map min(int),...,max(int) to 0,...,max(uint) (-1, 0, 1 need to be right next to each other)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int16_t> dis(std::numeric_limits<int16_t>::min(), std::numeric_limits<int16_t>::max());
        // std::uniform_int_distribution<int16_t> dis(-5, 5);
        for (int i = 0; i < 100000; i++) {
            int16_t x = dis(gen) - offset;
            int16_t y = dis(gen) - offset;
            int16_t z = dis(gen) - offset;

            // todo: map range to 21 bits? or 20

            // mortonnd::MortonNDBmi_3D_64::Encode()
            
            uint_fast64_t code = libmorton::morton3D_64_encode((uint_fast32_t)x, (uint_fast32_t)y, (uint_fast32_t)z);
            uint_fast32_t xu, yu, zu;
            libmorton::morton3D_64_decode(code, xu, yu, zu);

            if (x == (int16_t)xu && y == (int16_t)yu && z == (int16_t)zu) {
                // std::cout << "success: " << x << " " << y << " " << z  << std::endl;
            }
            else {
                std::cout << "         " << x << " " << y << " " << z 
                    << "\n\tcompared:" << xu << " " << yu << " " << zu << std::endl;
            }
        }
        return;
        
        std::vector<uint_fast64_t> mortonCodes;
        mortonCodes.reserve(scan.size());
        for (auto pCur = scan.cbegin(); pCur != scan.cend(); pCur++) {
            // calculate voxel position in DAG tree
            // Eigen::Vector3<int32_t> voxelPos32;
            // remap min<int32_t>, max<int32_t>
            // to: 0, max<uint32_t>
            // Eigen::Vector3<uint32_t> voxelPos16;
            // 

            
            uint_fast32_t x, y, z; // todo
            mortonCodes.push_back(libmorton::morton3D_64_encode(x, y, z));
        }



        // 1. calc morton codes
        // 2. sort (radix?) scan via morton code
        // 3? create binary radix tree from sorted data
        // 4. create DAG sub-tree from data
        // 5. calculate each point's normal via approximated nearest neighbour hyperplane
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