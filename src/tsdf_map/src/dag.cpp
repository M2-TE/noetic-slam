#include "dag/dag.hpp"
//
#include <algorithm>
#include <atomic>
#include <bitset>
#include <execution>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>
#include <array>
#include <cmath>
#include <span>
#include <thread>
#include <condition_variable>
#include <parallel/algorithm>
#include <cstdint>
//
#include <Eigen/Eigen>
//
#include "dag/constants.hpp"
#include "dag/norm_est.hpp"
#include "dag/morton_code.hpp"
#include "dag/node_levels.hpp"
#include "dag/octree.hpp"
#include "dag/node.hpp"

// todo: move static funcs to separate headers and move needed header files with them
static void sort_points(std::vector<Eigen::Vector3f>& points, std::vector<std::pair<MortonCode, Eigen::Vector3f>>& mortonCodes) {
    auto beg = std::chrono::steady_clock::now();
    
    // sort morton code vector
    auto sorter = [](const auto& a, const auto& b){
        return std::get<0>(a) > std::get<0>(b);
    };
    std::sort(std::execution::par_unseq, mortonCodes.begin(), mortonCodes.end(), sorter);
    
    // insert sorted morton code points back into points vector
    auto it_points = points.begin();
    for (auto it_morton = mortonCodes.cbegin(); it_morton != mortonCodes.cend(); it_morton++) {
        *it_points++ = std::get<1>(*it_morton);
    }
    
    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "pnts sort " << dur << '\n';
}
static auto calc_normals(Pose pose, std::vector<std::pair<MortonCode, Eigen::Vector3f>>& mortonCodes) -> std::vector<Eigen::Vector3f> {
    auto beg = std::chrono::steady_clock::now();
    
    // construct neighbourhoods of points
    typedef std::vector<std::pair<MortonCode, Eigen::Vector3f>>::const_iterator MortonIter;
    struct Neighbourhood {
        Neighbourhood(MortonIter it_beg, MortonIter it_end): it_beg(it_beg), it_end(it_end) {}
        MortonIter it_beg;
        MortonIter it_end;
    };
    auto calc_neigh_map = [&mortonCodes](size_t neighLevel) {
        // the level up to which is checked to see if two morton codes belong to the same neighbourhood
        size_t mask = std::numeric_limits<size_t>::max() << neighLevel * 3;
        phmap::flat_hash_map<typeof(MortonCode::val), Neighbourhood> neighMap;
        // phmap::btree_map<typeof(MortonCode::val), Neighbourhood> neighMap;
        // std::map<typeof(MortonCode::val), Neighbourhood> neighMap;
        
        // insert the first neighbourhood
        MortonCode mc_neigh = std::get<0>(mortonCodes.front());
        mc_neigh.val &= mask;
        auto it_neigh = neighMap.emplace(mc_neigh.val, Neighbourhood(mortonCodes.cbegin(), mortonCodes.cbegin())).first;
        
        // iterate through all points to build neighbourhoods
        for (auto it_morton = mortonCodes.cbegin() + 1; it_morton != mortonCodes.cend(); it_morton++) {
            // get morton codes from current point and current neighbourhood
            MortonCode mc_point = std::get<0>(*it_morton);
            MortonCode mc_neigh = std::get<0>(*it_neigh->second.it_beg);
            mc_point.val &= mask;
            mc_neigh.val &= mask;
            
            // check if the current point doesnt fit the neighbourhood
            if (mc_neigh != mc_point) {
                // set end() iterator for current neighbourhood
                it_neigh->second.it_end = it_morton;
                // create a new neighbourhood starting at current point
                it_neigh = neighMap.emplace(mc_point.val, Neighbourhood(it_morton, it_morton)).first;
            }
        }
        // set end() iterator for final neighbourhood
        it_neigh->second.it_end = mortonCodes.cend();
        
        return neighMap;
    };
    // use chunk at level 2 (relative from leaves) for neighbourhoods by default
    static size_t neighLevel = 2;
    auto neighMap = calc_neigh_map(neighLevel);
    // rough approximation of points per neighbourhood
    size_t nPts = mortonCodes.size() / neighMap.size();
    // decrease neigh level if too many points per neighbourhood are present
    while (nPts > 50) {
        neighLevel--;
        neighMap = calc_neigh_map(neighLevel);
        // rough approximation of points per neighbourhood
        nPts = mortonCodes.size() / neighMap.size();
        std::cout << "decreased neighbourhood level to: " << neighLevel << '\n';
    }
    // increase neigh level until the desired amount of points per neighbourhood is reached
    while (nPts < 6) {
        neighLevel++;
        neighMap = calc_neigh_map(neighLevel);
        // rough approximation of points per neighbourhood
        nPts = mortonCodes.size() / neighMap.size();
        std::cout << "increased neighbourhood level to: " << neighLevel << '\n';
    }
    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "neig calc " << dur << '\n';
    beg = std::chrono::steady_clock::now();
    
    // build normals using local neighbourhoods
    float maxDist = leafResolution * (1 << neighLevel);
    std::vector<Eigen::Vector3f> normals(mortonCodes.size());
    typedef decltype(neighMap)::const_iterator NeighIter;
    auto normFnc = [&normals, &neighMap, &mortonCodes, pose, maxDist](NeighIter beg, NeighIter end) {
        for (auto it_neigh = beg; it_neigh != end; it_neigh++) {
            // get morton code for current neighbourhood
            MortonCode mc_neigh = std::get<0>(*it_neigh);
            const Neighbourhood& neigh = std::get<1>(*it_neigh);
            Eigen::Vector3i pos_neigh = mc_neigh.decode();
            
            // gather adjacent neighbourhoods
            std::vector<Neighbourhood*> adjNeighs;
            for (int32_t z = -1; z <= +1; z++) {
                for (int32_t y = -1; y <= +1; y++) {
                    for (int32_t x = -1; x <= +1; x++) {
                        Eigen::Vector3i offset { x, y, z };
                        offset *= 1 << neighLevel;
                        MortonCode mc_near(pos_neigh + offset);
                        // attempt to find adjacent neighbourhood in map
                        auto it_near = neighMap.find(mc_near.val);
                        if (it_near != neighMap.cend()) {
                            Neighbourhood& adj = std::get<1>(*it_near);
                            adjNeighs.push_back(&adj);	
                        }
                    }
                }
            }
            
            // count total points in all adjacent neighbourhoods
            size_t n_points = 0;
            for (auto it_neigh = adjNeighs.cbegin(); it_neigh != adjNeighs.cend(); it_neigh++) {
                Neighbourhood& neigh = **it_neigh;
                n_points += neigh.it_end - neigh.it_beg;
            }
            // collect points from adjacent neighbours
            std::vector<Eigen::Vector3f> localPoints;
            localPoints.reserve(n_points);
            for (auto it_neigh = adjNeighs.cbegin(); it_neigh != adjNeighs.cend(); it_neigh++) {
                Neighbourhood& neigh = **it_neigh;
                for (auto it_point = neigh.it_beg; it_point != neigh.it_end; it_point++) {
                    localPoints.push_back(std::get<1>(*it_point));
                }
            }
            
            // reserve some generous space for nearest points
            std::vector<Eigen::Vector4d> nearestPoints;
            nearestPoints.reserve(n_points);
            // for every point within the current neighbourhood it_neigh, find its nearest neighbours for normal calc
            for (auto it_point = neigh.it_beg; it_point != neigh.it_end; it_point++) {
                Eigen::Vector3f point = std::get<1>(*it_point);
                
                // go over all points and store the ones within the boundary
                nearestPoints.clear();
                for (auto it_other = localPoints.cbegin(); it_other != localPoints.cend(); it_other++) {
                    Eigen::Vector3f diff = *it_other - point;
                    float distSqr = diff.squaredNorm();
                    if (distSqr <= maxDist*maxDist) {
                        Eigen::Vector3f other = *it_other;
                        nearestPoints.emplace_back(other.x(), other.y(), other.z(), 0.0);
                    }
                }
                
                // use these filtered nearest points to calc normal
                Eigen::Vector3f normal;
                if (nearestPoints.size() >= 3) normal = normal_from_neighbourhood(nearestPoints);
                else normal = (point - pose.pos).normalized();
                
                // flip normal if needed
                float dot = normal.dot(point - pose.pos);
                if (dot < 0.0f) normal *= -1.0f;
                
                // figure out index of point
                size_t index = it_point - mortonCodes.cbegin();
                normals[index] = normal;
                // normals[index] = point.normalized();
            }
        }
    };
    // balance load across several threads
    std::vector<std::thread> threads;
    threads.reserve(std::thread::hardware_concurrency());
    for (size_t iThread = 0; iThread < threads.capacity(); iThread++) {
        // neighbours per thread
        size_t nNeighs = neighMap.size() / threads.capacity();
        // prepare iterators
        NeighIter beg = neighMap.cbegin();
        std::advance(beg, nNeighs * iThread);
        NeighIter end;
        if (iThread == threads.capacity() - 1) {
            end = neighMap.cend();
        }
        else {
            end = beg;
            std::advance(end, nNeighs);
        }
        threads.emplace_back(normFnc, beg, end);
    }
    for (auto& thread: threads) thread.join();
    threads.clear();
    
    end = std::chrono::steady_clock::now();
    dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "norm calc " << dur << '\n';
    return normals;
}
static void build_trie_whatnot(Octree& octree, const Eigen::Vector3f* inputPos, Eigen::Vector3f inputNorm/*deprecated*/, uint32_t tid) {
    // convert to chunk position
    Eigen::Vector3f v = *inputPos * (1.0 / leafResolution);
    // properly floor instead of relying on float => int conversion
    v = v.unaryExpr([](float f){ return std::floor(f); });
    Eigen::Vector3i center_clusterChunk = v.cast<int32_t>();
    // %4 to get the chunk with 4x4x4 leaf clusters
    Eigen::Vector3i base_clusterChunk = center_clusterChunk.unaryExpr([](int32_t val) { return val - val%4; });
    
    // set constraints for which leaf clusters get created
    Eigen::Vector3i min = center_clusterChunk + Eigen::Vector3i(-1, -1, -1);
    Eigen::Vector3i max = center_clusterChunk + Eigen::Vector3i(+2, +2, +2);
    
    // fill large 3x3x3 neighbourhood around point with signed distances. Most will be discarded
    for (int32_t z4 = -4; z4 <= +4; z4 += 4) {
        for (int32_t y4 = -4; y4 <= +4; y4 += 4) {
            for (int32_t x4 = -4; x4 <= +4; x4 += 4) {
                // position of current main chunk
                Eigen::Vector3i main_clusterChunk = base_clusterChunk + Eigen::Vector3i(x4, y4, z4);
                MortonCode code(main_clusterChunk);
                code.val = code.val >> 3; // shift to cover the 3 LSB (leaves), which wont be encoded
                Octree::Node* oct_node = octree.insert(code.val, 19);
                
                // iterate over 2x2x2 sub chunks within the 4x4x4 main chunk
                uint8_t lc_index = 0; // leaf cluster index
                for (int32_t z2 = 0; z2 <= 2; z2 += 2) {
                    for (int32_t y2 = 0; y2 <= 2; y2 += 2) {
                        for (int32_t x2 = 0; x2 <= 2; x2 += 2, lc_index++) {
                            Eigen::Vector3i sub_clusterChunk = main_clusterChunk + Eigen::Vector3i(x2, y2, z2);
                            Octree::Node* cluster = oct_node->children[lc_index];
                            // if it doesnt exist yet, allocate some space for the pointers
                            if (cluster == nullptr) {
                                cluster = octree.allocate_misc_node();
                                oct_node->children[lc_index] = cluster;
                            }
                            // iterate over leaves within clusters
                            uint8_t iLeaf = 0;
                            for (int32_t z1 = 0; z1 <= 1; z1++) {
                                for (int32_t y1 = 0; y1 <= 1; y1++) {
                                    for (int32_t x1 = 0; x1 <= 1; x1++, iLeaf++) {
                                        Eigen::Vector3i leafChunk = sub_clusterChunk + Eigen::Vector3i(x1, y1, z1);
                                        // test if leaf falls within the constraints for current scan point
                                        bool valid = true;
                                        if (leafChunk.x() < min.x() || leafChunk.x() > max.x()) valid = false;
                                        if (leafChunk.y() < min.y() || leafChunk.y() > max.y()) valid = false;
                                        if (leafChunk.z() < min.z() || leafChunk.z() > max.z()) valid = false;
                                        if (!valid) continue;
                                        
                                        // if valid, calc real position of leaf
                                        Eigen::Vector3f leafPos = leafChunk.cast<float>() * leafResolution;
                                        
                                        // assign the closest point to leaf
                                        const Eigen::Vector3f*& closestPoint = cluster->leafPoints[iLeaf];
                                        if (closestPoint == nullptr) closestPoint = inputPos;
                                        else {
                                            float distSqr = (*inputPos - leafPos).squaredNorm();
                                            float distSqrOther = (*closestPoint - leafPos).squaredNorm();
                                            if (distSqr < distSqrOther) closestPoint = inputPos;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                // if (tid == 0) exit(0);
            }
        }
    }
    // if (tid == 0) exit(0);
}
static auto get_trie(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) -> Octree {
    auto beg = std::chrono::steady_clock::now();

    // round threads down to nearest power of two
    uint32_t lz = __builtin_clz(std::thread::hardware_concurrency());
    size_t nThreads = 1 << (32 - lz - 1);
    // std::cout << "Using " << nThreads << " threads for trie ctor\n";
    std::vector<std::thread> threads;
    threads.reserve(nThreads);
    std::vector<Octree> octrees(nThreads);

    // thread groups throughout the stages
    struct ThreadGroup {
        // no sync needed
        std::vector<Octree::Key> collisions;
        std::array<Octree*, 2> trees;
        uint32_t iLeadThread;
        uint32_t collisionDepth;
        // sync needed
        std::unique_ptr<std::condition_variable> cv;
        std::unique_ptr<std::mutex> mutex;
        uint32_t nCompleted = 0;
        bool bPrepared = false;
    };
    struct Stage {
        uint32_t groupSize; // threads per group
        std::vector<ThreadGroup> groups;
    };
    // calc number of total stages
    uint32_t nStages = std::sqrt(nThreads);
    std::vector<Stage> stages(nStages);
    // prepare thread groups for each stage
    for (uint32_t i = 0; i < nStages; i++) {
        auto& stage = stages[i];

        // figure out how many groups there are
        stage.groupSize = 2 << i;
        uint32_t nGroups = nThreads / stage.groupSize;

        // fill each group with info data
        stage.groups.resize(nGroups);
        for (uint32_t iGrp = 0; iGrp < nGroups; iGrp++) {
            auto& grp = stage.groups[iGrp];
            grp.iLeadThread = iGrp * stage.groupSize;
            grp.cv = std::make_unique<std::condition_variable>();
            grp.mutex = std::make_unique<std::mutex>();
        }
    }
    
    // build smaller octrees on several threads
    size_t progress = 0;
    for (size_t id = 0; id < nThreads; id++) {
        size_t nElements = points.size() / nThreads;
        if (id == nThreads - 1) nElements = 0; // special value for final thread to read the rest
        // build one octree per thread
        threads.emplace_back([&points, &normals, &octrees, id, progress, nElements](){
            auto pCur = points.cbegin() + progress;
            auto pEnd = (nElements == 0) ? (points.cend()) : (pCur + nElements);
            auto pNorm = normals.cbegin() + progress;
            Octree& octree = octrees[id];
            // Octree::PathCache cache(octree);
            for (; pCur < pEnd; pCur++, pNorm++) {
                const Eigen::Vector3f* inputPtr = &*pCur;
                build_trie_whatnot(octree, inputPtr, *pNorm, id);
            }
        });
        progress += nElements;
    }
    for (auto& thread: threads) thread.join();
    threads.clear();

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "trie ctor " << dur << '\n';
    
    // merge the smaller octrees into one
    beg = std::chrono::steady_clock::now();
    for (size_t id = 0; id < nThreads; id++) {
        threads.emplace_back([&octrees, &stages, id]() {
            // combine octrees via multiple stages using thread groups
            for (uint32_t iStage = 0; iStage < stages.size(); iStage++) {
                auto& stage = stages[iStage];
                uint32_t iGrp = id / stage.groupSize;
                uint32_t iLocal = id % stage.groupSize; // block-local index
                auto& grp = stage.groups[iGrp];

                if (grp.iLeadThread == id) { // leader thread
                    // wait for all dependent groups to finish
                    if (iStage > 0) {
                        auto& prevStage = stages[iStage - 1];
                        uint32_t iPrevGrp = id / prevStage.groupSize;
                        auto& grpA = prevStage.groups[iPrevGrp];
                        auto& grpB = prevStage.groups[iPrevGrp + 1];
                        uint8_t res = 0;
                        // wait for group A to be done
                        std::unique_lock lockA(*grpA.mutex);
                        grpA.cv->wait(lockA, [&]{ return grpA.nCompleted >= prevStage.groupSize; });
                        lockA.release();
                        // wait for group B to be done
                        std::unique_lock lockB(*grpB.mutex);
                        grpB.cv->wait(lockB, [&]{ return grpB.nCompleted >= prevStage.groupSize; });
                        lockB.release();
                    }
                    // set up pointers for group trees
                    grp.trees[0] = &octrees[id];
                    grp.trees[1] = &octrees[id + stage.groupSize/2];

                    // in order to keep a balanced load, have N collisions per thread
                    uint32_t nTargetCollisions = stage.groupSize;
                    std::tie(grp.collisions, grp.collisionDepth) = grp.trees[0]->merge(*grp.trees[1], nTargetCollisions);

                    // signal that this group thread is fully prepared
                    grp.mutex->lock();
                    grp.bPrepared = true;
                    grp.mutex->unlock();
                    grp.cv->notify_all();
                }
                // wait until the octree is ready to be worked on
                std::unique_lock lock(*grp.mutex);
                grp.cv->wait(lock, [&]{ return grp.bPrepared; });
                lock.unlock();
                
                // resolve assigned collisions
                uint32_t colIndex = iLocal;
                while (colIndex < grp.collisions.size()) {
                    grp.trees[0]->merge(*grp.trees[1], grp.collisions[colIndex], grp.collisionDepth);
                    colIndex += stage.groupSize;
                }
                // increment completion counter
                grp.mutex->lock();
                grp.nCompleted++;
                grp.mutex->unlock();
                grp.cv->notify_one();
            }
        });
    }
    for (auto& thread: threads) thread.join();
    threads.clear();

    end = std::chrono::steady_clock::now();
    dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "trie merg " << dur << '\n';
    return std::move(octrees[0]);
}

Dag::Dag() {
    // create node levels
    node_levels = new NodeLevel[63/3];
    leaf_level = new LeafLevel();
    // create a main root node, 1 child mask, 8 children
    for (auto i = 0; i < 9; i++) {
        node_levels[0].data.push_back(0);
    }
    node_levels[0].nOccupied += 9;
    uniques[0]++;
}
Dag::~Dag() {
    delete node_levels;
    delete leaf_level;
}
void Dag::insert_scan(Eigen::Vector3f position, Eigen::Quaternionf rotation, std::vector<Eigen::Vector3f>& points) {
    auto beg = std::chrono::steady_clock::now();
    Pose pose = { position, rotation };
    // TODO: transform points here, if not already transformed
    auto mortonCodes = calc_morton(points);
    sort_points(points, mortonCodes);
    auto normals = calc_normals(pose, mortonCodes);
    auto octree = get_trie(points, normals);
    auto dagAddr = insert_octree(octree, points, normals);
    merge_dag(dagAddr);

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "full  dur " << dur << '\n';
    
    // add points to large points vector
    grid_points.insert(grid_points.end(), points.cbegin(), points.cend());
}
void Dag::print_stats() {
    size_t nUniques = 0;
    size_t nDupes = 0;
    size_t nBytes = 0;
    for (size_t i = 0; i < uniques.size(); i++) {
        std::cout << std::fixed;
        std::cout << "Level " << i << ": " << uniques[i] << " uniques, " << dupes[i] << " dupes, ";
        std::cout << static_cast<double>(uniques[i] * sizeof(uint32_t)) / 1024.0 / 1024.0 << " MiB \n";
        nUniques += uniques[i];
        nDupes += dupes[i];
    }
    std::cout << static_cast<double>(nUniques * sizeof(uint32_t)) / 1024.0 / 1024.0 << " MiB used in total\n";
}
auto Dag::insert_octree(Octree& octree, std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) -> uint32_t {
    auto beg = std::chrono::steady_clock::now();
    uint32_t rootAddr = node_levels[0].data.size();
    
    // trackers that will be updated during traversal
    std::array<uint8_t, 63/3> path;
    std::array<std::array<uint32_t, 8>, 63/3> dagNodes;
    std::array<const Octree::Node*, 63/3> octNodes;
    // reset arrays
    path.fill(0);
    octNodes.fill(nullptr);
    for (auto& level: dagNodes) level.fill(0);
    // set starting values
    int64_t depth = 0;
    octNodes[0] = octree.get_root();
    
    // iterate through octree and insert into hashDAG
    while (depth >= 0) {
        auto iChild = path[depth]++;
        
        // when all children were iterated
        if (iChild == 8) {
            // gather all children for this new node
            std::vector<uint32_t> children(1);
            for (auto i = 0; i < 8; i++) {
                if (dagNodes[depth][i] == 0) continue;
                children.push_back(dagNodes[depth][i]);
                children[0] |= 1 << i; // child mask
            }
            // add child count to mask (TODO: deprecate)
            children[0] |= (children.size() - 1) << 8;
            // reset node tracker for used-up nodes
            dagNodes[depth].fill(0);
            // skip node creation when there are no children
            if (children.size() > 1) {
                // check path in parent depth to know this node's child ID
                uint32_t indexInParent = path[depth - 1] - 1;
                
                // resize data if necessary and then copy over
                auto& level = node_levels[depth];
                level.data.resize(level.nOccupied + children.size());
                std::memcpy(
                    level.data.data() + level.nOccupied,
                    children.data(),
                    children.size() * sizeof(uint32_t));
                // check if the same node existed previously
                uint32_t temporary = level.nOccupied;
                auto [pIndex, bNew] = level.hashSet.emplace(temporary);
                if (bNew) {
                    level.nOccupied += children.size();
                    uniques[depth]++;
                    if (depth > 0) {
                        dagNodes[depth - 1][indexInParent] = temporary;
                    }
                }
                else {
                    dupes[depth]++;
                    if (depth > 0) {
                        dagNodes[depth - 1][indexInParent] = *pIndex;
                    }
                }
            }
            
            depth--;
        }
        // node contains children
        else if (depth < 63/3 - 1) {
            // retrieve child node
            auto* pChild = octNodes[depth]->children[iChild];
            if (pChild == nullptr) continue;
            // walk deeper
            depth++;
            octNodes[depth] = pChild;
            path[depth] = 0;
        }
        // node contains leaf cluster
        else {
            // get node containing the points closest to each leaf
            Octree::Node* clusterPoints = octNodes[depth]->children[iChild];
            if (clusterPoints == nullptr) continue;
            
            // reconstruct morton code from path
            uint64_t mortonCode = 0;
            for (uint64_t k = 0; k < 63/3; k++) {
                uint64_t part = path[k] - 1;
                mortonCode |= part << (60 - k*3);
            }
            // revert shift on insertion
            mortonCode = mortonCode << 3;
            // convert to actual cluster chunk position
            Eigen::Vector3i cluster_chunk;
            std::tie(cluster_chunk.x(), cluster_chunk.y(), cluster_chunk.z()) = mortonnd::MortonNDBmi_3D_64::Decode(mortonCode);
            // convert from 21-bit inverted to 32-bit integer
            cluster_chunk = cluster_chunk.unaryExpr([](auto i){ return i - (1 << 20); });
            
            // iterate over leaves within clusters to calc signed distances
            std::array<std::pair<float, bool>, 8> leaves;
            uint8_t iLeaf = 0;
            for (int32_t z = 0; z <= 1; z++) {
                for (int32_t y = 0; y <= 1; y++) {
                    for (int32_t x = 0; x <= 1; x++, iLeaf++) {
                        // actual leaf position
                        Eigen::Vector3i leafChunk = cluster_chunk + Eigen::Vector3i(x, y, z);
                        Eigen::Vector3f leafPos = leafChunk.cast<float>() * leafResolution;
                        const Eigen::Vector3f* nearestPoint = clusterPoints->leafPoints[iLeaf];
                        
                        // store leaf validity
                        leaves[iLeaf].second = nearestPoint != nullptr;
                        if (!leaves[iLeaf].second) continue;
                        
                        // check if point is too far away from leaf
                        Eigen::Vector3f diff = leafPos - *nearestPoint;
                        // calculate signed distance for current leaf
                        size_t index = nearestPoint - points.data();
                        float signedDistance = normals[index].dot(diff);
                        leaves[iLeaf].first = signedDistance;
                    }
                }
            }
            LeafCluster lc(leaves);
            // skip if all leaves were invalid
            if (lc.cluster == std::numeric_limits<LeafCluster::ClusterT>().max()) continue;
            
            // check if this leaf cluster already exists
            auto temporaryIndex = leaf_level->data.size();
            auto [pIter, bNew] = leaf_level->hashMap.emplace(lc.cluster, temporaryIndex);
            if (bNew) {
                uniques[depth+1]++;
                // insert as new leaf cluster
                auto [part0, part1] = lc.get_parts();
                leaf_level->data.push_back(part0);
                leaf_level->data.push_back(part1);
                dagNodes[depth][iChild] = temporaryIndex;
            }
            else {
                dupes[depth+1]++;
                // simply update references to the existing cluster
                dagNodes[depth][iChild] = pIter->second;
            }
        }
    }
    
    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "dag  ctor " << dur << '\n';
    return rootAddr;
}
void Dag::merge_dag(uint32_t srcAddr) {
    auto beg = std::chrono::steady_clock::now();
    
    // trackers that will be updated during traversal
    std::array<uint8_t, 63/3> path;
    std::array<const Node*, 63/3> dstNodes;
    std::array<const Node*, 63/3> srcNodes;
    std::array<std::array<uint32_t, 8>, 63/3> newNodes;
    // reset arrays
    path.fill(0);
    dstNodes.fill(nullptr);
    srcNodes.fill(nullptr);
    for (auto& level: newNodes) level.fill(0);
    // set starting values
    int64_t depth = 0;
    dstNodes[0] = Node::conv(node_levels[0].data, 1);
    srcNodes[0] = Node::conv(node_levels[0].data, srcAddr);
    
    while (depth >= 0) {
        auto iChild = path[depth]++;
        // all children iterated, go back up and create node
        if (iChild == 8) {
            // normal node
            if (depth > 0) {
                // gather all children for this new node
                std::vector<uint32_t> children(1); // init with child mask
                for (auto i = 0; i < 8; i++) {
                    if (newNodes[depth][i] == 0) continue;
                    children.push_back(newNodes[depth][i]);
                    children[0] |= 1 << i; // child mask
                }
                // add child count to mask (TODO: deprecate)
                children[0] |= (children.size() - 1) << 8;
                // reset node tracker for used-up nodes
                newNodes[depth].fill(0);
                
                // check path in parent depth to know this node's child ID
                uint32_t indexInParent = path[depth - 1] - 1;
                
                // resize data if necessary and then copy over
                auto& level = node_levels[depth];
                level.data.resize(level.nOccupied + children.size());
                std::memcpy(
                    level.data.data() + level.nOccupied,
                    children.data(),
                    children.size() * sizeof(uint32_t));
                // check if the same node existed previously
                uint32_t temporary = level.nOccupied;
                auto [pIndex, bNew] = level.hashSet.emplace(temporary);
                if (bNew) {
                    level.nOccupied += children.size();
                    uniques[depth]++;
                    newNodes[depth - 1][indexInParent] = temporary;
                }
                else {
                    dupes[depth]++;
                    newNodes[depth - 1][indexInParent] = *pIndex;
                }
                depth--;
            }
            // root node
            else {
                // the root node "dst" will always exist and will already have space allocated for all 8 children
                // therefore, only need to insert into it
                auto* rootNode = dstNodes[0];
                
                // add existing or new children
                std::vector<uint32_t> children(1); // init with child mask
                for (auto i = 0; i < 8; i++) {
                    uint32_t addr = 0;
                    // first get existing node
                    if (rootNode->contains_child(i)) {
                        addr = rootNode->get_child_addr(i);
                    }
                    // overwrite with new if present
                    if (newNodes[depth][i] > 0) {
                        addr = newNodes[depth][i];
                    }
                    // skip this child if it doesnt exist
                    if (addr == 0) continue;
                    // add to child vector and child mask
                    children.push_back(addr);
                    children[0] |= 1 << i; // child mask
                }
                // add child count to mask (TODO: deprecate)
                children[0] |= (children.size() - 1) << 8;
                
                // overwrite old root node
                auto& level = node_levels[0];
                std::memcpy(
                    level.data.data() + 1,
                    children.data(),
                    children.size() * sizeof(uint32_t));		
                depth--;
            }
        }
        // normal child node
        else if (depth < 63/3 - 1) {
            auto* dstNode = dstNodes[depth];
            auto* srcNode = srcNodes[depth];
            
            // insert if present in src tree
            if (srcNode->contains_child(iChild)) {
                uint32_t srcChildAddr = srcNode->get_child_addr(iChild);
                
                // when present in dst tree, walk further down that path
                if (dstNode->contains_child(iChild)) {
                    uint32_t dstChildAddr = dstNode->get_child_addr(iChild);
                    depth++;
                    path[depth] = 0;
                    dstNodes[depth] = Node::conv(node_levels[depth].data, dstChildAddr);
                    srcNodes[depth] = Node::conv(node_levels[depth].data, srcChildAddr);
                }
                // else simply add the existing node to "new nodes"
                else {
                    newNodes[depth][iChild] = srcChildAddr;
                }
            }
            // preserve the already existing node from dst
            else if (dstNode->contains_child(iChild)) {
                uint32_t dstChildAddr = dstNode->get_child_addr(iChild);
                newNodes[depth][iChild] = dstChildAddr;
            }
        }
        // leaf cluster node
        else {
            // todo: note that this shouldnt return a leaf cluster, but just an addr
            uint32_t dstLcAddr = dstNodes[depth]->get_child_addr(iChild);
            uint32_t srcLcAddr = srcNodes[depth]->get_child_addr(iChild);
            // leaf clusters are 64 bit, so they take 2 addresses
            LeafCluster dstLc(leaf_level->data[dstLcAddr], leaf_level->data[dstLcAddr + 1]);
            LeafCluster srcLc(leaf_level->data[srcLcAddr], leaf_level->data[srcLcAddr + 1]);
            
            // merge both leaf clusters
            LeafCluster resLc(dstLc);
            dstLc.merge(srcLc);
            
            // check if result is equal to either one of the clusters
            if (resLc == dstLc) {
                newNodes[depth][iChild] = dstLcAddr;
            }
            else if (resLc == srcLc) {
                newNodes[depth][iChild] = srcLcAddr;
            }
            // if not, create a new leaf based on resLc
            else {
                // check if this leaf cluster already exists
                auto temporaryIndex = leaf_level->data.size();
                auto [pIter, bNew] = leaf_level->hashMap.emplace(resLc.cluster, temporaryIndex);
                if (bNew) {
                    uniques[depth+1]++;
                    // insert as new leaf cluster
                    auto [part0, part1] = resLc.get_parts();
                    leaf_level->data.push_back(part0);
                    leaf_level->data.push_back(part1);
                    newNodes[depth][iChild] = temporaryIndex;
                }
                else {
                    dupes[depth+1]++;
                    // simply update references to the existing cluster
                    newNodes[depth][iChild] = pIter->second;
                }
            }
        }
    }
    
    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    std::cout << "dag  merg " << dur << '\n';
}