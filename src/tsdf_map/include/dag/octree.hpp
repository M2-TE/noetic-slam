#pragma once
#include <algorithm>
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <vector>
//
#include <Eigen/Eigen>
#include <parallel_hashmap/phmap.h>
//
#include "dag/constants.hpp"

struct Octree {
    typedef uint64_t Key; // only 63 bits in use
    typedef uint64_t Leaf;
    union Node {
        std::array<const Eigen::Vector3f*, 8> leafPoints;
        std::array<Node*, 8> children;
        std::array<Leaf, 8> leaves; // dont really need this anymore tbh
    };
    struct MemoryBlock {
        MemoryBlock(size_t capacity): _capacity(capacity), _size(0) {
            void* mem = std::aligned_alloc(sizeof(Node), _capacity * sizeof(Node));
            _pData = static_cast<Node*>(mem);
        }
        MemoryBlock(MemoryBlock&) = delete;
        MemoryBlock(MemoryBlock&& other) {
            _size = other._size;
            _pData = other._pData;
            other._pData = nullptr;
            other._size = 0;
        }
        MemoryBlock& operator=(MemoryBlock&)  = delete;
        MemoryBlock& operator=(MemoryBlock&&) = delete;
        ~MemoryBlock() {
            // if (_pData != nullptr) delete _pData;
        }
        Node* get_new() {
            Node* pNode = _pData + _size++;
            std::memset(pNode, 0, sizeof(Node));
            return pNode;
        }
        size_t check_remaining() {
            return _capacity - _size;
        }
        
    private:
        Node* _pData;
        size_t _capacity;
        size_t _size;
    };
    Octree() {
        // create a reasonably sized memory block
        size_t n_capacity = 1<<16;
        _memory_blocks.emplace_back(n_capacity);

        // create root node
        _pRoot = _memory_blocks.front().get_new();
        std::memset(_pRoot, 0, sizeof(Node));
    }
    Octree(Octree&)  = delete;
    Octree(Octree&& other) {
        _memory_blocks = std::move(other._memory_blocks);
        _pRoot = other._pRoot;
    }
    Octree& operator=(Octree&)  = delete;
    Octree& operator=(Octree&&) = delete;
    ~Octree() = default;
    
    const Node* get_root() { return _pRoot; }
    // finds leaf via key and emplaces nodes if needed
    Leaf& find(Key key) {
        // depth from 20 (root) to 0 (leaf)
        size_t rDepth = 63/3 - 1;
        
        // check if memory block has enough space left
        if (_memory_blocks.back().check_remaining() < 63/3) {
            _memory_blocks.emplace_back(1<<16);
        }
        auto& memblock = _memory_blocks.back();
        
        // go from root to leaf parent
        Node* pNode = _pRoot;
        while (rDepth > 0) {
            size_t index = (key >> rDepth*3) & 0b111;
            Node* pChild = pNode->children[index];
            if (pChild == nullptr) {
                pChild = memblock.get_new();
                pNode->children[index] = pChild;
            }
            pNode = pChild;
            rDepth--;
        }
        // pNode will be leaf parent
        return pNode->leaves[key & 0b111];
    }
    // return node via key from given depth (0 root to 20 leaf)
    Node* find(Key key, size_t target_depth) const {
        // depth from 20 (root) to 0 (leaf)
        size_t rDepth = 63/3 - 1;
        // reverse target depth to fit internal depth iterator
        target_depth = rDepth - target_depth;
        
        // go from root to leaf parent
        Node* pNode = _pRoot;
        while (rDepth > target_depth) {
            size_t index = (key >> rDepth*3) & 0b111;
            pNode = pNode->children[index];
            rDepth--;
        }
        // return pointer to the requested child node
        size_t index = (key >> rDepth*3) & 0b111;
        return pNode->children[index];
    }
    // insert a node up to a given level
    Node* insert(Key key, size_t target_depth) {
        // depth from 20 (root) to 0 (leaf)
        size_t rDepth = 63/3 - 1;
        // reverse target depth to fit internal depth iterator
        target_depth = rDepth - target_depth;
        
        // check if memory block has enough space left
        if (_memory_blocks.back().check_remaining() < 63/3) {
            _memory_blocks.emplace_back(1<<16);
        }
        auto& memblock = _memory_blocks.back();
        
        // go from root to leaf parent
        Node* pNode = _pRoot;
        while (rDepth >= target_depth) {
            size_t index = (key >> rDepth*3) & 0b111;
            Node* pChild = pNode->children[index];
            if (pChild == nullptr) {
                pChild = memblock.get_new();
                pNode->children[index] = pChild;
            }
            pNode = pChild;
            rDepth--;
        }
        // return pointer to the requested child node
        return pNode;
    }
    // emplace chosen leaf and all required nodes inbetween
    void emplace(Key key, Leaf leaf) {
        find(key) = leaf;
    }
    // merge up until the depth where minCollisions is met
    std::pair<std::vector<Key>, uint32_t> merge(Octree& other, uint32_t minCollisions) {
        std::vector<Key> collisions;
        phmap::flat_hash_map<Key, Node*> parents_a;
        phmap::flat_hash_map<Key, Node*> layer_a, layer_b;
        uint32_t prevCollisions = 0;
        uint32_t depth = 0;
        
        // set up the initial layer of roots
        layer_a[0] = _pRoot;
        layer_b[0] = other._pRoot;
        
        // move memory block to merge target (pointers remain stable)
        for (auto& memblock: other._memory_blocks) {
            _memory_blocks.emplace_back(std::move(memblock));
        }
        
        // go through the layers to merge nodes and build collisions
        for (; depth < 63/3; depth++) {
            collisions.clear();
            
            // go through current layer and replace it with the next one
            decltype(layer_a) new_a;
            for (auto& node: layer_a) {
                for (uint64_t i = 0; i < 8; i++) {
                    Node* pChild = node.second->children[i];
                    if (pChild != nullptr) {
                        // construct key for this child node
                        uint64_t key = i << (60 - depth*3);
                        key |= node.first;
                        // insert into layer
                        new_a[key] = pChild;
                    }
                }
            }
            decltype(layer_b) new_b;
            for (auto& node: layer_b) {
                for (uint64_t i = 0; i < 8; i++) {
                    Node* pChild = node.second->children[i];
                    if (pChild != nullptr) {
                        // construct key for this child node
                        uint64_t key = i << (60 - depth*3);
                        key |= node.first;
                        // insert into layer
                        new_b[key] = pChild;
                    }
                }
            }
            // overwrite old layers
            parents_a = std::move(layer_a);
            layer_a = std::move(new_a);
            layer_b = std::move(new_b);
            
            // check for collisions between the layers
            std::vector<Key> queuedRemovals;
            for (auto& node: layer_b) {
                Key key = node.first;
                // if a collision occurs, save it
                if (layer_a.contains(key)) {
                    collisions.push_back(key);
                }
                // when node is missing from layer_a, insert it as new child
                else {
                    // index of new child node (0 to 7)
                    Key childKey = key >> (60 - depth*3);
                    childKey &= 0b111;
                    // remove child part from key to get parent key
                    Key parentKey = key ^ (childKey << (60 - depth*3));
                    
                    // use parent key to find the parent node in previous layer
                    Node* pParent = parents_a[parentKey];
                    // insert new child into it
                    pParent->children[childKey] = node.second;
                    // queue node for removal from layer_b
                    queuedRemovals.push_back(key);
                }
            }
            // remove the nodes from b that were already inserted into a
            for (auto& key: queuedRemovals) layer_b.erase(key);
            if (layer_b.empty()) {
                // no need for any more merging at all
                collisions.clear();
                depth = 63/3;
                break;
            }
            // only check for break condition if collision count has changed
            if (depth > 0 && collisions.size() != prevCollisions) {
                // break if collision target is met or collisions are decreasing
                if (collisions.size() >= minCollisions) break;
                if (collisions.size() < prevCollisions) break;
            }
            prevCollisions = collisions.size();
        }
        return { collisions, depth };
    }
    // merge via previously found collisions
    void merge(Octree& other, Key key, uint32_t startDepth) {
        uint32_t pathLength = 63/3 - startDepth;
        std::vector<uint8_t> path(pathLength);
        std::vector<Node*> nodes_a(pathLength);
        std::vector<Node*> nodes_b(pathLength);
        
        // when start depth is equal to 63/3, it is a special value indicating no need for merging
        if (pathLength == 0) return;
        
        // find the colliding nodes in both trees
        path[0] = 0;
        nodes_a[0] = find(key, startDepth);
        nodes_b[0] = other.find(key, startDepth);
        uint32_t depth = 0;

        // begin traversal
        while (path[0] <= 8) {
            auto iChild = path[depth]++;
            if (iChild >= 8) {
                // go back up to parent
                depth--;
                continue;
            }
            // retrieve child nodes
            Node* pChild_a = nodes_a[depth]->children[iChild];
            Node* pChild_b = nodes_b[depth]->children[iChild];
            
            if (pChild_a != nullptr) {
                // if this node is missing from B, simply skip it
                if (pChild_b == nullptr) continue;
                // if child nodes are leaves, call resolver
                if (depth >= pathLength - 2) {
                    // reconstruct morton code from path
                    uint64_t mortonCode = key;
                    for (uint64_t k = 0; k < pathLength - 1; k++) {
                        uint64_t part = path[k] - 1;
                        uint64_t shiftDist = pathLength*3 - 6 - k*3;
                        mortonCode |= part << shiftDist;
                    }
                    // revert shift on insertion
                    mortonCode = mortonCode << 3;
                    
                    // convert to actual cluster chunk position
                    Eigen::Vector3i cluster_chunk;
                    std::tie(cluster_chunk.x(), cluster_chunk.y(), cluster_chunk.z()) = mortonnd::MortonNDBmi_3D_64::Decode(mortonCode);
                    // convert from 21-bit inverted to 32-bit integer
                    cluster_chunk = cluster_chunk.unaryExpr([](auto i){ return i - (1 << 20); });
                    
                    // get nodes containing the scanpoint pointers
                    Node* leafPoints_a = nodes_a[depth]->children[iChild];
                    Node* leafPoints_b = nodes_b[depth]->children[iChild];
                    
                    // iterate over leaves within clusters
                    uint8_t iLeaf = 0;
                    for (int32_t z = 0; z <= 1; z++) {
                        for (int32_t y = 0; y <= 1; y++) {
                            for (int32_t x = 0; x <= 1; x++, iLeaf++) {
                                Eigen::Vector3i leafChunk = cluster_chunk + Eigen::Vector3i(x, y, z);
                                Eigen::Vector3f leafPos = leafChunk.cast<float>() * leafResolution;

                                // get clusters of closest points to this leaf
                                Node*& closestPoints_a = leafPoints_a->children[iLeaf];
                                Node*& closestPoints_b = leafPoints_b->children[iLeaf];

                                // check validity of pointer
                                if (closestPoints_b == nullptr) continue;
                                else if (closestPoints_a == nullptr) {
                                    closestPoints_a = closestPoints_b;
                                    continue;
                                }
                                
                                // count total children in both leaves
                                std::vector<const Eigen::Vector3f*> children;
                                children.reserve(closestPoints_a->leafPoints.size() * 2);
                                for (uint32_t i = 0; i < closestPoints_a->leafPoints.size(); i++) {
                                    if (closestPoints_a->leafPoints[i] != nullptr) {
                                        children.push_back(closestPoints_a->leafPoints[i]);
                                    }
                                    if (closestPoints_b->leafPoints[i] != nullptr) {
                                        children.push_back(closestPoints_b->leafPoints[i]);
                                    }
                                }

                                // simply merge if they fit
                                if (children.size() <= closestPoints_a->leafPoints.size()) {
                                    // refill with merged children
                                    for (std::size_t i = 0; i < children.size(); i++) {
                                        closestPoints_a->leafPoints[i] = children[i];
                                    }
                                }
                                // merge only closest points if not
                                else {
                                    // calc distances for each point to this leaf
                                    typedef std::pair<const Eigen::Vector3f*, float> PairedDist;
                                    std::vector<PairedDist> distances;
                                    distances.reserve(children.size());
                                    for (std::size_t i = 0; i < children.size(); i++) {
                                        float distSqr = (*children[i] - leafPos).squaredNorm();
                                        distances.emplace_back(children[i], distSqr);
                                    }
                                    // sort via distances
                                    auto sort_fnc = [](PairedDist& a, PairedDist& b){
                                        return a.second < b.second;
                                    };
                                    std::sort(distances.begin(), distances.end(), sort_fnc);

                                    // insert the 8 closest points into a
                                    for (std::size_t i = 0; i < closestPoints_a->leafPoints.size(); i++) {
                                        closestPoints_a->leafPoints[i] = distances[i].first;
                                    }
                                }
                            }
                        }
                    }
                }
                else {
                    // walk down path
                    depth++;
                    path[depth] = 0;
                    nodes_a[depth] = pChild_a;
                    nodes_b[depth] = pChild_b;
                }
            }
            else if (pChild_b != nullptr) {
                // add missing child to A
                nodes_a[depth]->children[iChild] = pChild_b;
            }
        }
    }
    // allocate some space and return a node for misc use
    Node* allocate_misc_node() {
        // check if memory block has enough space left
        if (_memory_blocks.back().check_remaining() < 63/3) {
            _memory_blocks.emplace_back(1<<16);
        }
        auto& memblock = _memory_blocks.back();
        return memblock.get_new();
    }
    
private:
    std::vector<MemoryBlock> _memory_blocks;
    Node* _pRoot;
};