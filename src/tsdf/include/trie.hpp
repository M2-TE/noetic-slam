#pragma once
#include <array>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <map>
#include <unistd.h>
#include <stdlib.h>
#include <vector>

struct Octree {
    typedef uint64_t Key; // only 63 bits in use
    typedef uint64_t Leaf;
    union Node {
        std::array<Node*, 8> children;
        std::array<Leaf, 8> leaves;
        static_assert(sizeof(children) == sizeof(leaves));
    };
    struct PathCache {
        PathCache(Octree& octree) {
            key = 0;
            nodes.back() = octree.pNodes;
        }
        std::array<Node*, 63/3> nodes;
        Key key;
    };
    Octree& operator=(Octree& other) = delete;
    Octree& operator=(Octree&& other) = delete;
    Octree(Octree& other) {
        pNodes = other.pNodes;
        mergedNodes = other.mergedNodes;
        nNodes = other.nNodes;
        nCapacity = other.nCapacity;
        // clear out relevant data from other octree
        other.pNodes = nullptr;
        other.bOwnsRoot = false;
        other.mergedNodes.clear();
    }
    Octree(Octree&& other) {
        pNodes = other.pNodes;
        mergedNodes = other.mergedNodes;
        nNodes = other.nNodes;
        nCapacity = other.nCapacity;
        // clear out relevant data from other octree
        other.pNodes = nullptr;
        other.bOwnsRoot = false;
        other.mergedNodes.clear();
    }
    Octree(uint32_t nMaxCapacity): nCapacity(nMaxCapacity) {
        // assume posix
        static_assert(__unix__);
        size_t size = sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
        if (size != sizeof(Node)) std::cerr << "cache line size mismatch\n";

        // alignment according to cache line size, which should be 64
        void* pRaw = std::aligned_alloc(size, nCapacity * size);
        pNodes = static_cast<Node*>(pRaw);

        // create root node (push onto mem of first thread)
        std::memset(pNodes, 0, sizeof(Node));
        nNodes++;
    }
    ~Octree() {
        if (!bOwnsRoot) return;
        free(pNodes);
        for (Node* root: mergedNodes) free(root);
        // std::cout << mergedNodes.size() << '\n';
    }

    Leaf& find_deprecated(Key key) {
        auto level = 63 - 3; // one level below root
        Node* pNode = pNodes;
        while (level > 0) {
            // extract the 3 relevant bits for current level
            auto index = (key >> level) & 0b111;
            auto& pChild = pNode->children[index];
            // create child if nonexistant
            if (pChild == nullptr) {
                pChild = pNodes + nNodes++;
                if (nNodes >= nCapacity) std::cout << "capacity reached\n";
                std::memset(pChild, 0, sizeof(Node));
            }
            pNode = pChild;
            level -= 3;
        }
        
        // this last node contains leaves
        auto index = (key >> level) & 0b111;
        return pNode->leaves[index];
    }
    Leaf& find_cached(Key key, PathCache& cache) {
        // read path cache
        key &= 0x7fffffffffffffff; // just in case..
        auto startingDepth = read_cache(key, cache);
        auto startingNode = cache.nodes[startingDepth];
        
        // begin traversal
        auto rDepth = startingDepth;
        Node* pNode = startingNode;
        while (rDepth > 0) {
            // extract the 3 relevant bits for current depth
            auto index = (key >> rDepth * 3) & 0b111;
            auto& pChild = pNode->children[index];
            // create child if nonexistant
            if (pChild == nullptr) {
                pChild = pNodes + nNodes++;
                if (nNodes >= nCapacity) std::cout << "capacity reached\n";
                std::memset(pChild, 0, sizeof(Node));
            }
            pNode = pChild;
            cache.nodes[--rDepth] = pNode;
        }

        // update cache key to the current path
        cache.key = key;
        
        // this last node contains values
        auto index = (key >> rDepth * 3) & 0b111;
        return pNode->leaves[index];
    }
    Node* find_node(Key key, uint32_t targetDepth) {
        uint32_t depth = 0;
        Node* pNode = pNodes;

        // no need to insert or check for validity
        while (depth < targetDepth) {
            Key index = (key >> (60 - depth*3)) & 0b111;
            pNode = pNode->children[index];
            depth++;
        }
        return pNode;
    }

    std::pair<std::vector<Key>, uint32_t> find_collisions_and_merge(Octree& other, uint32_t minCollisions) {
        std::vector<Key> collisions;
        std::map<Key, Node*> parentsA;
        std::map<Key, Node*> layerA, layerB;
        uint32_t prevCollisions = 0;
        uint32_t depth = 1;

        // pass ownership of memory block from other octree to this one
        mergedNodes.push_back(other.pNodes);
        other.bOwnsRoot = false;
        mergedNodes.insert(mergedNodes.end(), other.mergedNodes.cbegin(), other.mergedNodes.cend());

        // find collisions until collision target is met
        for (; depth < 63/3; depth++) {
            collisions.clear();
            // get node layers via root node
            if (depth == 1) {
                // get the first layer after root for both trees
                for (uint64_t i = 0; i < 8; i++) {
                    Node* pChild = nullptr;
                    pChild = pNodes->children[i];
                    if (pChild != nullptr) layerA[i << (63 - depth*3)] = pChild;
                    pChild = other.pNodes->children[i];
                    if (pChild != nullptr) layerB[i << (63 - depth*3)] = pChild;
                }
            }
            // get node layers via prev layer nodes
            else {
                // create new layers from the current one's children
                decltype(layerA) newA;
                for (auto& node: layerA) {
                    for (uint64_t i = 0; i < 8; i++) {
                        Node* pChild = node.second->children[i];
                        if (pChild != nullptr) {
                            // construct key for this child node
                            uint64_t key = i << (63 - depth*3);
                            key |= node.first;
                            // insert into layer
                            newA[key] = pChild;
                        }
                    }
                }
                decltype(layerB) newB;
                for (auto& node: layerB) {
                    for (uint64_t i = 0; i < 8; i++) {
                        Node* pChild = node.second->children[i];
                        if (pChild != nullptr) {
                            // construct key for this child node
                            uint64_t key = i << (63 - depth*3);
                            key |= node.first;
                            // insert into layer
                            newB[key] = pChild;
                        }
                    }
                }
                // overwrite old layers
                parentsA = std::move(layerA);
                layerA = std::move(newA);
                layerB = std::move(newB);
            }
            // check for collisions
            std::vector<Key> queuedRemovals;
            for (auto& node: layerB) {
                // on collision, save to vector
                if (layerA.contains(node.first)) {
                    collisions.push_back(node.first);
                }
                // when node is missing from main octree, simply give it the pointer
                else {
                    // get only last 3 bits for child index
                    Key key = node.first >> (63 - depth*3);
                    key &= 0b111;

                    if (depth == 1) {
                        // use pNode as parent
                        pNodes->children[key] = node.second;
                    }
                    else {
                        // scrape the last 3 bits off the key
                        auto parentDepth = depth - 1;
                        Key mask = (1ull << parentDepth*3) - 1;
                        mask = mask << (63 - parentDepth*3);
                        // use it to find parent and insert into its children
                        Node* pParent = parentsA[node.first & mask];
                        pParent->children[key] = node.second;
                    }
                    
                    // queue node for removal (in the temp layerB)
                    queuedRemovals.push_back(node.first);
                }
            }

            // remove already inserted keys from layerB
            for (auto& key: queuedRemovals) layerB.erase(key);
            if (layerB.empty()) {
                std::cout << "Breaking out of merge early\n";
                return { {}, 0 }; // no need for merging at all
            }

            // only check for break condition if collision count has changed
            // compared to previous depth
            if (depth > 1 && collisions.size() != prevCollisions) {
                // break if collision target is met
                if (collisions.size() >= minCollisions) break;
                // also break if the number of collisions is shrinking before reaching target
                if (collisions.size() < prevCollisions) break;
            }
            prevCollisions = collisions.size();
        }
        return { collisions, depth };
    }
    void resolve_collisions(Octree& other, Key key, uint32_t depth, void(*resolver)(Node*,Node*)) {
        uint32_t pathLength = 63/3 - depth;
        uint32_t leafDepth = pathLength - 2;
        std::vector<uint8_t> path(pathLength);
        std::vector<Node*> nodesA(pathLength);
        std::vector<Node*> nodesB(pathLength);

        path[0] = 0;
        nodesA[0] = find_node(key, depth);
        nodesB[0] = other.find_node(key, depth);
        uint32_t pathDepth = 0;

        // begin traversal
        while (path[0] <= 8) {
            auto iChild = path[pathDepth]++;
            if (iChild >= 8) {
                // go back up to parent
                pathDepth--;
                continue;
            }
            // A
            Node* pParentA = nodesA[pathDepth];
            Node* pA = pParentA->children[iChild];
            // B
            Node* pParentB = nodesB[pathDepth];
            Node* pB = pParentB->children[iChild];
            
            if (pA != nullptr) {
                // if this node is missing from B, simply skip it
                if (pB == nullptr) continue;

                if (pathDepth >= leafDepth) {
                    // leaf reached
                    resolver(pA, pB);
                }
                else {
                    // walk down path
                    pathDepth++;
                    path[pathDepth] = 0;
                    nodesA[pathDepth] = pA;
                    nodesB[pathDepth] = pB;
                }
            }
            else if (pB != nullptr) {
                // if this node is missing from A, add it
                pParentA->children[iChild] = pB;
            }
        }
    }

    inline auto leading_zeroes(unsigned long v) { return __builtin_clzl(v); }
    inline auto leading_zeroes(unsigned long long v) { return __builtin_clzll(v); }
    inline size_t read_cache(Key key, PathCache& cache) {
        Key xorKey = cache.key ^ key;
        if (xorKey == 0) return 0;
        if (cache.key == 0) return 60/3;
        Key leadingBits = leading_zeroes(xorKey);
        Key level = sizeof(Key) * 8 - leadingBits - 1;
        level /= 3; // 3 bits per level
        return level;
    }

    Node* pNodes = nullptr;
    bool bOwnsRoot = true;
    std::vector<Node*> mergedNodes;
    uint_fast32_t nNodes = 0;
    uint_fast32_t nCapacity = 0;
};