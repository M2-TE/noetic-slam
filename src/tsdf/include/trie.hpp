#pragma once
#include <array>
#include <bitset>
#include <cstring>
#include <limits>
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>

struct Octree {
    typedef uint64_t Key; // only 63 bits in use
    typedef uint64_t Leaf;
    union Node {
        std::array<Node*, 8> children;
        std::array<Leaf, 8> leaves;
        static_assert(sizeof(children) == sizeof(leaves));
    };
    struct PathCache {
        std::array<Node*, 63/3> nodes;
        Key key;
    };

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
        // create dummy node to fill path cache
        cache.key = 0;
        cache.nodes.back() = pNodes;
    }
    ~Octree() {
        free(pNodes);
    }

    Leaf& find(Key key) {
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
    Leaf& find_cached(Key key) {
        // read path cache
        key &= 0x7fffffffffffffff; // just in case..
        auto startingDepth = read_cache(key);
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

    inline auto leading_zeroes(unsigned long v) { return __builtin_clzl(v); }
    inline auto leading_zeroes(unsigned long long v) { return __builtin_clzll(v); }
    inline size_t read_cache(Key key) {
        Key xorKey = cache.key ^ key;
        if (xorKey == 0) return 0;
        if (cache.key == 0) return 60/3;
        Key leadingBits = leading_zeroes(xorKey);
        Key level = sizeof(Key) * 8 - leadingBits - 1;
        level /= 3; // 3 bits per level
        return level;
    }

    Node* pNodes = nullptr;
    uint_fast32_t nNodes = 0;
    uint_fast32_t nCapacity = 0;
    PathCache cache;
};

class Trie {
public:
    typedef uint64_t Key;
    typedef uint64_t Value;
    union Node {
        std::array<Node*, 8> children;
        std::array<uint64_t, 8> leafClusters;
        static_assert(sizeof(children) == sizeof(leafClusters));
        static_assert(sizeof(children) == 64);
    };
    
private:
    struct Path {
        std::array<Node*, 63/3> nodes;
        Key key;
    };

public:
    Trie() {
        // assume posix
        static_assert(__unix__);
        static_assert(sizeof(Node) == 64);
        if (sysconf(_SC_LEVEL1_DCACHE_LINESIZE) != 64) std::cerr << "cache line size mismatch\n";

        // allocate a (hopefully) sufficient chunk of aligned memory
        // alignment according to cache line size, which should be 64
        void* pData = std::aligned_alloc(64, nMax * sizeof(Node));
        pNodes = static_cast<decltype(pNodes)>(pData);

        // first node will be root node
        static_assert(nullptr == 0b0);
        static_assert((uintptr_t)NULL == 0b0);
        nNodes = 0;
        (pNodes + nNodes++)->children = {
            (Node*)defVal, (Node*)defVal, (Node*)defVal, (Node*)defVal,
            (Node*)defVal, (Node*)defVal, (Node*)defVal, (Node*)defVal
        };

        // keep track of last path to speed up subsequent accesses
        cache.key = 0;
        cache.nodes.back() = pNodes;
        // fill cache with valid nodes
        insert(0x7fffffffffffffff, 0);
    }
    ~Trie() {
        free(pNodes);
    }

    inline void insert(Key key, Value value) {
        find(key) = value;
    }
    inline Value& find(Key key) {
        auto depth = read_cache(key);
        cache.key = key;
        Node* pNode = cache.nodes[depth];
        while (depth > 0) {
            auto index = (key >> depth * 3) & 0b111;
            auto& pChild = pNode->children[index];
            // create child if nonexistant
            if (pChild == (Node*)defVal) {
                pChild = pNodes + nNodes++;
                pChild->children = {
                    (Node*)defVal, (Node*)defVal, (Node*)defVal, (Node*)defVal,
                    (Node*)defVal, (Node*)defVal, (Node*)defVal, (Node*)defVal
                };
            }
            pNode = pChild;
            cache.nodes[--depth] = pNode;
        }
        
        // this last node contains values
        auto index = (key >> depth * 3) & 0b111;
        return pNode->leafClusters[index];
    }
    inline Node* get_root() {
        return pNodes;
    }

    void printstuff() {
        std::cout << "populated " << nNodes
            << " of the potential " << nMax
            << " nodes (" << (float)nNodes / (float)nMax * 100.0f<< "%)\n";
    }
    
private:
    // find depth of lowest common node
    inline size_t read_cache(Key key) {
        auto xorKey = cache.key ^ key;
        if (xorKey == 0) return 0;
        size_t leadingBits = leading_zeroes(xorKey);
        auto depth = sizeof(size_t) * 8 - 1 - leadingBits;
        depth /= 3;
        return depth;
    }
    inline size_t leading_zeroes(unsigned long v) { return __builtin_clzl(v); }
    inline size_t leading_zeroes(unsigned long long v) { return __builtin_clzll(v); }

private:
    Node* pNodes;
    size_t nNodes;
    Path cache;
    static constexpr size_t nMax = 10'000'000;
public:
    // the default value of an uninitialized node/leaf
    static constexpr uintptr_t defVal = std::numeric_limits<uintptr_t>::max();
};