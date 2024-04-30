#pragma once
#include <array>
#include <cstring>
#include <limits>
#include <cstdint>
#include <iostream>
#include <mutex>
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

    Octree(uint32_t nThreads, uint32_t nNodes) {
        // assume posix
        static_assert(__unix__);
        size_t size = sysconf(_SC_LEVEL1_DCACHE_LINESIZE);
        if (size != sizeof(Node)) std::cerr << "cache line size mismatch\n";

        // alignment according to cache line size, which should be 64
        void* pRaw = std::aligned_alloc(size, nNodes * size);
        pNodes = static_cast<Node*>(pRaw);

        // keep track of allocation size
        nCapacity = nNodes;
        perThreadNodes.resize(nThreads, 0);
        perThreadOffset.reserve(nThreads);
        for (uint_fast32_t i = 0; i < nThreads; i++) {
            uint_fast32_t span = nNodes / nThreads;
            perThreadOffset.push_back(i * span);
        }

        // create root node (push onto mem of first thread)
        perThreadNodes.front()++;
        std::memset(pNodes, 0, sizeof(Node));
    }
    ~Octree() {
        free(pNodes);
    }

    void insert(Key key, Leaf value, uint_fast32_t threadIndex) {
        auto offset = perThreadOffset[threadIndex];
        auto nNodes = perThreadNodes[threadIndex];
        std::cout << offset << '\n';

        constexpr uint_fast32_t root = 63;
        constexpr uint_fast32_t levelBits = 3;
        auto level = root - levelBits; // one level below root
        Node* pNode = pNodes;
        while (level > 0) {
            // extract the 3 relevant bits for current level
            auto index = (key >> level) & 0b111;
            // index into children of current node
            auto& pChild = pNode->children[index];
            // create child if nonexistant
            if (pChild == nullptr) {
                pChild = pNodes + offset + nNodes++; // todo: adjust for per-thread mem block
                std::memset(pChild, 0, sizeof(Node));
            }
            pNode = pChild;
            level -= levelBits;
        }

        // update node count for current thread
        perThreadNodes[threadIndex] = nNodes;
        
        // this last node contains values
        auto index = (key >> level * 3) & 0b111;
        pNode->leaves[index] = value;
    }

private:
    // best one so far:
    std::mutex threadKeysMutex;
    std::vector<Key> threadKeys;

    std::vector<int> voteForHighestLevelWhereSingleMutexGovernsAllChildren;

    Node* pNodes;
    uint_fast32_t nCapacity; // total node capacity
    // partition block of memory between threads
    std::vector<uint_fast32_t> perThreadOffset; // each thread starts index 0 at this offset
    std::vector<uint_fast32_t> perThreadNodes; // each thread occupies data starting from offset
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
        std::array<Node*, 21> nodes;
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
    inline size_t leading_zeroes(unsigned long v) { return __builtin_clzl(v); }
    inline size_t leading_zeroes(unsigned long long v) { return __builtin_clzll(v); }
    // find depth of lowest common node
    inline size_t read_cache(Key key) {
        auto xorKey = cache.key ^ key;
        if (xorKey == 0) return 0;
        size_t leadingBits = leading_zeroes(xorKey);
        auto depth = sizeof(size_t) * 8 - 1 - leadingBits;
        depth /= 3;
        return depth;
    }

private:
    Node* pNodes;
    size_t nNodes;
    Path cache;
    static constexpr size_t nMax = 10'000'000;
public:
    // the default value of an uninitialized node/leaf
    static constexpr uintptr_t defVal = std::numeric_limits<uintptr_t>::max();
};