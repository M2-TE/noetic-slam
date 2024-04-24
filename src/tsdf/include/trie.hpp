#pragma once
#include <array>
#include <limits>
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>

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
    static constexpr size_t msb = 63; // the most significant bit of a key
public:
    // the default value of an uninitialized node/leaf
    static constexpr uintptr_t defVal = std::numeric_limits<uintptr_t>::max();
};