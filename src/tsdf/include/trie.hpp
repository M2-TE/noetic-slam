#pragma once
#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>

class Trie {
    // Trie node containing 8 children (2x2x2 voxel chunk)
    union Node {
        std::array<Node*, 8> children;
        std::array<uint64_t, 8> leafClusters;
        static_assert(sizeof(children) == sizeof(leafClusters));
        static_assert(sizeof(children) == 64);
    };
    typedef uint64_t Key;
    typedef uint64_t Value;
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
        static_assert(NULL == 0b0);
        static_assert(NULL == nullptr);
        nNodes = 0;
        (pNodes + nNodes++)->children = {
            NULL, NULL, NULL, NULL,
            NULL, NULL, NULL, NULL
        };
    }
    ~Trie() {
        free(pNodes);
    }

    // insert value without position hint
    inline Node* insert(Key key, Value value) {
        Node* pNode = pNodes; // start at root node
        Key mask = 0b111; // every node covers 3 bits (8 children)
        auto depth = msb;

        // progress through tree
        auto nNew = 0;
        while (depth > 3) {
            depth -= 3;
            auto index = (key >> depth) & mask;
            auto& pChild = pNode->children[index];
            // create child if nonexistant
            if (pChild == nullptr) {
                pChild = pNodes + nNodes++;
                pChild->children = {
                    NULL, NULL, NULL, NULL,
                    NULL, NULL, NULL, NULL
                };
                nNew++;
            }
            pNode = pChild;
        }

        // this last node (current pNode) does not contain pointers
        // it instead contains the direct values
        depth -= 3;
        auto index = (key >> depth) & mask;
        pNode->leafClusters[index] = value;

        return pNode; // return node as potential hint
    }
    inline Value find(Key key) {
        return 0;
    }
    inline Value find(Key hint, Node* pHint, Key key) {
        return 0;
    }

    void printstuff() {
        std::cout << "populated " << nNodes
            << " of the potential " << nMax
            << " nodes (" << (float)nNodes / (float)nMax * 100.0f<< "%)\n";
    }
    
private:
    Node* pNodes;
    size_t nNodes;
    static constexpr size_t nMax = 1'000'000;
    static constexpr size_t msb = 63; // the most significant bit of a key
};