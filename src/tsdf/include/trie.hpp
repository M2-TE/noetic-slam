#pragma once
#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>

class Trie {
    typedef uint64_t Key;
    typedef uint64_t Value;
    union Node {
        std::array<Node*, 8> children;
        std::array<uint64_t, 8> leafClusters;
        static_assert(sizeof(children) == sizeof(leafClusters));
        static_assert(sizeof(children) == 64);
    };
    struct Path {
        Path(Key key): key(key) {}
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
    inline Path insert(Key key, Value value) {
        Path path(key); // an iterator of sorts
        auto depth = msb / 3 - 1;
        path.nodes[depth] = pNodes;

        Node* pNode = pNodes; // start at root node
        while (depth > 0) {
            auto index = (key >> depth * 3) & 0b111;
            auto& pChild = pNode->children[index];
            // create child if nonexistant
            if (pChild == nullptr) {
                pChild = pNodes + nNodes++;
                pChild->children = {
                    NULL, NULL, NULL, NULL,
                    NULL, NULL, NULL, NULL
                };
            }
            pNode = pChild;
            path.nodes[--depth] = pNode;
        }
        
        // this last node contains values
        auto index = (key >> depth * 3) & 0b111;
        pNode->leafClusters[index] = value;
        return path; // return hint
    }
    inline Value find(Path& hint, Key key) {
        // find lowest common node
        auto depth = msb / 3;
        auto xorKey = hint.key ^ key;
        while (depth > 0) {
            if ((xorKey >> (depth * 3)) & 0b111) break;
            depth--;
        }

        // iterate through trie via key
        Node* pNode = hint.nodes[depth];
        while (depth > 0) {
            auto index = (key >> depth * 3) & 0b111;
            pNode = pNode->children[index];
            if (pNode == nullptr) return 0;
            hint.nodes[--depth] = pNode;
        }

        // this last node contains values
        auto index = (key >> depth * 3) & 0b111;
        return pNode->leafClusters[index];
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