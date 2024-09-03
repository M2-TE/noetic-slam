#pragma once
#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <vector>
//
#include "dag/leaf_cluster.hpp"

// TODO: move this stuff out of hpp and into the dag class

// pure helper struct, so no ctor/dtor
struct Node {
    Node() = delete;
    ~Node() = delete;
    // reinterpret an explicit node address
    static Node* conv(std::vector<uint32_t>& data, uint32_t addr) {
        return reinterpret_cast<Node*>(&data[addr]);
    }
    
    // check header mask for a specific child
    bool contains_child(uint32_t iChild) const {
        uint32_t childBit = 1 << iChild;
        return header & childBit;
    }
    // check header mask for a specific leaf
    bool contains_leaf(uint32_t iLeaf) const {
        std::cout << std::bitset<32>(header) << '\n';
        return contains_child(iLeaf);
    }
    // retrieve child addr from sparse children array
    uint32_t get_child_addr(uint32_t iChild) const {
        // popcount lookup table: https://stackoverflow.com/a/51388543
        constexpr uint8_t bitcount[] = {
            0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
            1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
            1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
            1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
            2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
            3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
            3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
            4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
        };
        // count how many children come before this one
        uint32_t childBit = 1 << iChild;
        uint32_t masked = header & (childBit - 1);
        uint32_t nChildren = bitcount[masked];
        // return actual index to child
        return children[nChildren];
    }
    
// private:
    uint32_t header; // contains child flags and child count
    std::array<uint32_t, 8> children; // sparse and compacted
};