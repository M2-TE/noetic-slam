#pragma once
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
//
#include <parallel_hashmap/phmap.h>

struct Octree {
    typedef uint64_t Key; // only 63 bits in use
    typedef uint64_t Leaf;
    union Node {
        std::array<Node*, 8> children;
        std::array<Leaf, 8> leaves;
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
    Leaf& find(Key key) {
        // depth from 20 (root) to 0 (leaf)
        size_t rDepth = 63/3 - 1;
        
        // heck if memory block has enough space left
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
    Node* find(Key key, size_t target_depth) {
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
    void merge(Octree& other, Key key, uint32_t startDepth, void(*resolver)(Leaf&, Leaf&)) {
        uint32_t pathLength = 63/3 - startDepth;
        std::vector<uint8_t> path(pathLength);
        std::vector<Node*> nodes_a(pathLength);
        std::vector<Node*> nodes_b(pathLength);
        
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
                    resolver(
                        nodes_a[depth]->leaves[iChild], 
                        nodes_b[depth]->leaves[iChild]
                    );
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
    
private:
    std::vector<MemoryBlock> _memory_blocks;
    Node* _pRoot;
};