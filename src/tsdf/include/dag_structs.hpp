#pragma once

typedef uint32_t DAG_Pointer; // could potentially downgrade to 16 bits
typedef uint64_t DAG_RootPos; // packing 3D position into single 64-bit unsigned integer

// DAG Nodes
struct DAG_Node {
    friend size_t hash_value(const DAG_Node& node) {
        return phmap::HashState().combine(0,
            node.childPointers[0], node.childPointers[1], 
            node.childPointers[2], node.childPointers[3],
            node.childPointers[4], node.childPointers[5], 
            node.childPointers[6], node.childPointers[7]);
    }

    uint32_t childMask : 8;
    std::array<DAG_Pointer, 8> childPointers; // could make this sparse
};
struct DAG_Leaf {
    friend size_t hash_value(const DAG_Leaf& node) { return (size_t)node.childMask; }
    uint32_t childMask : 8;
};

// DAG Levels
struct DAG_RootLevel {
    phmap::flat_hash_map<DAG_RootPos, DAG_Pointer> hashmap;
    std::vector<DAG_Node> roots;
};
struct DAG_Level {
    phmap::flat_hash_map<DAG_Node, DAG_Pointer> hashmap;
    std::vector<DAG_Node> nodes;
};
struct DAG_LeafLevel {
    phmap::flat_hash_map<DAG_Leaf, DAG_Pointer> hashmap;
    std::vector<DAG_Node> leaves;
};