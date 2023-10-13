#pragma once

typedef uint32_t DAG_Pointer; // could potentially downgrade to 16 bits
typedef uint64_t DAG_RootPos; // packing 3D position into single 64-bit unsigned integer
typedef uint32_t DAG_LeavesMask; // contains 8 leaves, each of which are 4 bits

// DAG Nodes
struct DAG_Node {
    friend size_t hash_value(const DAG_Node& node) {
        return phmap::HashState().combine(0,
            node.childPointers[0], node.childPointers[1], 
            node.childPointers[2], node.childPointers[3],
            node.childPointers[4], node.childPointers[5], 
            node.childPointers[6], node.childPointers[7]);
    }

    uint32_t childMask : 8; // only really need 8 bits for this atm
    std::array<DAG_Pointer, 8> childPointers; // could make this sparse
};
struct DAG_Leaves {
    DAG_Leaves(DAG_LeavesMask leavesMask) : leavesMask(leavesMask) {}
    friend size_t hash_value(const DAG_Leaves& node) { return (size_t)node.leavesMask; }
    uint32_t leavesMask; // 8 children, 4 bits each
};

// TODO: could use pointer-stable variant of hashmaps to avoid usage of additional vector
struct DAG_RootLevel {
    phmap::flat_hash_map<DAG_RootPos, DAG_Pointer> hashmap;
    std::vector<DAG_Node> roots;
};
struct DAG_StandardLevel {
    phmap::flat_hash_map<DAG_Node, DAG_Pointer> hashmap;
    std::vector<DAG_Node> nodes;
};
struct DAG_LeavesLevel {
    phmap::flat_hash_map<DAG_LeavesMask, DAG_Pointer> hashmap;
    std::vector<DAG_Leaves> leaves;
};

// DAG Levels
template<uint nLevels>
struct DAG_Levels {
private:

public:
    static inline constexpr uint size() { 
        return nLevels; 
    }
    inline DAG_Node& get_root_node(DAG_RootPos rootPos) {
        auto result = dagRootLevel.hashmap.try_emplace(rootPos);
        if (result.second) { // create new root node
            result.first->second = dagRootLevel.roots.size();
            dagRootLevel.roots.emplace_back();
            ROS_INFO("Created root node");
        }
        else ROS_INFO("Found root node");
        return dagRootLevel.roots[result.first->second];
    }
    inline DAG_Node& get_node(DAG_Pointer nodePointer, uint8_t nodeDepth) {
        return dagStandardLevels[nodeDepth - 1].nodes[nodePointer];
    }
    inline DAG_Node& get_node_addr(DAG_Node nodeKey, uint8_t nodeDepth) {
        // auto result = dagStandardLevels[nodeDepth].hashmap.try_emplace(nodeKey);
        // if (result.second) { // create new standard node

        // }
    }
    
    // get leaves node (containing 8 leaves) with hash key
    inline DAG_Pointer get_leaves_node_addr(DAG_LeavesMask leavesMask) {
        auto result = dagLeavesLevel.hashmap.try_emplace(leavesMask);
        if (result.second) { // create new leaves node
            result.first->second = dagLeavesLevel.leaves.size();
            dagLeavesLevel.leaves.emplace_back(leavesMask);
            ROS_INFO("Created Leaves node");
            return result.first->second;
        }
        else ROS_INFO("Found leaves node");
        return result.first->second;
    }

private:
    DAG_RootLevel dagRootLevel;
    DAG_LeavesLevel dagLeavesLevel;
    std::array<DAG_StandardLevel, nLevels - 2> dagStandardLevels;
};