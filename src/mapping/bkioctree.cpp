#include <cmath>

#include "bkioctree.h"

namespace see_csom {

    unsigned short SemanticOcTree::max_depth = 0;

    OcTreeHashKey node_to_hash_key(unsigned short depth, unsigned short index) {
        return (depth << 16) + index;
    }

    void hash_key_to_node(OcTreeHashKey key, unsigned short &depth, unsigned short &index) {
        depth = (unsigned short) (key >> 16);
        index = (unsigned short) (key & 0xFFFF);
    }

    //语义八叉树初始化，就是得到所有深度所有节点的索引
    SemanticOcTree::SemanticOcTree() {
        if (max_depth <= 0)
            node_arr = nullptr;  //最大深度没有，那就是空的
        else {
            node_arr = new SemanticOcTreeNode *[max_depth]();  //否则新建一个长度为max_depth节点索引，每个深度都有一些节点
            for (unsigned short i = 0; i < max_depth; ++i) {
                node_arr[i] = new SemanticOcTreeNode[(int) pow(8, i)]();   //每个深度都有相应个数节点
            }
        }
    }

    SemanticOcTree::~SemanticOcTree() {
        if (node_arr != nullptr) {
            for (unsigned short i = 0; i < max_depth; ++i) {
                if (node_arr[i] != nullptr) {
                    delete[] node_arr[i];
                }
            }
            delete[] node_arr;
        }
    }

    SemanticOcTree::SemanticOcTree(const SemanticOcTree &other) {
        if (other.node_arr == nullptr) {
            node_arr = nullptr;
            return;
        }
        node_arr = new SemanticOcTreeNode *[max_depth]();
        for (unsigned short i = 0; i < max_depth; ++i) {
            if (other.node_arr[i] != nullptr) {
                int n = (int) pow(8, i);
                node_arr[i] = new SemanticOcTreeNode[n]();
                std::copy(node_arr[i], node_arr[i] + n, other.node_arr[i]);
            } else
                node_arr[i] = nullptr;
        }
    }

    SemanticOcTree &SemanticOcTree::operator=(const SemanticOcTree &other) {
        SemanticOcTreeNode **local_node_arr = new SemanticOcTreeNode *[max_depth]();
        for (unsigned short i = 0; i < max_depth; ++i) {
            if (local_node_arr[i] != nullptr) {
                int n = (int) pow(8, i);
                local_node_arr[i] = new SemanticOcTreeNode[n]();
                std::copy(local_node_arr[i], local_node_arr[i] + n, other.node_arr[i]);
            } else
                local_node_arr[i] = nullptr;
        }

        node_arr = local_node_arr;
        return *this;
    }

    // 判断是否是叶节点：如果这个节点是未剪枝但没有下一层（已经是最后一层了），或者有下一层但是下一层是空的或者下一层的节点是被剪枝的。
    bool SemanticOcTree::is_leaf(unsigned short depth, unsigned short index) const {
        if (node_arr != nullptr && node_arr[depth] != nullptr && node_arr[depth][index].get_state() != State::PRUNED) {
            if (depth + 1 < max_depth) {
                if (node_arr[depth + 1] == nullptr || node_arr[depth + 1][index * 8].get_state() == State::PRUNED)
                    return true;
            } else {
                return true;
            }
        }
        return false;
    }

    bool SemanticOcTree::is_leaf(OcTreeHashKey key) const {
        unsigned short depth = 0;
        unsigned short index = 0;
        hash_key_to_node(key, depth, index);
        return is_leaf(depth, index);
    }

    bool SemanticOcTree::search(OcTreeHashKey key) const {
        unsigned short depth;
        unsigned short index;
        hash_key_to_node(key, depth, index);

        return node_arr != nullptr &&
               node_arr[depth] != nullptr &&
               node_arr[depth][index].get_state() != State::PRUNED;
    }

    // 重载了取值操作，根据哈希键，返回一个节点
    SemanticOcTreeNode &SemanticOcTree::operator[](OcTreeHashKey key) const {
        unsigned short depth;
        unsigned short index;
        hash_key_to_node(key, depth, index);
        return node_arr[depth][index];
    }
}
