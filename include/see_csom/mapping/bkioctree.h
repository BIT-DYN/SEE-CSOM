#pragma once

#include <stack>
#include <vector>

#include "point3f.h"
#include "bkioctree_node.h"

// Octoreee是一个block中的一个八叉树，OcTreeHashKey说明了在这个block中结点所处的深度的位置

namespace see_csom {

    /// Hash key to index OcTree nodes given depth and the index in that layer.  Octree的哈希键索引给定深度的八叉树节点和该层中的索引。
    typedef int OcTreeHashKey;   

    /// Convert from node to hask key. 从节点转换为Octree的hask键。转化关系就是深度值左移16位
    OcTreeHashKey node_to_hash_key(unsigned short depth, unsigned short index);

    /// Convert from hash key to node. 从hask键转换为节点
    void hash_key_to_node(OcTreeHashKey key, unsigned short &depth, unsigned short &index);

    /*
     * @brief A simple OcTree to organize occupancy data in one block.
     *
     * OcTree doesn't store positions of nodes in order to reduce memory usage.
     * The nodes in OcTrees are indexed by OcTreeHashKey which can be used to
     * retrieve positions later (See Block).
     * For the purpose of mapping, this OcTree has fixed depth which should be
     * set before using OcTrees.
     */

    // 一种简单的八叉树，用于在一个块中组织占用数据。
    // 八叉树不存储节点的位置以减少内存使用。
    // 八叉树中的节点由OcTreeHashKey索引，OcTreeHashKey可用于
    // 稍后检索位置（参见方框）。
    // 为了建图的目的，这个八叉树有固定的深度，应该在使用八叉树之前设置。
    class SemanticOcTree {
        friend class SemanticBKIOctoMap;

    public:
        SemanticOcTree();

        ~SemanticOcTree();

        SemanticOcTree(const SemanticOcTree &other);

        SemanticOcTree &operator=(const SemanticOcTree &other);

        /*
         * @brief Rursively pruning OcTreeNodes with the same state.
         *
         * Prune nodes by setting nodes to PRUNED.
         * Delete the layer if all nodes are pruned.
         */
        //bool prune();

        /// @return true if this node is a leaf node.  是否是叶节点
        bool is_leaf(OcTreeHashKey key) const;

        /// @return true if this node is a leaf node.  对于给定深度和索引，判断是否是叶节点
        bool is_leaf(unsigned short depth, unsigned short index) const;

        /// @return true if this node exists and is not pruned.  如果此节点存在且未剪枝，则为true。
        bool search(OcTreeHashKey key) const;

        /// @return Occupancy of the node (without checking if it exists!) 根据数哈希键返回节点
        SemanticOcTreeNode &operator[](OcTreeHashKey key) const;

        /// Leaf iterator for OcTrees: iterate all leaf nodes not pruned.  八叉树的叶迭代器：迭代所有未修剪的叶节点。
        class LeafIterator : public std::iterator<std::forward_iterator_tag, SemanticOcTreeNode> {
        public:
            LeafIterator() : tree(nullptr) { }

            LeafIterator(const SemanticOcTree *tree)
                    : tree(tree != nullptr && tree->node_arr != nullptr ? tree : nullptr) {  //看看tree是不是空的，空的就返回空的
                if (tree != nullptr) {
                    stack.emplace(0, 0);    //先压进去0,0，也就是第一个大块，然后++
                    stack.emplace(0, 0);
                    ++(*this);
                }
            }

            LeafIterator(const LeafIterator &other) : tree(other.tree), stack(other.stack) { }

            LeafIterator &operator=(const LeafIterator &other) {
                tree = other.tree;
                stack = other.stack;
                return *this;
            }

            bool operator==(const LeafIterator &other) const {
                return (tree == other.tree) &&
                       (stack.size() == other.stack.size()) &&
                       (stack.size() == 0 || (stack.size() > 0 &&
                                              (stack.top().depth == other.stack.top().depth) &&
                                              (stack.top().index == other.stack.top().index)));
            }

            bool operator!=(const LeafIterator &other) const {
                return !(this->operator==(other));
            }

            // 叶迭代器++，就是得到一个，然后继续推进
            LeafIterator operator++(int) {
                LeafIterator result(*this);
                ++(*this);
                return result;
            }

            // 只遍历叶节点
            LeafIterator &operator++() {
                if (stack.empty()) {
                    tree = nullptr;
                } else {
                    stack.pop(); //拿出来一个
                    while (!stack.empty() && !tree->is_leaf(stack.top().depth, stack.top().index))  //当下一个不是空的但不是叶节点的时候，把下一个的子节点弄进去
                        single_inc();
                    if (stack.empty())
                        tree = nullptr;
                }
                return *this;
            }

            // 重载了取值操作符，就是得到当前这个节点
            inline SemanticOcTreeNode &operator*() const {
                return (*tree)[get_hash_key()];
            }

            inline SemanticOcTreeNode &get_node() const {
                return operator*();
            }

            inline OcTreeHashKey get_hash_key() const {
                OcTreeHashKey key = node_to_hash_key(stack.top().depth, stack.top().index);
                return key;
            }

        protected:
            void single_inc() {
                StackElement top(stack.top());
                stack.pop();

                for (int i = 0; i < 8; ++i) {
                    stack.emplace(top.depth + 1, top.index * 8 + i);
                }
            }

            struct StackElement {
                unsigned short depth;
                unsigned short index;

                StackElement(unsigned short depth, unsigned short index)
                        : depth(depth), index(index) { }
            };  //一个包含深度和索引的类

            const SemanticOcTree *tree;
            std::stack<StackElement, std::vector<StackElement> > stack;  //一个包含深度和索引的类的堆栈
        };

        /// @return the beginning of leaf iterator  返回叶迭代器的第一个结点
        inline LeafIterator begin_leaf() const { return LeafIterator(this); };

        /// @return the end of leaf iterator  //返回叶迭代器的最后一个节点
        inline LeafIterator end_leaf() const { return LeafIterator(nullptr); };

    private:
        SemanticOcTreeNode **node_arr;  //一个节点的索引
        static unsigned short max_depth;
    };
}
