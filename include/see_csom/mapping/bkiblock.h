#pragma once

#include <unordered_map>
#include <array>

#include "point3f.h"
#include "bkioctree_node.h"
#include "bkioctree.h"

namespace see_csom {

    /// Hask key to index Block given block's center.  BLOCK的Hask键用于索引给定块中心的块。
    typedef int64_t BlockHashKey;

    /// Initialize Look-Up Table 初始化了一个深度为max_depth的查找表
    // 这个里面存储的是每块(包括所有深度的块)的索引和坐标
    std::unordered_map<OcTreeHashKey, point3f> init_key_loc_map(float resolution, unsigned short max_depth);

    // 这是一个索引图，存储了最小块的新的索引，和哈希键。
    std::unordered_map<unsigned short, OcTreeHashKey> init_index_map(const std::unordered_map<OcTreeHashKey, point3f> &key_loc_map,
                                                                     unsigned short max_depth);

    /// Extended Block  扩展块
#ifdef PREDICT
    typedef std::array<BlockHashKey, 27> ExtendedBlock;
#else
    typedef std::array<BlockHashKey, 7> ExtendedBlock;   // 7个类型为 int64_t （索引） 的向量
#endif

    /// Convert from block to hash key.  从块转换为哈希键(整个地图中block对应的哈希键)
    BlockHashKey block_to_hash_key(point3f center);

    /// Convert from block to hash key. 
    BlockHashKey block_to_hash_key(float x, float y, float z);

    /// Convert from hash key to block. 从哈希键转换为块，得到绝对坐标
    point3f hash_key_to_block(BlockHashKey key);

    /// Get current block's extended block.  获取当前块的扩展块（上下左右前后6个）
    ExtendedBlock get_extended_block(BlockHashKey key);  

    /*
     * @brief Block is built on top of OcTree, providing the functions to locate nodes.
     *
     * Block stores the information needed to locate each OcTreeNode's position:
     * fixed resolution, fixed block_size, both of which must be initialized.
     * The localization is implemented using Loop-Up Table.
     */

    // block建立在八叉树之上，提供定位节点的功能。
    // 块存储定位每个八叉树节点位置所需的信息：固定分辨率，固定块大小，两者都必须初始化。
    // 使用循环表实现定位。
    class Block : public SemanticOcTree {
        friend BlockHashKey block_to_hash_key(point3f center);

        friend BlockHashKey block_to_hash_key(float x, float y, float z);

        friend point3f hash_key_to_block(BlockHashKey key);

        friend ExtendedBlock get_extended_block(BlockHashKey key);

        friend class SemanticBKIOctoMap;


    public:
        Block();

        Block(point3f center);

        /// @return location of the OcTreeNode given OcTree's LeafIterator.  给定八叉树的叶索引，返回八叉树节点的位置（绝对坐标）。
        inline point3f get_loc(const LeafIterator &it) const {
            return Block::key_loc_map[it.get_hash_key()] + center;
        }

        /// @return size of the OcTreeNode given OcTree's LeafIterator.  给定叶节点的叶索引，返回该节点的尺寸大小
        inline float get_size(const LeafIterator &it) const {
            unsigned short depth, index;
            hash_key_to_node(it.get_hash_key(), depth, index);
            return float(size / pow(2, depth));
        }

        /// @return center of current Block.   当前快的中心绝对坐标
        inline point3f get_center() const { return center; }

        /// @return min lim of current Block.  当前块的最小极限。
        inline point3f get_lim_min() const { return center - point3f(size / 2.0f, size / 2.0f, size / 2.0f); }

        /// @return max lim of current Block.  当前块的最大极限。
        inline point3f get_lim_max() const { return center + point3f(size / 2.0f, size / 2.0f, size / 2.0f); }

        /// @return ExtendedBlock of current Block. 得到当前快的扩展快的哈希键
        ExtendedBlock get_extended_block() const;

        /// @return ExtendedBlock of current Block. 得到当前块二维的扩展快的哈希键
        ExtendedBlock get_2d_extended_block() const;

        OcTreeHashKey get_node(unsigned short x, unsigned short y, unsigned short z) const; //根据block中的xyz形式索引得到哈希键

        point3f get_point(unsigned short x, unsigned short y, unsigned short z) const;   //根据block中的xyz形式索引得到点的绝对坐标

        void get_index(const point3f &p, unsigned short &x, unsigned short &y, unsigned short &z) const; //根据点绝对坐标，得到xyz形式索引

        SemanticOcTreeNode &search(float x, float y, float z) const;   //根据点绝对坐标获得节点

        SemanticOcTreeNode &search(point3f p) const;  //根据点坐标获得节点

    private:
        // Loop-Up Table
        static std::unordered_map<OcTreeHashKey, point3f> key_loc_map;   
        static std::unordered_map<unsigned short, OcTreeHashKey> index_map;
        static float resolution;  //地图的分辨率
        static float size;  //每个块的尺寸= resolution*2^（depth - 1）
        static unsigned short cell_num;  //在某一维度block分块的数目

        point3f center;  //块的中心绝对坐标
    };
}
