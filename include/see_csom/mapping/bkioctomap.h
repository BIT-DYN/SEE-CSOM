#pragma once

#include <unordered_map>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "rtree.h"
#include "bkiblock.h"
#include "bkioctree_node.h"
#include "seg_point_type.h"

namespace see_csom {

    /// PCL PointCloud types as input
    typedef pcl::PointXYZL PCLPointType;  //带标签的点
    typedef pcl::PointCloud<PCLPointType> PCLPointCloud;  //带标签的点云

    /*
     * @brief BGKOctoMap
     *
     * Bayesian Generalized Kernel Inference for Occupancy Map Prediction
     * The space is partitioned by Blocks in which OcTrees with fixed
     * depth are rooted. Occupancy values in one Block is predicted by 
     * its ExtendedBlock via Bayesian generalized kernel inference.
     */



    // 基于贝叶斯广义核推理的地图占有率预测
    // 空间由块分割，块中有固定深度的八叉树。利用扩展块的贝叶斯广义核推理，预测一个块的占用率。 
    class SemanticBKIOctoMap {
    public:
        /// Types used internally
        typedef std::vector<point3f> PointCloud;  // 点的一个容器
        typedef std::pair<point3f, float> GPPointType;  // 一个点，一个float类型，点就是点的坐标，float是标签
        typedef std::vector<GPPointType> GPPointCloud;  //这就是高斯过程点的点云
        typedef RTree<GPPointType *, float, 3, float> MyRTree;  // 一颗3维的边界矩形树，节点为GP点

        

    public:
        SemanticBKIOctoMap();

        /*
         * @param resolution (default 0.1m)
         * @param block_depth maximum depth of OcTree (default 4)
         * @param sf2 signal variance in GPs (default 1.0)
         * @param ell length-scale in GPs (default 1.0)
         * @param noise noise variance in GPs (default 0.01)
         * @param l length-scale in logistic regression function (default 100)
         * @param min_var minimum variance in Occupancy (default 0.001)
         * @param max_var maximum variance in Occupancy (default 1000)
         * @param max_known_var maximum variance for Occuapncy to be classified as KNOWN State (default 0.02)
         * @param free_thresh free threshold for Occupancy probability (default 0.3)
         * @param occupied_thresh occupied threshold for Occupancy probability (default 0.7)
         */
        // 地图的构建
        SemanticBKIOctoMap(float resolution,   //地图分辨率
                unsigned short block_depth,   //每个块的深度值
                int num_class,  //类别数量
                float sf2,  //Actually sigma_0 in sparse kernel  阿尔法0
                float ell,   // Length scale of the sparse kernel  核的范围
                float prior,  //每个种类的先验概率
                float var_thresh,  //区分已知/未知的方差阈值
                float free_thresh,  //free概率阈值
                float occupied_thresh);   //占用概率阈值

        ~SemanticBKIOctoMap();

        /// Set resolution.设置分辨率，是地图的分辨率
        void set_resolution(float resolution);

        /// Set block max depth.设置块的最大深度，和分辨率一起决定了块的大小，得到block的key_loc_map
        void set_block_depth(unsigned short max_depth);

        /// Get resolution.获得分辨率
        inline float get_resolution() const { return resolution; }

        /// Get block max depth. 获得最大深度
        inline float get_block_depth() const { return block_depth; }

        /*
         * @brief Insert PCL PointCloud into BGKOctoMaps.
         * @param cloud one scan in PCLPointCloud format
         * @param origin sensor origin in the scan
         * @param ds_resolution downsampling resolution for PCL VoxelGrid filtering (-1 if no downsampling)
         * @param free_res resolution for sampling free training points along sensor beams (default 2.0)
         * @param max_range maximum range for beams to be considered as valid measurements (-1 if no limitation)
         */

        // 把点云放入BGKOctoMaps中的两种方法，一个是csm语义计数器模型，另一个是BKI模型
        void insert_pointcloud_csm(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                               float free_res = 2.0f,
                               float max_range = -1);

        void insert_map_point(pcl::PointCloud<MapPoint>::Ptr transformed_cells);

        void insert_map_point_single(pcl::PointCloud<MapPoint>::Ptr transformed_cells);

        void insert_map_point_multi(pcl::PointCloud<MapPoint>::Ptr transformed_cells);

        // void fusion_block(Block *block, pcl::PointCloud<MapPoint>::const_iterator it);

        void BKIforGlobalMap( void );

        int block_is_unknow(const BlockHashKey &key);

        void print_vaule(void);

        void print_vaule_2(void);

        void insert_pointcloud(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                               float free_res = 2.0f,
                               float max_range = -1);

        // 测试的代码
        void insert_pointcloud_test(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                               float free_res = 2.0f,
                               float max_range = -1);

        // 之前插入点云的代码
        void insert_pointcloud_raw(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                               float free_res = 2.0f,
                               float max_range = -1);

        //void insert_training_data(const GPPointCloud &cloud);

        /// Get bounding box of the map. 获取当前地图的边界框。
        void get_bbox(point3f &lim_min, point3f &lim_max) const;

        // 光线投射器
        // class RayCaster {
        // public:
        //     RayCaster(const SemanticBKIOctoMap *map, const point3f &start, const point3f &end) : map(map) {
        //         assert(map != nullptr);

        //         _block_key = block_to_hash_key(start);  // 根据点得到某个block的哈希键索引
        //         block = map->search(_block_key);  //找到对应的block
        //         lim = static_cast<unsigned short>(pow(2, map->block_depth - 1));  // 一维下块被分为多少

        //         if (block != nullptr) {
        //             block->get_index(start, x, y, z);   // xyz为起始点的块的索引xyz
        //             block_lim = block->get_center();  //block_lim是当前块的中心，
        //             block_size = block->size;    //block_size是当前块的大小尺寸
        //             current_p = start;  // 当前点
        //             resolution = map->resolution;  // 地图分辨率

        //             int x0 = static_cast<int>((start.x() / resolution));
        //             int y0 = static_cast<int>((start.y() / resolution));
        //             int z0 = static_cast<int>((start.z() / resolution));
        //             int x1 = static_cast<int>((end.x() / resolution));
        //             int y1 = static_cast<int>((end.y() / resolution));
        //             int z1 = static_cast<int>((end.z() / resolution));
        //             dx = abs(x1 - x0);
        //             dy = abs(y1 - y0);
        //             dz = abs(z1 - z0);
        //             n = 1 + dx + dy + dz; //n是起始点到终止点的xyz相差和+1
        //             // x1>x0，则x_inc=1，相等则为0，否则为-1；yz一样
        //             x_inc = x1 > x0 ? 1 : (x1 == x0 ? 0 : -1);
        //             y_inc = y1 > y0 ? 1 : (y1 == y0 ? 0 : -1);
        //             z_inc = z1 > z0 ? 1 : (z1 == z0 ? 0 : -1);
        //             xy_error = dx - dy;
        //             xz_error = dx - dz;
        //             yz_error = dy - dz;
        //             dx *= 2;
        //             dy *= 2;
        //             dz *= 2;
        //         } else {
        //             n = 0;
        //         }
        //     }

        //     inline bool end() const { return n <= 0; }  // 如果n<=0，说明没了

        //     // 找到下一个点
        //     bool next(point3f &p, SemanticOcTreeNode &node, BlockHashKey &block_key, OcTreeHashKey &node_key) {
        //         assert(!end());
        //         bool valid = false;
        //         unsigned short index = x + y * lim + z * lim * lim;  // 索引和xyz值的关系
        //         node_key = Block::index_map[index];
        //         block_key = _block_key;
        //         if (block != nullptr) {
        //             valid = true;
        //             node = (*block)[node_key];
        //             current_p = block->get_point(x, y, z);
        //             p = current_p;
        //         } else {
        //             p = current_p;
        //         }

        //         if (xy_error > 0 && xz_error > 0) {
        //             x += x_inc;
        //             current_p.x() += x_inc * resolution;
        //             xy_error -= dy;
        //             xz_error -= dz;
        //             if (x >= lim || x < 0) {
        //                 block_lim.x() += x_inc * block_size;
        //                 _block_key = block_to_hash_key(block_lim);
        //                 block = map->search(_block_key);
        //                 x = x_inc > 0 ? 0 : lim - 1;
        //             }
        //         } else if (xy_error < 0 && yz_error > 0) {
        //             y += y_inc;
        //             current_p.y() += y_inc * resolution;
        //             xy_error += dx;
        //             yz_error -= dz;
        //             if (y >= lim || y < 0) {
        //                 block_lim.y() += y_inc * block_size;
        //                 _block_key = block_to_hash_key(block_lim);
        //                 block = map->search(_block_key);
        //                 y = y_inc > 0 ? 0 : lim - 1;
        //             }
        //         } else if (yz_error < 0 && xz_error < 0) {
        //             z += z_inc;
        //             current_p.z() += z_inc * resolution;
        //             xz_error += dx;
        //             yz_error += dy;
        //             if (z >= lim || z < 0) {
        //                 block_lim.z() += z_inc * block_size;
        //                 _block_key = block_to_hash_key(block_lim);
        //                 block = map->search(_block_key);
        //                 z = z_inc > 0 ? 0 : lim - 1;
        //             }
        //         } else if (xy_error == 0) {
        //             x += x_inc;
        //             y += y_inc;
        //             n -= 2;
        //             current_p.x() += x_inc * resolution;
        //             current_p.y() += y_inc * resolution;
        //             if (x >= lim || x < 0) {
        //                 block_lim.x() += x_inc * block_size;
        //                 _block_key = block_to_hash_key(block_lim);
        //                 block = map->search(_block_key);
        //                 x = x_inc > 0 ? 0 : lim - 1;
        //             }
        //             if (y >= lim || y < 0) {
        //                 block_lim.y() += y_inc * block_size;
        //                 _block_key = block_to_hash_key(block_lim);
        //                 block = map->search(_block_key);
        //                 y = y_inc > 0 ? 0 : lim - 1;
        //             }
        //         }
        //         n--;
        //         return valid;
        //     }

        // private:
        //     const SemanticBKIOctoMap *map;   //输入的地图
        //     Block *block;  //一个block块
        //     point3f block_lim;   //当前块的中心
        //     float block_size, resolution;  // 块的大小和地图的分辨率
        //     int dx, dy, dz, error, n;
        //     int x_inc, y_inc, z_inc, xy_error, xz_error, yz_error;
        //     unsigned short index, x, y, z, lim;
        //     BlockHashKey _block_key;  //块的哈希索引
        //     point3f current_p;
        // };

        /// LeafIterator for iterating all leaf nodes in blocks 用于迭代地图中所有叶节点的叶迭代器
        class LeafIterator : public std::iterator<std::forward_iterator_tag, SemanticOcTreeNode> {
        public:
            LeafIterator(const SemanticBKIOctoMap *map) {
                assert(map != nullptr);

                block_it = map->block_arr.cbegin();  // 第一个block
                end_block = map->block_arr.cend();  // 最后一个block

                if (map->block_arr.size() > 0) {   
                    leaf_it = block_it->second->begin_leaf();  // 当前block(第一个block)的叶指针
                    end_leaf = block_it->second->end_leaf();
                } else {
                    leaf_it = SemanticOcTree::LeafIterator();
                    end_leaf = SemanticOcTree::LeafIterator();
                }
            }

            // just for initializing end iterator
            LeafIterator(std::unordered_map<BlockHashKey, Block *>::const_iterator block_it,
                         SemanticOcTree::LeafIterator leaf_it)
                    : block_it(block_it), leaf_it(leaf_it), end_block(block_it), end_leaf(leaf_it) { }

            bool operator==(const LeafIterator &other) {
                return (block_it == other.block_it) && (leaf_it == other.leaf_it);
            }

            bool operator!=(const LeafIterator &other) {
                return !(this->operator==(other));
            }

            LeafIterator operator++(int) {
                LeafIterator result(*this);
                ++(*this);
                return result;
            }

            LeafIterator &operator++() {
                ++leaf_it;
                if (leaf_it == end_leaf) {   //如果到了当前block最后一个叶子，就取下一个block。
                    ++block_it;
                    if (block_it != end_block) {
                        leaf_it = block_it->second->begin_leaf();
                        end_leaf = block_it->second->end_leaf();
                    }
                }
            }

            SemanticOcTreeNode &operator*() const {
                return *leaf_it;
            }

            std::vector<point3f> get_pruned_locs() const {
                std::vector<point3f> pruned_locs;
                point3f center = get_loc();
                float size = get_size();
                float x0 = center.x() - size * 0.5 + Block::resolution * 0.5;
                float y0 = center.y() - size * 0.5 + Block::resolution * 0.5;
                float z0 = center.z() - size * 0.5 + Block::resolution * 0.5;
                float x1 = center.x() + size * 0.5;
                float y1 = center.y() + size * 0.5;
                float z1 = center.z() + size * 0.5;
                for (float x = x0; x < x1; x += Block::resolution) {
                    for (float y = y0; y < y1; y += Block::resolution) {
                        for (float z = z0; z < z1; z += Block::resolution) {
                            pruned_locs.emplace_back(x, y, z);
                        }
                    }
                }
                return pruned_locs;
            }

            inline SemanticOcTreeNode &get_node() const {
                return operator*();
            }

            inline point3f get_loc() const {
                return block_it->second->get_loc(leaf_it);
            }

            inline float get_size() const {
                return block_it->second->get_size(leaf_it);
            }

        private:
            std::unordered_map<BlockHashKey, Block *>::const_iterator block_it;
            std::unordered_map<BlockHashKey, Block *>::const_iterator end_block;

            SemanticOcTree::LeafIterator leaf_it;
            SemanticOcTree::LeafIterator end_leaf;
        };

        /// @return the beginning of leaf iterator
        inline LeafIterator begin_leaf() const { return LeafIterator(this); }

        /// @return the end of leaf iterator
        inline LeafIterator end_leaf() const { return LeafIterator(block_arr.cend(), SemanticOcTree::LeafIterator()); }

        SemanticOcTreeNode search(point3f p) const;

        SemanticOcTreeNode search(float x, float y, float z) const;

        Block *search(BlockHashKey key) const;

        inline float get_block_size() const { return block_size; }

    private:
        /// @return true if point is inside a bounding box given min and max limits.
        inline bool gp_point_in_bbox(const GPPointType &p, const point3f &lim_min, const point3f &lim_max) const {
            return (p.first.x() > lim_min.x() && p.first.x() < lim_max.x() &&
                    p.first.y() > lim_min.y() && p.first.y() < lim_max.y() &&
                    p.first.z() > lim_min.z() && p.first.z() < lim_max.z());
        }

        /// Get the bounding box of a pointcloud. 获取点云的边界框。
        void bbox(const GPPointCloud &cloud, point3f &lim_min, point3f &lim_max) const;

        /// Get all block indices inside a bounding box. 获取边界框内的所有块索引。
        void get_blocks_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                std::vector<BlockHashKey> &blocks) const;

        /// Get all points inside a bounding box assuming pointcloud has been inserted in rtree before.
        //获取边界框中的所有点，假设点云之前已插入rtree。
        int get_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                  GPPointCloud &out);
        int get_hits_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                  GPPointCloud &out);                  
        int get_frees_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                  GPPointCloud &out);

        /// @return true if point exists inside a bounding box assuming pointcloud has been inserted in rtree before.
        // 如果点存在于边界框内，则为true，假设点云之前已插入到rtree中。
        int has_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max);
        int has_hits_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max);
        int has_frees_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max);

        /// Get all points inside a bounding box (block) assuming pointcloud has been inserted in rtree before.
        int get_gp_points_in_bbox(const BlockHashKey &key, GPPointCloud &out);
        int get_hits_gp_points_in_bbox(const BlockHashKey &key, GPPointCloud &out);
        int get_frees_gp_points_in_bbox(const BlockHashKey &key, GPPointCloud &out);

        /// @return true if point exists inside a bounding box (block) assuming pointcloud has been inserted in rtree before.
        int has_gp_points_in_bbox(const BlockHashKey &key);
        int has_hits_gp_points_in_bbox(const BlockHashKey &key);
        int has_frees_gp_points_in_bbox(const BlockHashKey &key);

        /// Get all points inside an extended block assuming pointcloud has been inserted in rtree before.
        int get_gp_points_in_bbox(const ExtendedBlock &block, GPPointCloud &out);

        /// @return true if point exists inside an extended block assuming pointcloud has been inserted in rtree before.
        int has_gp_points_in_bbox(const ExtendedBlock &block);

        /// RTree callback function
        static bool count_callback(GPPointType *p, void *arg);

        /// RTree callback function
        static bool search_callback(GPPointType *p, void *arg);

        /// Downsample PCLPointCloud using PCL VoxelGrid Filtering.
        void downsample(const PCLPointCloud &in, PCLPointCloud &out, float ds_resolution) const;

        /// Sample free training points along sensor beams.  
        void beam_sample(const point3f &hits, const point3f &origin, PointCloud &frees,
                         float free_resolution) const;


        /// Get training data from one sensor scan.
        void get_training_data(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                               float free_resolution, float max_range, GPPointCloud &xy) const;

        /// Get training data from one sensor scan.
        void get_training_data(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                               float free_resolution, float max_range, GPPointCloud &xy, GPPointCloud &xy_hits, GPPointCloud &xy_frees) const;

        float resolution;
        float block_size;
        unsigned short block_depth;
        float ent_thre;  // 分布熵的阈值

        std::unordered_map<BlockHashKey, Block *> block_arr;  //存储block哈希键和block本身的矩阵
        MyRTree rtree;
        MyRTree rtree_hits;
        MyRTree rtree_frees;
    };

}
