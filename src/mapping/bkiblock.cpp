#include <queue>
#include <algorithm>

#include "bkiblock.h"

namespace see_csom {

    // 初始化一个查找表，方法类似与广度优先搜索。每个块的这个都一样，存储的信息包括：Octree哈希键（这里面就包括了现在是第几层第几个的信息）和块中心位置（相对block中心）
    std::unordered_map<OcTreeHashKey, point3f> init_key_loc_map(float resolution, unsigned short max_depth) {
        std::unordered_map<OcTreeHashKey, point3f> key_loc_map;

        std::queue<point3f> center_q;    //一个队列，里面是每个块的中心，先进先出
        center_q.push(point3f(0.0f, 0.0f, 0.0f));   //把原点推进去，第一个是原点

        for (unsigned short depth = 0; depth < max_depth; ++depth) {
            unsigned short q_size = (unsigned short) center_q.size();   //已经有了的数目，是上一个深度下的块的个数
            float half_size = (float) (resolution * pow(2, max_depth - depth - 1) * 0.5f); //在这个深度下，每个块的大小
            for (unsigned short index = 0; index < q_size; ++index) {
                point3f center = center_q.front();
                center_q.pop();
                key_loc_map.emplace(node_to_hash_key(depth, index), center);  //把哈希键（(depth << 16) + index）和小块中心坐标放入无序图中

                if (depth == max_depth - 1)  // 如果这是最后一层，下面没法分了
                    continue;
                for (unsigned short i = 0; i < 8; ++i) {
                    float x = (float) (center.x() + half_size * (i & 4 ? 0.5 : -0.5));
                    float y = (float) (center.y() + half_size * (i & 2 ? 0.5 : -0.5));
                    float z = (float) (center.z() + half_size * (i & 1 ? 0.5 : -0.5));
                    center_q.emplace(x, y, z);   //如果还能分，分成8块存入中心队列，广度优先，继续等着分
                }
            }
        }
        return key_loc_map;
    }

    // 根据key_loc_map，初始化一个另一个查询图
    std::unordered_map<unsigned short, OcTreeHashKey> init_index_map(
            const std::unordered_map<OcTreeHashKey, point3f> &key_loc_map, unsigned short max_depth) {

        std::vector<std::pair<OcTreeHashKey, point3f>> temp;  //这是octree哈希键+坐标的一个容器，把unordered_map中最后一层的最小块放入容器
        for (auto it = key_loc_map.begin(); it != key_loc_map.end(); ++it) {
            unsigned short depth, index;
            hash_key_to_node(it->first, depth, index);   // 得到这个块是block的哪个深度和该深度的第几个
            if (depth == max_depth - 1)   //如果是最后一层，也就是说这一层才是所有的block分的块最小的
                temp.push_back(std::make_pair(it->first, it->second));
        }

        //  先按x升序排序，再按y，最后z，
        std::stable_sort(temp.begin(), temp.end(),
                         [](const std::pair<OcTreeHashKey, point3f> &p1,
                            const std::pair<OcTreeHashKey, point3f> &p2) {
                             return p1.second.x() < p2.second.x();
                         });
        std::stable_sort(temp.begin(), temp.end(),
                         [](const std::pair<OcTreeHashKey, point3f> &p1,
                            const std::pair<OcTreeHashKey, point3f> &p2) {
                             return p1.second.y() < p2.second.y();
                         });
        std::stable_sort(temp.begin(), temp.end(),
                         [](const std::pair<OcTreeHashKey, point3f> &p1,
                            const std::pair<OcTreeHashKey, point3f> &p2) {
                             return p1.second.z() < p2.second.z();
                         });

        std::unordered_map<unsigned short, OcTreeHashKey> index_map;
        int index = 0;
        for (auto it = temp.cbegin(); it != temp.cend(); ++it, ++index) {
            index_map.insert(std::make_pair(index, it->first));
        }

        return index_map;  // 所以这个就是最小块在block中的索引和它的哈希键
    };

    BlockHashKey block_to_hash_key(point3f center) {
        return block_to_hash_key(center.x(), center.y(), center.z());
    }

    // 这个应该是根据block中心得到一个哈希键。和当前在xyz方向的第几个block有关。这里就进行了近似，以block_size为尺度的近似
    // 我删掉了524288.5中的.5
    BlockHashKey block_to_hash_key(float x, float y, float z) {
        return (int64_t(x / (double) Block::size + 524288.5) << 40) |   //2的19次方
               (int64_t(y / (double) Block::size + 524288.5) << 20) |
               (int64_t(z / (double) Block::size + 524288.5));
    }

    // 也可以根据哈希键得到block的中心
    // 我把524288减了0.5
    point3f hash_key_to_block(BlockHashKey key) {
        return point3f(((key >> 40) - 524288) * Block::size,
                       (((key >> 20) & 0xFFFFF) - 524288) * Block::size,
                       ((key & 0xFFFFF) - 524288) * Block::size);
    }

    // 获得周围的block
    ExtendedBlock get_extended_block(BlockHashKey key) {
        ExtendedBlock blocks;
        point3f center = hash_key_to_block(key);  //这是这个block的中心
        float x = center.x();
        float y = center.y();
        float z = center.z();
        blocks[0] = key;  //这是这个块本身的索引键

        float ex, ey, ez;
        for (int i = 0; i < 6; ++i) {
            ex = (i / 2 == 0) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;  
            ey = (i / 2 == 1) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ez = (i / 2 == 2) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            blocks[i + 1] = block_to_hash_key(ex + x, ey + y, ez + z);   // 距离这个块最近的6个块的索引
            //分别为block本身、三个两两对称的block
        }
        return blocks;
    }

    float Block::resolution = 0.1f;
    float Block::size = 0.8f;
    unsigned short Block::cell_num = static_cast<unsigned short>(round(Block::size / Block::resolution));  //在某一维度这个块可以细分为多少块

    std::unordered_map<OcTreeHashKey, point3f> Block::key_loc_map;   // 在一个block中的哈希键
    std::unordered_map<unsigned short, OcTreeHashKey> Block::index_map;   // 一个block最小块的索引和哈希键

    Block::Block() : SemanticOcTree(), center(0.0f, 0.0f, 0.0f) { }

    Block::Block(point3f center) : SemanticOcTree(), center(center) { }

    // 根据当前block中心，得到自己和周围的block的哈希键
    ExtendedBlock Block::get_extended_block() const {
        ExtendedBlock blocks;
        float x = center.x();
        float y = center.y();
        float z = center.z();
        blocks[0] = block_to_hash_key(x, y, z);

        float ex, ey, ez;
        for (int i = 0; i < 6; ++i) {
            ex = (i / 2 == 0) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ey = (i / 2 == 1) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ez = (i / 2 == 2) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            blocks[i + 1] = block_to_hash_key(ex + x, ey + y, ez + z);
        }

        return blocks;
    }

     // 根据当前block中心，得到自己和周围二维扩展的block的哈希键
    ExtendedBlock Block::get_2d_extended_block() const {
        ExtendedBlock blocks;
        float x = center.x();
        float y = center.y();
        float z = center.z();
        blocks[0] = block_to_hash_key(x, y, z);

        float ex, ey, ez;
        for (int i = 0; i < 4; ++i) {
            ex = (i / 2 == 0) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ey = (i / 2 == 1) ? (i % 2 == 0 ? Block::size : -Block::size) : 0;
            ez = 0;
            blocks[i + 1] = block_to_hash_key(ex + x, ey + y, ez + z);
        }

        return blocks;
    }

    // xyz应该是从xyz最小的那一块开始计数，返回最小块的索引
    OcTreeHashKey Block::get_node(unsigned short x, unsigned short y, unsigned short z) const {
        unsigned short index = x + y * Block::cell_num + z * Block::cell_num * Block::cell_num;
        return Block::index_map[index];
    }

    // 这要直到在这个block中所处的xyz，就可以得到绝对坐标
    point3f Block::get_point(unsigned short x, unsigned short y, unsigned short z) const {
        return Block::key_loc_map[get_node(x, y, z)] + center;
    }

    // 根据点的绝对坐标，得到它的xyz形式的索引
    void Block::get_index(const point3f &p, unsigned short &x, unsigned short &y, unsigned short &z) const {
        int xx = static_cast<int>((p.x() - center.x()) / resolution + Block::cell_num / 2);
        int yy = static_cast<int>((p.y() - center.y()) / resolution + Block::cell_num / 2);
        int zz = static_cast<int>((p.z() - center.z()) / resolution + Block::cell_num / 2);
        auto clip = [](int a) -> int { return std::max(0, std::min(a, Block::cell_num - 1)); }; // 最小为0，最大为cell_num-1
        x = static_cast<unsigned short>(clip(xx));
        y = static_cast<unsigned short>(clip(yy));
        z = static_cast<unsigned short>(clip(zz));
    }

    // 根据绝对坐标，得到节点
    SemanticOcTreeNode& Block::search(float x, float y, float z) const {
        return search(point3f(x, y, z));
    }

    // 根据节点位置返回节点
    SemanticOcTreeNode& Block::search(point3f p) const {
        unsigned short x, y, z;
        get_index(p, x, y, z);
        return operator[](get_node(x, y, z));
    }
}
