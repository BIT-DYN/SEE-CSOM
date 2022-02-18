#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include<fstream>
#include <iostream>
#include <string>   
#include <numeric>
#include <iomanip>
#include <ros/ros.h>

#include "bkioctomap.h"
#include "bki.h"

using std::vector;
using namespace std;


// #define DEBUG true;
// #define OPENMP true;

#ifdef DEBUG

#include <iostream>

#define Debug_Msg(msg) {\
std::cout << "Debug: " << msg << std::endl; }
#endif

namespace see_csom {

    SemanticBKIOctoMap::SemanticBKIOctoMap() : SemanticBKIOctoMap(0.1f, // resolution
                                        4, // block_depth
                                        3,  // num_class
                                        1.0, // sf2
                                        1.0, // ell
                                        1.0f, // prior
                                        1.0f, // var_thresh
                                        0.3f, // free_thresh
                                        0.7f // occupied_thresh
                                    ) { }

    SemanticBKIOctoMap::SemanticBKIOctoMap(float resolution,
                        unsigned short block_depth,
                        int num_class,
                        float sf2,
                        float ell,
                        float prior,
                        float var_thresh,
                        float free_thresh,
                        float occupied_thresh)
            : resolution(resolution), block_depth(block_depth),
              block_size((float) pow(2, block_depth - 1) * resolution) {
        ent_thre = 1;
        Block::resolution = resolution;
        Block::size = this->block_size;   //size是每个块的大小
        Block::key_loc_map = init_key_loc_map(resolution, block_depth);
        Block::index_map = init_index_map(Block::key_loc_map, block_depth);
        
        // Bug fixed
        Block::cell_num = static_cast<unsigned short>(round(Block::size / Block::resolution));
        std::cout << "block::resolution: " << Block::resolution << std::endl;
        std::cout << "block::size: " << Block::size << std::endl;
        std::cout << "block::cell_num: " << Block::cell_num << std::endl;
        
        SemanticOcTree::max_depth = block_depth;

        SemanticOcTreeNode::num_class = num_class;
        SemanticOcTreeNode::sf2 = sf2;
        SemanticOcTreeNode::ell = ell;
        SemanticOcTreeNode::prior = prior;
        SemanticOcTreeNode::var_thresh = var_thresh;
        SemanticOcTreeNode::free_thresh = free_thresh;
        SemanticOcTreeNode::occupied_thresh = occupied_thresh;
    }

    SemanticBKIOctoMap::~SemanticBKIOctoMap() {
        for (auto it = block_arr.begin(); it != block_arr.end(); ++it) {
            if (it->second != nullptr) {
                delete it->second;
            }
        }
    }

    void SemanticBKIOctoMap::set_resolution(float resolution) {
        this->resolution = resolution;
        Block::resolution = resolution;  //分辨率
        this->block_size = (float) pow(2, block_depth - 1) * resolution;  // block的尺寸
        Block::size = this->block_size;
        Block::key_loc_map = init_key_loc_map(resolution, block_depth);  //初始化block的块索引列表
    }

    void SemanticBKIOctoMap::set_block_depth(unsigned short max_depth) {
        this->block_depth = max_depth;
        SemanticOcTree::max_depth = max_depth;
        this->block_size = (float) pow(2, block_depth - 1) * resolution;
        Block::size = this->block_size;
        Block::key_loc_map = init_key_loc_map(resolution, block_depth);
    }

   

    // 插入局部地图点云的函数，单对单（自己写的-----------------------------------------------------------------------------------------------------------------------------------）
    void SemanticBKIOctoMap::insert_map_point_single(pcl::PointCloud<MapPoint>::Ptr transformed_cells)
    {
        // 记录地图2、地图1、和全局地图的语义标签和概率
        std::ofstream OutFile("/home/dengyinnan/catkin_ws/src/BKISemanticMapping/multi_result/single.txt"); 
        std::vector<BlockHashKey>  new_block;
        int if_free_node;
        std::vector<float>  old_prob;
        std::vector<float>  old_prob_2;
        std::vector<float>  new_prob;
        // std::cout << "reserive" << std::endl;
        // int i = 0;
        // int num = transformed_cells->size();
        // std::cout << "num" << num << std::endl;
        for (pcl::PointCloud<MapPoint>::const_iterator it = transformed_cells->begin(); it != transformed_cells->end(); ++it)
        {
            BlockHashKey key = block_to_hash_key(it->x, it->y, it->z); // 这就是这个点所在的block的key
#ifdef OPENMP
#pragma omp critical
#endif
            {
                if (block_arr.find(key) == block_arr.end())   //如果这个点所在block没在block_arr（应该是用来存储所有block的了）中
                {
                    block_arr.emplace(key, new Block(hash_key_to_block(key)));  //就新建一个
                    new_block.push_back(key);
                }
            };
            Block *block = block_arr[key]; // 这就是这个点所在的block了
            SemanticOcTreeNode &node = block->search(it->x, it->y, it->z);  

            // if (std::find(new_block.begin(), new_block.end(), key) == new_block.end())  //如果这个block本来就有（不是新增的）
            if ( std::count(new_block.begin(), new_block.end(), key) == 0 )
             {
                std::vector<float> probs(SemanticOcTreeNode::num_class);
                node.get_probs(probs);
                int semantic = node.get_semantics();
                if (semantic >= SemanticOcTreeNode::num_class || semantic < 0)
                    semantic = 0;
                // std::cout << "semantic: " <<semantic <<std::endl;
                if (probs[semantic] <= 0.05)   // 理论上是0.05，即不是空的就用来比较，但是因为机器人1看到的共同区域距离较远，不确定性更高，所以筛选严格一点，如果换了数据集不一定一样
                    if_free_node = 1;
                else
                    if_free_node = 0;
                if ( if_free_node == 0 ) // 这个节点也是非空的时候才比较
                {
                    old_prob.push_back(probs[semantic]);
                    OutFile<<std::setw(2)<<semantic<<"    "<<std::fixed <<std::setprecision(4)<<probs[semantic]<<"    ";
                }
            }

            std::vector<float> ybars(SemanticOcTreeNode::num_class, 0);
            ybars[it->semantic_label1] = it->prob1;
            ybars[it->semantic_label2] = it->prob2;
            ybars[it->semantic_label3] = it->prob3;
            node.update(ybars); 

            // if (std::find(new_block.begin(), new_block.end(), key) == new_block.end())
            if ( std::count(new_block.begin(), new_block.end(), key) == 0 &&  if_free_node == 0 )
            {
                old_prob_2.push_back(it->true_prob);
                OutFile<<std::setw(2)<<it->semantic_label1<<"    "<<std::fixed <<std::setprecision(4)<<it->true_prob<<"    ";
                std::vector<float> probs(SemanticOcTreeNode::num_class);
                node.get_probs(probs);
                int semantic = node.get_semantics();
                if (semantic >= SemanticOcTreeNode::num_class || semantic < 0)
                    semantic = 0;
                // std::cout << "semantic: " <<semantic <<std::endl;
                new_prob.push_back(probs[semantic]);
                OutFile<<std::setw(2)<<semantic<<"    "<<std::fixed <<std::setprecision(4)<<probs[semantic]<<"\n";
            }

            // std::cout << "i: " << i <<std::endl;
            // i++;
        }
        OutFile.close();
        int size = old_prob.size();
        // std::cout << "size: " << size <<std::endl;
        double sum_old = accumulate(old_prob.begin(), old_prob.end(), 0.0) ;
        double sum_old_2 = accumulate(old_prob_2.begin(), old_prob_2.end(), 0.0) ;
        // std::cout << "sum_old: " << sum_old <<std::endl;
        double sum_new = accumulate(new_prob.begin(), new_prob.end(), 0.0) ;
        // std::cout << "sum_new: " << sum_new <<std::endl;
        float mean_old = sum_old  / size;   // 新的平均概率
        float mean_old_2 = sum_old_2  / size;   // 新的平均概率
        float mean_new = sum_new / size;   // 新的平均概率
        std::cout << "map1: " <<std::fixed <<std::setprecision(4)<< mean_old << std::endl;
        std::cout << "map2: " <<std::fixed <<std::setprecision(4)<< mean_old_2 << std::endl;
        std::cout << "global map: " <<std::fixed <<std::setprecision(4)<< mean_new << std::endl;
    }


    // 插入局部地图点云的函数，多对多（自己写的-----------------------------------------------------------------------------------------------------------------------------------）
    // 把共同的block和node位置记录下来，对每个block连续建图，查看之后共同node的熵值
    void SemanticBKIOctoMap::insert_map_point_multi(pcl::PointCloud<MapPoint>::Ptr transformed_cells)
    {
        // 记录地图2、地图1、全局地图、连续化之后全局地图的语义标签和概率
        std::ofstream OutFile("/home/dengyinnan/catkin_ws/src/BKISemanticMapping/multi_result/multi.txt"); 
        std::vector<BlockHashKey>  new_block;
        int if_free_node; //判断是否为新节点，1是新的，0都有，即0的时候就是common_node
        std::vector<BlockHashKey>  common_block; // 两个地图共有的block，用于之后的连续化建图
        std::vector<see_csom::point3f>  common_node; // 两个地图共有的block，用于之后的连续化建图
        std::vector<int>  old_semantic;
        std::vector<float>  old_prob;
        std::vector<int>  old_semantic_2;
        std::vector<float>  old_prob_2;
        std::vector<int>  new_semantic;
        std::vector<float>  new_prob;
        std::vector<int>  new_semantic_after;
        std::vector<float>  new_prob_after;
        std::unordered_map<BlockHashKey, SemanticBKI3f *> bgk_arr;
        for (pcl::PointCloud<MapPoint>::const_iterator it = transformed_cells->begin(); it != transformed_cells->end(); ++it)
        {
            BlockHashKey key = block_to_hash_key(it->x, it->y, it->z); // 这就是这个点所在的block的key
#ifdef OPENMP
#pragma omp critical
#endif
            {
                if (block_arr.find(key) == block_arr.end())   //如果这个点所在block没在block_arr（应该是用来存储所有block的了）中
                {
                    block_arr.emplace(key, new Block(hash_key_to_block(key)));  //就新建一个
                    new_block.push_back(key);             
                }
                else
                {
                    Block *block = block_arr[key];
                    common_block.push_back(key);
                    ExtendedBlock eblock = block->get_extended_block();
                    for (auto block_it = eblock.cbegin(); block_it != eblock.cend(); ++block_it) 
                    {
                        if (bgk_arr.find(*block_it) == bgk_arr.end())  //如果需要测试的block的扩展block没有bgk模型，就加进来一个
                        {
                            vector<float> block_x, block_y;
                            Block *block = block_arr[key];
                            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) //这个扩展block中的每个叶节点
                            {   
                                point3f p = block->get_loc(leaf_it);     //这个扩展block中这个叶节点的坐标                                   
                                std::vector<float> scs(SemanticOcTreeNode::num_class);
                                leaf_it.get_node().get_ms(scs);
                                // 每个节点出的点个数和语义
                                for(int i = 0; i < SemanticOcTreeNode::num_class; i++)
                                {
                                    for(float j = 1; j <= scs[i]; j = j + 1.0)
                                    {
                                        block_x.push_back(p.x());
                                        block_x.push_back(p.y());
                                        block_x.push_back(p.z());
                                        block_y.push_back((float)i);
                                    }
                                }
                            }
                            SemanticBKI3f *bgk = new SemanticBKI3f(SemanticOcTreeNode::num_class, SemanticOcTreeNode::sf2*10, SemanticOcTreeNode::ell); // 影响大一点，因为没有距离太近的点云
                            bgk->train(block_x, block_y);
                            bgk_arr.emplace(key, bgk);
                        }
                    }
                }
            };
            Block *block = block_arr[key]; // 这就是这个点所在的block了
            SemanticOcTreeNode &node = block->search(it->x, it->y, it->z);  

            // if (std::find(new_block.begin(), new_block.end(), key) == new_block.end())  //如果这个block本来就有（不是新增的）
            if ( std::count(new_block.begin(), new_block.end(), key) == 0 )
             {
                std::vector<float> probs(SemanticOcTreeNode::num_class);
                node.get_probs(probs);
                int semantic = node.get_semantics();
                if (semantic >= SemanticOcTreeNode::num_class || semantic < 0)
                    semantic = 0;
                // std::cout << "semantic: " <<semantic <<std::endl;
                if (probs[semantic] <= 0.05)   // 理论上是0.05，即不是空的就用来比较，但是因为机器人1看到的共同区域距离较远，不确定性更高，所以筛选严格一点，如果换了数据集不一定一样
                    if_free_node = 1;
                else
                    if_free_node = 0;
                if ( if_free_node == 0 ) // 这个节点也是非空的时候才比较
                {
                    old_semantic.push_back(semantic);
                    old_prob.push_back(probs[semantic]);
                    see_csom::point3f node_center;
                    node_center.x() = it->x;
                    node_center.y() = it->y;
                    node_center.z() = it->z;
                    common_node.push_back(node_center);
                }
            }

            std::vector<float> ybars(SemanticOcTreeNode::num_class, 0);
            ybars[it->semantic_label1] = it->prob1;
            ybars[it->semantic_label2] = it->prob2;
            ybars[it->semantic_label3] = it->prob3;
            node.update(ybars); 

            // if (std::find(new_block.begin(), new_block.end(), key) == new_block.end())
            if ( std::count(new_block.begin(), new_block.end(), key) == 0 &&  if_free_node == 0 )
            {
                old_semantic_2.push_back(it->semantic_label1);
                old_prob_2.push_back(it->true_prob);
                std::vector<float> probs(SemanticOcTreeNode::num_class);
                node.get_probs(probs);
                int semantic = node.get_semantics();
                if (semantic >= SemanticOcTreeNode::num_class || semantic < 0)
                    semantic = 0;
                // std::cout << "semantic: " <<semantic <<std::endl;
                new_semantic.push_back(semantic);
                new_prob.push_back(probs[semantic]);
            }
        }
        for (auto block_key = common_block.begin(); block_key != common_block.end(); block_key++)
        {
            // 公共block的所有结点位置
            vector<float> xs;
            Block *block = block_arr[*block_key];
            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                point3f p = block->get_loc(leaf_it);
                xs.push_back(p.x());
                xs.push_back(p.y());
                xs.push_back(p.z());
            }
            ExtendedBlock eblock = block->get_extended_block();
            for (auto block_it = eblock.cbegin(); block_it != eblock.cend(); ++block_it) 
            {
                auto bgk = bgk_arr.find(*block_it);
                if (bgk == bgk_arr.end())
                    continue;
               	vector<vector<float>> ybars;
                bgk->second->predict(xs, ybars);
                int j = 0;
                for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) 
                {
                    SemanticOcTreeNode &node = leaf_it.get_node();
                    node.update(ybars[j]);
               }
            }
        }

        int size = common_node.size();
        for (int  i = 0; i < size; i++)
        {
            see_csom::point3f p = common_node[i];
            SemanticOcTreeNode node = search(p);
            int semantic = node.get_semantics();
            new_semantic_after.push_back( semantic );
            std::vector<float> probs(SemanticOcTreeNode::num_class);
            node.get_probs(probs);
            new_prob_after.push_back( probs[semantic]);
        }

        for (int i = 0; i < size; i++)
        {
            OutFile<<std::setw(2)<<old_semantic[i]<<"    "<<std::fixed <<std::setprecision(4)<<old_prob[i]
                            <<"    "<<std::setw(2)<<old_semantic_2[i]<<"    "<<std::fixed <<std::setprecision(4)<<old_prob_2[i]
                            <<"    "<<std::setw(2)<<new_semantic[i]<<"    "<<std::fixed <<std::setprecision(4)<<new_prob[i]
                            <<"    "<<std::setw(2)<<new_semantic_after[i]<<"    "<<std::fixed <<std::setprecision(4)<<new_prob_after[i]<<"\n";
        }
        OutFile.close();
        double sum_old = accumulate(old_prob.begin(), old_prob.end(), 0.0) ;
        double sum_old_2 = accumulate(old_prob_2.begin(), old_prob_2.end(), 0.0) ;
        double sum_new = accumulate(new_prob.begin(), new_prob.end(), 0.0) ;
        double sum_new_after = accumulate(new_prob_after.begin(), new_prob_after.end(), 0.0) ;
        float mean_old = sum_old  / size;   // 新的平均概率
        float mean_old_2 = sum_old_2  / size;   // 新的平均概率
        float mean_new = sum_new / size;   // 新的平均概率
        float mean_new_after = sum_new_after / size;   // 新的平均概率
        std::cout << "map1: " <<std::fixed <<std::setprecision(4)<< mean_old << std::endl;
        std::cout << "map2: " <<std::fixed <<std::setprecision(4)<< mean_old_2 << std::endl;
        std::cout << "global map: " <<std::fixed <<std::setprecision(4)<< mean_new << std::endl;
        std::cout << "global map after bki: " <<std::fixed <<std::setprecision(4)<< mean_new_after << std::endl;
    }



    int SemanticBKIOctoMap::block_is_unknow(const BlockHashKey &key) {
        Block *block = block_arr[key];  // 当前这个block
        for (auto it = block->begin_leaf(); it != block->end_leaf(); it++)
        {
            if( it.get_node().get_state() != see_csom::State::UNKNOWN )
                return 0;
        }
        return 1;
    }

    // 用于stanford数据集打印每个节点的预测标签
    void SemanticBKIOctoMap::print_vaule(void){
        std::ofstream OutFile("/home/dyn/catkin_ws/src/BKISemanticMapping/stanford_result/lounge/csm.txt"); 
        int i = 0;
        // 为了准确使用int，然后除以100
        // int dis = 25;    //dyn和bki的时候使用
        int dis = 0;   //csm的时候使用
        for(int x_int = 2000-dis; x_int < 12000-dis; x_int = x_int + resolution*1000)
        {
            for(int y_int = -9000-dis; y_int < -1000-dis; y_int = y_int + resolution*1000)
            {
                for(int z_int = -500-dis; z_int < 3500-dis; z_int = z_int + resolution*1000)
                {
                    i++;
                    float x, y, z;
                    x = x_int/1000.0;
                    y = y_int/1000.0;
                    z = z_int/1000.0;
                    BlockHashKey key = block_to_hash_key(x, y, z); // 这就是这个点所在的block的key
#ifdef OPENMP
#pragma omp critical
#endif
                    {
                        if (block_arr.find(key) == block_arr.end())   //如果这个点所在block没在block_arr（应该是用来存储所有block的了）中
                            block_arr.emplace(key, new Block(hash_key_to_block(key)));  //就新建一个
                    };
                    Block *block = block_arr[key]; // 这就是这个点所在的block了
                    SemanticOcTreeNode &node = block->search(x, y, z);     //这个结点
                    if (node.get_state() == see_csom::State::OCCUPIED)
                    {
                        int semantics_num = node.get_semantics();
                        std::string semantics = std::to_string(semantics_num);
                        OutFile << semantics << "\n"; 
                    }
                    else
                    {
                        OutFile << "0" << "\n"; 
                    }
                }
            }
        }
        OutFile.close();
        std::cout << i <<std::endl;
    }


    // 用于SemanticKITTI数据集打印每个节点的预测标签
    void SemanticBKIOctoMap::print_vaule_2(void){
        std::ofstream OutFile("/home/dyn/catkin_ws/src/BKISemanticMapping/semantickitti_result/04/csm.txt"); 
        // std::ofstream OutFile2("/home/dengyinnan/catkin_ws/src/BKISemanticMapping/semantickitti_result/bki_pro.txt"); 
        // std::fstream InFile("/home/dengyinnan/catkin_ws/src/BKISemanticMapping/semantickitti_result/truth.txt"); 
        int i = 0;
        // 为了准确使用int，然后除以10
        // int dis = 15;    //dyn和bki和truth的时候使用
        int dis = 0;   //csm的时候使用
        for(int x_int = -5100-dis; x_int < 5100-dis; x_int = x_int + resolution*100)
        {
            for(int y_int = -5100-dis; y_int < 5700-dis; y_int = y_int + resolution*100)
            {
                for(int z_int = -300-dis; z_int < 390-dis; z_int = z_int + resolution*100)
                {
        // for(int x_int = -4800-dis; x_int < 4800-dis; x_int = x_int + resolution*100)
        // {
        //     for(int y_int = -4800-dis; y_int < 6000-dis; y_int = y_int + resolution*100)
        //     {
        //         for(int z_int = -300-dis; z_int < 360-dis; z_int = z_int + resolution*100)
        //         {
                    i++;
                    // 打印概率用的，现在不用
                    // int truth_num;
                    // string s;
                    // getline(InFile, s);
                    // if (!s.empty()) 
                    // {
                    //     stringstream ss;
                    //     ss << s;
                    //     ss >> truth_num; 
                    // }
                    float x, y, z;
                    x = x_int/100.0;
                    y = y_int/100.0;
                    z = z_int/100.0;
                    BlockHashKey key = block_to_hash_key(x, y, z); // 这就是这个点所在的block的key
#ifdef OPENMP
#pragma omp critical
#endif
                    {
                        if (block_arr.find(key) == block_arr.end())   //如果这个点所在block没在block_arr（应该是用来存储所有block的了）中
                            block_arr.emplace(key, new Block(hash_key_to_block(key)));  //就新建一个
                    };
                    Block *block = block_arr[key]; // 这就是这个点所在的block了
                    SemanticOcTreeNode &node = block->search(x, y, z);     //这个结点
                    if (node.get_state() == see_csom::State::OCCUPIED)
                    {
                        int semantics_num = node.get_semantics();
                        OutFile << semantics_num << "\n"; 
                    }
                    else
                    {
                        OutFile << "0" << "\n"; 
                    }
                    // std::vector<float> probs(SemanticOcTreeNode::num_class);
                    // node.get_probs(probs);  
                    // float pro = probs[truth_num];
                    // OutFile2 << pro <<"\n";
                }
            }
        }
        OutFile.close();
        // OutFile2.close();
        std::cout << i <<std::endl;
    }



// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// -----------------------------------------------------------------------------计数传感器模型-----------------------------------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

    void SemanticBKIOctoMap::insert_pointcloud_csm(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                                      float free_res, float max_range) {

#ifdef DEBUG
        Debug_Msg("Insert pointcloud: " << "cloud size: " << cloud.size() << " origin: " << origin);
#endif

        ////////// Preparation //////////////////////////
        /////////////////////////////////////////////////
        GPPointCloud xy;
        get_training_data(cloud, origin, ds_resolution, free_res, max_range, xy);
#ifdef DEBUG
        Debug_Msg("Training data size: " << xy.size());
#endif
        // If pointcloud after max_range filtering is empty
        //  no need to do anything
        if (xy.size() == 0) {
            return;
        }

        point3f lim_min, lim_max;
        bbox(xy, lim_min, lim_max);

        vector<BlockHashKey> blocks;
        get_blocks_in_bbox(lim_min, lim_max, blocks);

        for (auto it = xy.cbegin(); it != xy.cend(); ++it) {
            float p[] = {it->first.x(), it->first.y(), it->first.z()};
            rtree.Insert(p, p, const_cast<GPPointType *>(&*it));
        }
        /////////////////////////////////////////////////

        ////////// Training /////////////////////////////
        /////////////////////////////////////////////////
        vector<BlockHashKey> test_blocks;
        std::unordered_map<BlockHashKey, SemanticBKI3f *> bgk_arr;
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < blocks.size(); ++i) {
            BlockHashKey key = blocks[i];
            ExtendedBlock eblock = get_extended_block(key);
            if (has_gp_points_in_bbox(eblock))
#ifdef OPENMP
#pragma omp critical
#endif
            {
                test_blocks.push_back(key);
            };

            GPPointCloud block_xy;
            get_gp_points_in_bbox(key, block_xy);
            if (block_xy.size() < 1)
                continue;

            vector<float> block_x, block_y;
            for (auto it = block_xy.cbegin(); it != block_xy.cend(); ++it) {
                block_x.push_back(it->first.x());
                block_x.push_back(it->first.y());
                block_x.push_back(it->first.z());
                block_y.push_back(it->second);
            
            
            //std::cout << search(it->first.x(), it->first.y(), it->first.z()) << std::endl;
            }

            SemanticBKI3f *bgk = new SemanticBKI3f(SemanticOcTreeNode::num_class, SemanticOcTreeNode::sf2, SemanticOcTreeNode::ell);
            bgk->train(block_x, block_y);
#ifdef OPENMP
#pragma omp critical
#endif
            {
                bgk_arr.emplace(key, bgk);
            };
        }
#ifdef DEBUG
        Debug_Msg("Training done");
        Debug_Msg("Prediction: block number: " << test_blocks.size());
#endif
        /////////////////////////////////////////////////

        ////////// Prediction ///////////////////////////
        /////////////////////////////////////////////////
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < test_blocks.size(); ++i) {
            BlockHashKey key = test_blocks[i];
#ifdef OPENMP
#pragma omp critical
#endif
            {
                if (block_arr.find(key) == block_arr.end())
                    block_arr.emplace(key, new Block(hash_key_to_block(key)));
            };
            Block *block = block_arr[key];
            vector<float> xs;
            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                point3f p = block->get_loc(leaf_it);
                xs.push_back(p.x());
                xs.push_back(p.y());
                xs.push_back(p.z());
            }
            //std::cout << "xs size: "<<xs.size() << std::endl;

	          // For counting sensor model
            auto bgk = bgk_arr.find(key);
            if (bgk == bgk_arr.end())
              continue;

            vector<vector<float>> ybars;
            bgk->second->predict_csm(xs, ybars);

            int j = 0;
            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) {
                SemanticOcTreeNode &node = leaf_it.get_node();

                // Only need to update if kernel density total kernel density est > 0
                node.update(ybars[j]);
            }

        }
#ifdef DEBUG
        Debug_Msg("Prediction done");
#endif

        ////////// Cleaning /////////////////////////////
        /////////////////////////////////////////////////
        for (auto it = bgk_arr.begin(); it != bgk_arr.end(); ++it)
            delete it->second;

        rtree.RemoveAll();
    }
    
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// ---------------------------------------------------------------------------------------分界线之DYN模型------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

    void SemanticBKIOctoMap::insert_pointcloud(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                                      float free_res, float max_range) {

#ifdef DEBUG
        Debug_Msg("Insert pointcloud: " << "cloud size: " << cloud.size() << " origin: " << origin);
#endif

        ////////// Preparation //////////////////////////
        /////////////////////////////////////////////////
        GPPointCloud xy, xy_hits, xy_frees;   // 先得到GP点云，里面包括hit_points和free_points的坐标以及标签（0.0则为空，其他都是占用，表示语义）
        bool if_use_free;

        get_training_data(cloud, origin, ds_resolution, free_res, max_range, xy, xy_hits, xy_frees);

        // std::cout << "hits_num:  " << xy_hits.size() <<std::endl;   
        // std::cout << "frees_num:  " << xy_frees.size() <<std::endl;   
        if (xy.size() == 0) {
            return;
        }
        if (xy_frees.size() > 10)
        {
            if_use_free = 1;
        }
        else
        {
            if_use_free = 0;
        }
        

        point3f lim_min, lim_max;
        bbox(xy, lim_min, lim_max);  //得到xy（训练数据）的边界（最小位置和最大位置）
        // std::cout<<"max and min: "<< lim_max <<lim_min<<std::endl;

        vector<BlockHashKey> blocks;
        get_blocks_in_bbox(lim_min, lim_max, blocks);   // 在这个边界中分块，得到所有block的索引blocks，怎么保证这个block不会和之前的block重叠呢？
        // std::cout<<"num of blocks  : "<<blocks.size()<<std::endl;

        for (auto it = xy_hits.cbegin(); it != xy_hits.cend(); ++it) {
            float p[] = {it->first.x(), it->first.y(), it->first.z()};
            rtree_hits.Insert(p, p, const_cast<GPPointType *>(&*it));   // 把hit点的绝对坐标和GP点存入R树中
        }

        if (if_use_free)
        {
            for (auto it = xy_frees.cbegin(); it != xy_frees.cend(); ++it) {
                float p[] = {it->first.x(), it->first.y(), it->first.z()};
                rtree_frees.Insert(p, p, const_cast<GPPointType *>(&*it));   // 把free点的绝对坐标和GP点存入R树中
            }
        }

        /////////////////////////////////////////////////
        ////////// Training /////////////////////////////
        /////////////////////////////////////////////////
        // 下面开始对这一部分点云进行训练,分为hit和free两部分
        vector<BlockHashKey> test_blocks_hits;  // 需要训练hit的block的容器（扩展块中有hitgp点）
        vector<BlockHashKey> test_blocks_frees;  // 需要训练free的block的容器（扩展块中有freegp点)，可以上上面这个有所重复
        std::unordered_map<BlockHashKey, SemanticBKI3f *> bgk_arr_hits;  //  hit贝叶斯高斯核模型的矩阵
        std::unordered_map<BlockHashKey, SemanticBKI3f *> bgk_arr_frees;  //  free贝叶斯高斯核模型的矩阵

// #ifdef OPENMP
// #pragma omp parallel for schedule(dynamic)
// #endif
        for (int i = 0; i < blocks.size(); ++i) {
            BlockHashKey key = blocks[i];
            ExtendedBlock eblock = get_extended_block(key);   //eblock是当前block的扩展block的哈希键列表.

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// ---------------------------------------------------我自己修改的解决膨胀问题的方法--------------------------------------------------------------------//
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//             float dis_ent = 0.0; //分布熵
//             point3f cur_block_center, ex_block_center, dis;
//             std::vector<point3f>  dis_vector;
//             cur_block_center = hash_key_to_block(key);  // 得到当前block的绝对坐标，正常的
//             // std::cout << cur_block_center <<std::endl;
//             for (auto it = eblock.cbegin(); it != eblock.cend(); ++it) {
//                 if (has_hits_gp_points_in_bbox(*it) > 0)
//                     if (it == eblock.cbegin()){
//                         dis_ent += 999;
//                         break;
//                     }
//                     else{
//                         ex_block_center = hash_key_to_block(*it);
//                         dis =  ( ex_block_center - cur_block_center ) / block_size ;
//                         dis_vector.push_back(dis);
//                     }
//             }
//             float x_sum =0.0;
//             float y_sum =0.0;
//             float z_sum =0.0;
//             float x_variance =0.0;
//             float y_variance =0.0;
//             float z_variance =0.0;
//             if (dis_ent < 999){
//                 for (auto it = dis_vector.cbegin(); it != dis_vector.cend(); ++it){
//                     x_sum += it->x();
//                     y_sum += it->y();
//                     z_sum += it->z();
//                 }
//                 float x_mean = x_sum / dis_vector.size();
//                 float y_mean = y_sum / dis_vector.size();
//                 float z_mean = z_sum / dis_vector.size();
//                 for (auto it = dis_vector.cbegin(); it != dis_vector.cend(); ++it){
//                     x_variance += (it->x() - x_mean) * (it->x() - x_mean);
//                     y_variance += (it->y() - y_mean) * (it->y() - y_mean);
//                     z_variance += (it->z() - z_mean) * (it->z() - z_mean);
//                 }
//                 dis_ent += x_variance + y_variance + z_variance;
//             }
// #ifdef OPENMP
// #pragma omp critical
// #endif
//             if (dis_ent>=ent_thre){   //如果当前block的hit分布熵超过熵阈值
//                 test_blocks_hits.push_back(key);  
//                 // std::cout<<"dis_ent:    " << dis_ent<<std::endl;
//             }

// ---------------------------------------------------------------            自己设计的方法简单来写                       --------------------------------------------------------------------//
            int num = 0;
            for (auto it = eblock.cbegin(); it != eblock.cend(); ++it) {
                if (has_hits_gp_points_in_bbox(*it) > 0)
                    if (it == eblock.cbegin()){
                        num = 2;
                        break;
                    }
                    else{
                        num++;
                    }
            } 
#ifdef OPENMP
#pragma omp critical
#endif
            if (num >= 2){   //如果当前block的hit分布熵超过熵阈值
                test_blocks_hits.push_back(key);  
            }
            
//----------------------------------------------------------------------------------------------------------------------------------------------------//
//------------------------------------------------------------分割线-------------------------------------------------------------------------------//
//----------------------------------------------------------------------------------------------------------------------------------------------------//
            else if (if_use_free)
            {
                if (has_frees_gp_points_in_bbox(key)>0)   // 只要这个block中有free点就需要free训练
                    test_blocks_frees.push_back(key);  
            }


            // 每个块的hit bgk和free bgk都是分开的
            GPPointCloud block_xy_hits;  
            get_hits_gp_points_in_bbox(key, block_xy_hits); // 得到这个block中的hits gp点
            if (block_xy_hits.size() >= 1)  
            {
                vector<float> block_x, block_y;  // block_x依次存储这个block中点绝对xyz坐标；block_y存储点类别
                for (auto it = block_xy_hits.cbegin(); it != block_xy_hits.cend(); ++it) {
                    block_x.push_back(it->first.x());
                    block_x.push_back(it->first.y());
                    block_x.push_back(it->first.z());
                    block_y.push_back(it->second); 
                }
                SemanticBKI3f *bgk = new SemanticBKI3f(SemanticOcTreeNode::num_class, SemanticOcTreeNode::sf2, SemanticOcTreeNode::ell);  // 初始化一个贝叶斯和推理模型
                bgk->train(block_x, block_y);
#ifdef OPENMP
#pragma omp critical
#endif
                {
                    bgk_arr_hits.emplace(key, bgk);   // 把block的哈希键和训练模型存入
                };
            }

            if (if_use_free)
            {
                GPPointCloud block_xy_frees;  
                get_frees_gp_points_in_bbox(key, block_xy_frees); // 得到这个block中的free gp点
                if (block_xy_frees.size() < 1)
                    continue;
                vector<float> block_x_2, block_y_2;  // block_x依次存储这个block中点绝对xyz坐标；block_y存储点类别
                for (auto it = block_xy_frees.cbegin(); it != block_xy_frees.cend(); ++it) {
                    block_x_2.push_back(it->first.x());
                    block_x_2.push_back(it->first.y());
                    block_x_2.push_back(it->first.z());
                    block_y_2.push_back(it->second); 
                }
                SemanticBKI3f *bgk_2 = new SemanticBKI3f(SemanticOcTreeNode::num_class, SemanticOcTreeNode::sf2, resolution/2);  // 初始化一个贝叶斯和推理模型,free点的影响范围比较小
                bgk_2->train(block_x_2, block_y_2);

    #ifdef OPENMP
    #pragma omp critical
    #endif
                {
                    bgk_arr_frees.emplace(key, bgk_2);   // 把block的哈希键和训练模型存入
                };
            }
        }

        // std::cout << "hit block num: " << test_blocks_hits.size() <<std::endl;
        // std::cout << "free block num: " << test_blocks_frees.size() <<std::endl;
        // std::cout << "hit bgk num: " << bgk_arr_hits.size() <<std::endl;
        // std::cout << "free bak num: " << bgk_arr_frees.size() <<std::endl;

        /////////////////////////////////////////////////

        ////////// Prediction ///////////////////////////
        /////////////////////////////////////////////////
        // 预测数据
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < test_blocks_hits.size(); ++i) {   //对于值得预测的hits block依次进行预测
            BlockHashKey key = test_blocks_hits[i];
#ifdef OPENMP
#pragma omp critical
#endif
            {
                if (block_arr.find(key) == block_arr.end())   //如果当前测试块在block_arr（应该是用来存储所有block的了）中没有找到
                    block_arr.emplace(key, new Block(hash_key_to_block(key)));  //就新建一个
            };
            Block *block = block_arr[key];
            vector<float> xs;  //这里面是test_blocks当前这个block的所有的的节点（子块）的xyz
            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                point3f p = block->get_loc(leaf_it);
                xs.push_back(p.x());
                xs.push_back(p.y());
                xs.push_back(p.z());
            }
            ExtendedBlock eblock = block->get_extended_block();
            // 对扩展块的每一块都进行处理以得到当前test_block的信息
            for (auto block_it = eblock.cbegin(); block_it != eblock.cend(); ++block_it) {
                auto bgk = bgk_arr_hits.find(*block_it);
                if (bgk == bgk_arr_hits.end())
                    continue;

               	vector<vector<float>> ybars;  //一个矩阵，行数为块的个数，列数为种类的数目，这样就得到了所有块到这个块的所有类别信息
		            bgk->second->predict(xs, ybars);  

                int j = 0;
                for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) {
                    SemanticOcTreeNode &node = leaf_it.get_node();
                    // Only need to update if kernel density total kernel density est > 0
                    //if (kbar[j] > 0.0)
                    node.update(ybars[j]);  // 把这个块的每个节点类别信息进行更新
                }
            }
        }

        if (if_use_free)
        {
            for (int i = 0; i < test_blocks_frees.size(); ++i) {   //对于值得预测的frees block依次进行预测
                BlockHashKey key = test_blocks_frees[i];
    #ifdef OPENMP
    #pragma omp critical
    #endif
                {
                    if (block_arr.find(key) == block_arr.end())   //如果当前测试块在block_arr（应该是用来存储所有block的了）中没有找到
                        block_arr.emplace(key, new Block(hash_key_to_block(key)));  //就新建一个
                };
                Block *block = block_arr[key];

                // vector<float> ybars;
                // ybars = { 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
                // for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                //     SemanticOcTreeNode &node = leaf_it.get_node();
                //     // Only need to update if kernel density total kernel density est > 0
                //     node.update(ybars);
                // }

                vector<float> xs;  //这里面是test_blocks当前这个block的所有的的节点（子块）的xyz
                for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                    point3f p = block->get_loc(leaf_it);
                    xs.push_back(p.x());
                    xs.push_back(p.y());
                    xs.push_back(p.z());
                }
                // 和BKI的不用在于，只需要使用该block中的gp点进行更新。
                auto bgk = bgk_arr_frees.find(key);
                if (bgk == bgk_arr_frees.end())
                continue;
                vector<vector<float>> ybars;
                bgk->second->predict(xs, ybars);
                int j = 0;
                for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) {
                    SemanticOcTreeNode &node = leaf_it.get_node();
                    node.update(ybars[j]);
                }
            }
        }

        ////////// Cleaning /////////////////////////////
        /////////////////////////////////////////////////
        // 直接清空？
        for (auto it = bgk_arr_hits.begin(); it != bgk_arr_hits.end(); ++it)
            delete it->second;

        for (auto it = bgk_arr_frees.begin(); it != bgk_arr_frees.end(); ++it)
            delete it->second;

        rtree_frees.RemoveAll();
        rtree_hits.RemoveAll();

        /////////////////debug//////////////////////////
        // std::cout << block_arr.size() << std::endl;
    }


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// ---------------------------------------------------------------------------------------分界线之TEST模型------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

    void SemanticBKIOctoMap::insert_pointcloud_test(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                                      float free_res, float max_range) {

        ////////// Preparation //////////////////////////
        /////////////////////////////////////////////////
        // ros::Time start = ros::Time::now();
        GPPointCloud xy;
        get_training_data(cloud, origin, ds_resolution, free_res, max_range, xy);
        // std::cout << "hits_num:  " << xy.size() <<std::endl;   
        if (xy.size() == 0) {
            return;
        }
        point3f lim_min, lim_max;
        bbox(xy, lim_min, lim_max);

        vector<BlockHashKey> blocks;
        get_blocks_in_bbox(lim_min, lim_max, blocks);
 
        for (auto it = xy.cbegin(); it != xy.cend(); ++it) 
        {
            float p[] = {it->first.x(), it->first.y(), it->first.z()};
            rtree.Insert(p, p, const_cast<GPPointType *>(&*it));
        }

        // ros::Time end = ros::Time::now();
        // ROS_INFO_STREAM("Preparation finished in " << (end - start).toSec() << "s");
        /////////////////////////////////////////////////

        ////////// Training /////////////////////////////
        /////////////////////////////////////////////////
        //  start = ros::Time::now();
// #pragma omp parallel for schedule(dynamic)
#pragma omp parallel for 
        for (int i = 0; i < blocks.size(); ++i) 
        {
            BlockHashKey key = blocks[i];

            if (has_gp_points_in_bbox(key) )
            {
                if (block_arr.find(key) == block_arr.end())   //如果当前测试块在block_arr（应该是用来存储所有block的了）中没有找到
#ifdef OPENMP
#pragma omp critical
#endif
                {
                    block_arr.emplace(key, new Block(hash_key_to_block(key)));  //就新建一个
                }
    
                Block *block = block_arr[key];
                
                GPPointCloud block_xy;
                get_gp_points_in_bbox(key, block_xy);

                vector<float> block_x, block_y;
                for (auto it = block_xy.cbegin(); it != block_xy.cend(); ++it) {
                    block_x.push_back(it->first.x());
                    block_x.push_back(it->first.y());
                    block_x.push_back(it->first.z());
                    block_y.push_back(it->second);
                }


                // 求类别熵
                //遍历，获得每个元素出现的次数

	            vector<int> class_num( SemanticOcTreeNode::num_class, 0);
#pragma omp parallel for
	            for(int r=0; r<block_y.size(); r++)
                {
		            class_num[(int)block_y[r]]++;
                }
                int num = 0;
                for(int r=0; r<SemanticOcTreeNode::num_class; r++)
                {
                    if ( class_num[r] != 0)
                        num ++;
                }
                auto maxPosition = max_element(class_num.begin(), class_num.end());
                int max_num = *maxPosition;
                float class_ent =  1.0 - (float)*maxPosition/block_y.size() + log(num)/log(SemanticOcTreeNode::num_class)/SemanticOcTreeNode::num_class;
                                                    // ( 1.0 + (float)(num-1)/(SemanticOcTreeNode::num_class-1)/(SemanticOcTreeNode::num_class-1) );
                // float class_ent =  1.0 - (float)*maxPosition/block_y.size() ;
                if (class_ent > 1 or class_ent < 0)
                {
                    std::cout << "no way" << std::endl;
                }

                SemanticBKI3f *bgk = new SemanticBKI3f(SemanticOcTreeNode::num_class, SemanticOcTreeNode::sf2, 3*resolution+2*class_ent*resolution);
                
                if (class_ent <= 0.3)
                    bgk->train(block_x, block_y);
                else
                {
                    ExtendedBlock eblock = get_extended_block(key);   //eblock是当前block的扩展block的哈希键列表.
                    GPPointCloud eblock_xy;
                    get_gp_points_in_bbox(eblock, eblock_xy);
                    vector<float> eblock_x, eblock_y;
                    for (auto it = eblock_xy.cbegin(); it != eblock_xy.cend(); ++it) 
                    {
                        eblock_x.push_back(it->first.x());
                        eblock_x.push_back(it->first.y());
                        eblock_x.push_back(it->first.z());
                        eblock_y.push_back(it->second);
                    }
                    bgk->train(eblock_x, eblock_y);
                }

                vector<float> xs;  //这里面是test_blocks当前这个block的所有的的节点（子块）的xyz
                for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                    point3f p = block->get_loc(leaf_it);
                    xs.push_back(p.x());
                    xs.push_back(p.y());
                    xs.push_back(p.z());
                }
                // 和BKI的不用在于，只需要使用该block中的gp点进行更新。
                vector<vector<float>> ybars;
                bgk->predict(xs, ybars);
                int j = 0;
                for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) {
                    SemanticOcTreeNode &node = leaf_it.get_node();
                    node.update(ybars[j]);
                }
            }
            else
            {
                ExtendedBlock eblock = get_extended_block(key);   //eblock是当前block的扩展block的哈希键列表.
                int num = 0;
                // 模型的简单写法，效果完全相同
                for (auto it = eblock.cbegin(); it != eblock.cend(); ++it) 
                {
                    if (has_gp_points_in_bbox(*it) > 0)
                        num++;
                }
                if (num >= 2)
                {
                     if (block_arr.find(key) == block_arr.end())   //如果当前测试块在block_arr（应该是用来存储所有block的了）中没有找到
#ifdef OPENMP
#pragma omp critical
#endif
                    {
                        block_arr.emplace(key, new Block(hash_key_to_block(key)));  //就新建一个
                    }
                    Block *block = block_arr[key];
                
                    GPPointCloud block_xy;
                    get_gp_points_in_bbox(eblock, block_xy);       

                    vector<float> block_x, block_y;
                    for (auto it = block_xy.cbegin(); it != block_xy.cend(); ++it) 
                    {
                        block_x.push_back(it->first.x());
                        block_x.push_back(it->first.y());
                        block_x.push_back(it->first.z());
                        block_y.push_back(it->second);
                    }

                    SemanticBKI3f *bgk = new SemanticBKI3f(SemanticOcTreeNode::num_class, SemanticOcTreeNode::sf2,  5*resolution);
                    bgk->train(block_x, block_y);

                    vector<float> xs;  //这里面是test_blocks当前这个block的所有的的节点（子块）的xyz
                    for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                        point3f p = block->get_loc(leaf_it);
                        xs.push_back(p.x());
                        xs.push_back(p.y());
                        xs.push_back(p.z());
                    }
                    // 和BKI的不用在于，只需要使用该block中的gp点进行更新。
                    vector<vector<float>> ybars;
                    bgk->predict(xs, ybars);
                    int j = 0;
                    for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) {
                        SemanticOcTreeNode &node = leaf_it.get_node();
                        node.update(ybars[j]);
                    }
                }
            }
        }

        // end = ros::Time::now();
        // ROS_INFO_STREAM("Train finished in " << (end - start).toSec() << "s");
        ////////// Cleaning /////////////////////////////
        /////////////////////////////////////////////////
        rtree.RemoveAll();
    }



//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// ---------------------------------------------------------------------------------------分界线之BKI模型------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void SemanticBKIOctoMap::insert_pointcloud_raw(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                                      float free_res, float max_range) {

#ifdef DEBUG
        Debug_Msg("Insert pointcloud: " << "cloud size: " << cloud.size() << " origin: " << origin);
#endif

        ////////// Preparation //////////////////////////
        /////////////////////////////////////////////////
        GPPointCloud xy;
        get_training_data(cloud, origin, ds_resolution, free_res, max_range, xy);
        // std::cout << "hits_num:  " << xy.size() <<std::endl;   

#ifdef DEBUG
        Debug_Msg("Training data size: " << xy.size());
#endif
        // If pointcloud after max_range filtering is empty
        //  no need to do anything
        if (xy.size() == 0) {
            return;
        }

        point3f lim_min, lim_max;
        bbox(xy, lim_min, lim_max);

        vector<BlockHashKey> blocks;
        get_blocks_in_bbox(lim_min, lim_max, blocks);

        for (auto it = xy.cbegin(); it != xy.cend(); ++it) {
            float p[] = {it->first.x(), it->first.y(), it->first.z()};
            rtree.Insert(p, p, const_cast<GPPointType *>(&*it));
        }
        /////////////////////////////////////////////////

        ////////// Training /////////////////////////////
        /////////////////////////////////////////////////
        vector<BlockHashKey> test_blocks;
        std::unordered_map<BlockHashKey, SemanticBKI3f *> bgk_arr;
        for (int i = 0; i < blocks.size(); ++i) {
            BlockHashKey key = blocks[i];
            ExtendedBlock eblock = get_extended_block(key);
            if (has_gp_points_in_bbox(eblock))
#ifdef OPENMP
#pragma omp critical
#endif
            {
                test_blocks.push_back(key);
            };

            GPPointCloud block_xy;
            get_gp_points_in_bbox(key, block_xy);
            if (block_xy.size() < 1)
                continue;

            vector<float> block_x, block_y;
            for (auto it = block_xy.cbegin(); it != block_xy.cend(); ++it) {
                block_x.push_back(it->first.x());
                block_x.push_back(it->first.y());
                block_x.push_back(it->first.z());
                block_y.push_back(it->second);
            }

            SemanticBKI3f *bgk = new SemanticBKI3f(SemanticOcTreeNode::num_class, SemanticOcTreeNode::sf2, SemanticOcTreeNode::ell);
            bgk->train(block_x, block_y);
#ifdef OPENMP
#pragma omp critical
#endif
            {
                bgk_arr.emplace(key, bgk);
            };
        }
        // std::cout << "hit block num: " << test_blocks.size() <<std::endl;
#ifdef DEBUG
        Debug_Msg("Training done");
        Debug_Msg("Prediction: block number: " << test_blocks.size());
#endif
        /////////////////////////////////////////////////

        ////////// Prediction ///////////////////////////
        /////////////////////////////////////////////////
#ifdef OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < test_blocks.size(); ++i) {
            BlockHashKey key = test_blocks[i];
#ifdef OPENMP
#pragma omp critical
#endif
            {
                if (block_arr.find(key) == block_arr.end())
                    block_arr.emplace(key, new Block(hash_key_to_block(key)));
            };
            Block *block = block_arr[key];
            vector<float> xs;
            for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it) {
                point3f p = block->get_loc(leaf_it);
                xs.push_back(p.x());
                xs.push_back(p.y());
                xs.push_back(p.z());
            }
            //std::cout << "xs size: "<<xs.size() << std::endl;

            ExtendedBlock eblock = block->get_extended_block();
            for (auto block_it = eblock.cbegin(); block_it != eblock.cend(); ++block_it) {
                auto bgk = bgk_arr.find(*block_it);
                if (bgk == bgk_arr.end())
                    continue;

               	vector<vector<float>> ybars;
		            bgk->second->predict(xs, ybars);

                int j = 0;
                for (auto leaf_it = block->begin_leaf(); leaf_it != block->end_leaf(); ++leaf_it, ++j) {
                    SemanticOcTreeNode &node = leaf_it.get_node();
                    // Only need to update if kernel density total kernel density est > 0
                    //if (kbar[j] > 0.0)
                    node.update(ybars[j]);
                }
            }
        }
#ifdef DEBUG
        Debug_Msg("Prediction done");
#endif

        ////////// Cleaning /////////////////////////////
        /////////////////////////////////////////////////
        for (auto it = bgk_arr.begin(); it != bgk_arr.end(); ++it)
            delete it->second;

        rtree.RemoveAll();
    }



    // 得到当前地图的最大界和最小界面
    void SemanticBKIOctoMap::get_bbox(point3f &lim_min, point3f &lim_max) const {
        lim_min = point3f(0, 0, 0);
        lim_max = point3f(0, 0, 0);

        GPPointCloud centers;
        for (auto it = block_arr.cbegin(); it != block_arr.cend(); ++it) {
            centers.emplace_back(it->second->get_center(), 1);
        }
        if (centers.size() > 0) {
            bbox(centers, lim_min, lim_max);
            lim_min -= point3f(block_size, block_size, block_size) * 0.5;
            lim_max += point3f(block_size, block_size, block_size) * 0.5;
        }
    }
    // 从一个传感器扫描获取训练数据，在xy中存储
    void SemanticBKIOctoMap::get_training_data(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                                      float free_resolution, float max_range, GPPointCloud &xy) const {

        PCLPointCloud sampled_hits;  //命中点
        downsample(cloud, sampled_hits, ds_resolution);  // 先降采样，减少点的个数，得到sampled_hits

        PCLPointCloud frees;  //未命中点
        frees.height = 1;
        frees.width = 0;
        xy.clear();
        for (auto it = sampled_hits.begin(); it != sampled_hits.end(); ++it) {
            point3f p(it->x, it->y, it->z);
            if (max_range > 0) {
                double l = (p - origin).norm();  // 到原点距离大小
                // double l = p.norm();
                if (l > max_range)
                    continue;
            }
            
            xy.emplace_back(p, it->label);   //在距离范围内把击中点的坐标和标签存入

            PointCloud frees_n;
            beam_sample(p, origin, frees_n, free_resolution); //根据每一个hit_points，采样得到free点

            // 认为原点是free_points，所以至少一个free_points了
            PCLPointType p_origin = PCLPointType();
            p_origin.x = origin.x();
            p_origin.y = origin.y();
            p_origin.z = origin.z();
            p_origin.label = 0; 
            frees.push_back(p_origin);
            
            for (auto p = frees_n.begin(); p != frees_n.end(); ++p) {
                PCLPointType p_free = PCLPointType();
                p_free.x = p->x();
                p_free.y = p->y();
                p_free.z = p->z();
                p_free.label = 0;   // free点label为0，所以其他的label绝对不能为0
                frees.push_back(p_free);
                frees.width++;
            }
        }

        PCLPointCloud sampled_frees;    
        downsample(frees, sampled_frees, ds_resolution);   // 对frees也降采样，得到sampled_frees
        // std::cout << "hits_num:  " << sampled_hits.size() <<std::endl;   
        // std::cout << "frees_num:  " << sampled_frees.size() <<std::endl;   

        for (auto it = sampled_frees.begin(); it != sampled_frees.end(); ++it) {
            xy.emplace_back(point3f(it->x, it->y, it->z), 0.0f);    //把free点的坐标和标签0存入
        }
    }

    // 从一个传感器扫描获取训练数据，在xy中存储
    void SemanticBKIOctoMap::get_training_data(const PCLPointCloud &cloud, const point3f &origin, float ds_resolution,
                                      float free_resolution, float max_range, GPPointCloud &xy, GPPointCloud &xy_hits, GPPointCloud &xy_frees) const {

        PCLPointCloud sampled_hits;  //命中点
        downsample(cloud, sampled_hits, ds_resolution);  // 先降采样，减少点的个数，得到sampled_hits

        PCLPointCloud frees;  //未命中点
        frees.height = 1;
        frees.width = 0;
        xy.clear();
        for (auto it = sampled_hits.begin(); it != sampled_hits.end(); ++it) {
            point3f p(it->x, it->y, it->z);
            if (max_range > 0) {
                double l = (p - origin).norm();  // 到原点距离大小
                // double l = p.norm();
                if (l > max_range)
                    continue;
            }
            
            xy.emplace_back(p, it->label);   //在距离范围内把击中点的坐标和标签存入
            xy_hits.emplace_back(p, it->label);   //在距离范围内把击中点的坐标和标签存入

            PointCloud frees_n;
            beam_sample(p, origin, frees_n, free_resolution); //根据每一个hit_points，采样得到free点

            // 认为原点是free_points，所以至少一个free_points了
            PCLPointType p_origin = PCLPointType();
            p_origin.x = origin.x();
            p_origin.y = origin.y();
            p_origin.z = origin.z();
            p_origin.label = 0; 
            frees.push_back(p_origin);
            
            for (auto p = frees_n.begin(); p != frees_n.end(); ++p) {
                PCLPointType p_free = PCLPointType();
                p_free.x = p->x();
                p_free.y = p->y();
                p_free.z = p->z();
                p_free.label = 0;   // free点label为0，所以其他的label绝对不能为0
                frees.push_back(p_free);
                frees.width++;
            }
        }

        PCLPointCloud sampled_frees;    
        downsample(frees, sampled_frees, ds_resolution);   // 对frees也降采样，得到sampled_frees

        for (auto it = sampled_frees.begin(); it != sampled_frees.end(); ++it) {
            xy.emplace_back(point3f(it->x, it->y, it->z), 0.0f);    //把free点的坐标和标签0存入
            xy_frees.emplace_back(point3f(it->x, it->y, it->z), 0.0f);    //把free点的坐标和标签0存入
        }
    }

    // 点云的下采样，就是把点云滤波成ds_resolution大小样子
    void SemanticBKIOctoMap::downsample(const PCLPointCloud &in, PCLPointCloud &out, float ds_resolution) const {
        if (ds_resolution < 0) {
            std::cout << "out == in" <<std::endl;
            out = in;
            return;
        }

        PCLPointCloud::Ptr pcl_in(new PCLPointCloud(in));

        pcl::VoxelGrid<PCLPointType> sor;
        sor.setInputCloud(pcl_in);
        sor.setLeafSize(ds_resolution, ds_resolution, ds_resolution);
        sor.filter(out);
    }

    // 根据光束击中点，从中得到free点
    void SemanticBKIOctoMap::beam_sample(const point3f &hit, const point3f &origin, PointCloud &frees,
                                float free_resolution) const {
        frees.clear();

        float x0 = origin.x();
        float y0 = origin.y();
        float z0 = origin.z();

        float x = hit.x();
        float y = hit.y();
        float z = hit.z();

        float l = (float) sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0) + (z - z0) * (z - z0));  // 这里的xyz应该是全局坐标系下，确实，因为转化到了全局坐标系

        float nx = (x - x0) / l;
        float ny = (y - y0) / l;
        float nz = (z - z0) / l;

        float d = free_resolution;

        // 之前的插值
        while (d < l) {
            frees.emplace_back(x0 + nx * d, y0 + ny * d, z0 + nz * d);
            d += free_resolution;
        }
        // 最后再补充上最后一个点
        if (l > free_resolution)
            frees.emplace_back(x0 + nx * (l - free_resolution), y0 + ny * (l - free_resolution), z0 + nz * (l - free_resolution));

    }







    /*
     * Compute bounding box of pointcloud
     * Precondition: cloud non-empty
     */
    
    // *计算点云的边界框：lim_min和lim_max
    // *前提条件：云非空 
    void SemanticBKIOctoMap::bbox(const GPPointCloud &cloud, point3f &lim_min, point3f &lim_max) const {
        assert(cloud.size() > 0);
        vector<float> x, y, z;
        for (auto it = cloud.cbegin(); it != cloud.cend(); ++it) {
            x.push_back(it->first.x());
            y.push_back(it->first.y());
            z.push_back(it->first.z());
        }

        auto xlim = std::minmax_element(x.cbegin(), x.cend());
        auto ylim = std::minmax_element(y.cbegin(), y.cend());
        auto zlim = std::minmax_element(z.cbegin(), z.cend());

        lim_min.x() = *xlim.first;
        lim_min.y() = *ylim.first;
        lim_min.z() = *zlim.first;

        lim_max.x() = *xlim.second;
        lim_max.y() = *ylim.second;
        lim_max.z() = *zlim.second;

        
    }

    // 在边界框中分块，还往小的地方多分了一块，往大的地方多分了两块。将块的哈希键存入blocks中
    void SemanticBKIOctoMap::get_blocks_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                       vector<BlockHashKey> &blocks) const {
        for (float x = lim_min.x() - block_size; x <= lim_max.x() + 2 * block_size; x += block_size) {
            for (float y = lim_min.y() - block_size; y <= lim_max.y() + 2 * block_size; y += block_size) {
                for (float z = lim_min.z() - block_size; z <= lim_max.z() + 2 * block_size; z += block_size) {
                    blocks.push_back(block_to_hash_key(x, y, z));
                }
            }
        }
    }

    // 根据block的哈希键，返回这个块里面的gp点
    int SemanticBKIOctoMap::get_gp_points_in_bbox(const BlockHashKey &key,
                                         GPPointCloud &out) {
        point3f half_size(block_size / 2.0f, block_size / 2.0f, block_size / 2.0);  //块尺寸的一半
        point3f lim_min = hash_key_to_block(key) - half_size;
        point3f lim_max = hash_key_to_block(key) + half_size;
        return get_gp_points_in_bbox(lim_min, lim_max, out);
    }

    // 根据block的哈希键，返回这个块里面的hits gp点
    int SemanticBKIOctoMap::get_hits_gp_points_in_bbox(const BlockHashKey &key,
                                         GPPointCloud &out) {
        point3f half_size(block_size / 2.0f, block_size / 2.0f, block_size / 2.0);  //块尺寸的一半
        point3f lim_min = hash_key_to_block(key) - half_size;
        point3f lim_max = hash_key_to_block(key) + half_size;
        return get_hits_gp_points_in_bbox(lim_min, lim_max, out);
    }

    // 根据block的哈希键，返回这个块里面的frees gp点
    int SemanticBKIOctoMap::get_frees_gp_points_in_bbox(const BlockHashKey &key,
                                         GPPointCloud &out) {
        point3f half_size(block_size / 2.0f, block_size / 2.0f, block_size / 2.0);  //块尺寸的一半
        point3f lim_min = hash_key_to_block(key) - half_size;
        point3f lim_max = hash_key_to_block(key) + half_size;
        return get_frees_gp_points_in_bbox(lim_min, lim_max, out);
    }

     // 看扩展夸块中是否有gp点
    int SemanticBKIOctoMap::has_gp_points_in_bbox(const ExtendedBlock &block) {
        for (auto it = block.cbegin(); it != block.cend(); ++it) {
            if (has_gp_points_in_bbox(*it) > 0)
                return 1;
        }
        return 0;
    }

    int SemanticBKIOctoMap::get_gp_points_in_bbox(const ExtendedBlock &block,
                                         GPPointCloud &out) {
        int n = 0;
        for (auto it = block.cbegin(); it != block.cend(); ++it) {
            n += get_gp_points_in_bbox(*it, out);
        }
        return n;
    }
    
    // 根据block的哈希键判断里面是否有gp点
    int SemanticBKIOctoMap::has_gp_points_in_bbox(const BlockHashKey &key) {
        point3f half_size(block_size / 2.0f, block_size / 2.0f, block_size / 2.0);
        point3f lim_min = hash_key_to_block(key) - half_size;
        point3f lim_max = hash_key_to_block(key) + half_size;
        return has_gp_points_in_bbox(lim_min, lim_max);
    }

    // 根据block的哈希键判断里面是否有hits gp点
    int SemanticBKIOctoMap::has_hits_gp_points_in_bbox(const BlockHashKey &key) {
        point3f half_size(block_size / 2.0f, block_size / 2.0f, block_size / 2.0);
        point3f lim_min = hash_key_to_block(key) - half_size;
        point3f lim_max = hash_key_to_block(key) + half_size;
        return has_hits_gp_points_in_bbox(lim_min, lim_max);
    }

    // 根据block的哈希键判断里面是否有frees gp点
    int SemanticBKIOctoMap::has_frees_gp_points_in_bbox(const BlockHashKey &key) {
        point3f half_size(block_size / 2.0f, block_size / 2.0f, block_size / 2.0);
        point3f lim_min = hash_key_to_block(key) - half_size;
        point3f lim_max = hash_key_to_block(key) + half_size;
        return has_frees_gp_points_in_bbox(lim_min, lim_max);
    }

    // 根据block的最大最小位置，返回这个块里面的gp点
    int SemanticBKIOctoMap::get_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                         GPPointCloud &out) {
        float a_min[] = {lim_min.x(), lim_min.y(), lim_min.z()};
        float a_max[] = {lim_max.x(), lim_max.y(), lim_max.z()};
        return rtree.Search(a_min, a_max, SemanticBKIOctoMap::search_callback, static_cast<void *>(&out));
    }

    // 根据block的最大最小位置，返回这个块里面的hits gp点
    int SemanticBKIOctoMap::get_hits_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                         GPPointCloud &out) {
        float a_min[] = {lim_min.x(), lim_min.y(), lim_min.z()};
        float a_max[] = {lim_max.x(), lim_max.y(), lim_max.z()};
        return rtree_hits.Search(a_min, a_max, SemanticBKIOctoMap::search_callback, static_cast<void *>(&out));
    }

    // 根据block的最大最小位置，返回这个块里面的free gp点
    int SemanticBKIOctoMap::get_frees_gp_points_in_bbox(const point3f &lim_min, const point3f &lim_max,
                                         GPPointCloud &out) {
        float a_min[] = {lim_min.x(), lim_min.y(), lim_min.z()};
        float a_max[] = {lim_max.x(), lim_max.y(), lim_max.z()};
        return rtree_frees.Search(a_min, a_max, SemanticBKIOctoMap::search_callback, static_cast<void *>(&out));
    }

    //根据block的最大位置和最小位置判断里面是否有gp点，返回点的个数
    int SemanticBKIOctoMap::has_gp_points_in_bbox(const point3f &lim_min,
                                         const point3f &lim_max) {
        float a_min[] = {lim_min.x(), lim_min.y(), lim_min.z()};
        float a_max[] = {lim_max.x(), lim_max.y(), lim_max.z()};
        return rtree.Search(a_min, a_max, SemanticBKIOctoMap::count_callback, NULL);
    }

    //根据block的最大位置和最小位置判断里面是否有hit gp点，返回点的个数
    int SemanticBKIOctoMap::has_hits_gp_points_in_bbox(const point3f &lim_min,
                                         const point3f &lim_max) {
        float a_min[] = {lim_min.x(), lim_min.y(), lim_min.z()};
        float a_max[] = {lim_max.x(), lim_max.y(), lim_max.z()};
        return rtree_hits.Search(a_min, a_max, SemanticBKIOctoMap::count_callback, NULL);
    }

    //根据block的最大位置和最小位置判断里面是否有free gp点，返回点的个数
    int SemanticBKIOctoMap::has_frees_gp_points_in_bbox(const point3f &lim_min,
                                         const point3f &lim_max) {
        float a_min[] = {lim_min.x(), lim_min.y(), lim_min.z()};
        float a_max[] = {lim_max.x(), lim_max.y(), lim_max.z()};
        return rtree_frees.Search(a_min, a_max, SemanticBKIOctoMap::count_callback, NULL);
    }

    bool SemanticBKIOctoMap::count_callback(GPPointType *p, void *arg) {
        return false;
    }

    bool SemanticBKIOctoMap::search_callback(GPPointType *p, void *arg) {
        GPPointCloud *out = static_cast<GPPointCloud *>(arg);
        out->push_back(*p);
        return true;
    }



    // // 看扩展夸块中是否有gp点，不需要了
    // int SemanticBKIOctoMap::has_gp_points_in_bbox(const ExtendedBlock &block) {
    //     for (auto it = block.cbegin(); it != block.cend(); ++it) {
    //         if (has_gp_points_in_bbox(*it) > 0)
    //             return 1;
    //     }
    //     return 0;
    // }

    // int SemanticBKIOctoMap::get_gp_points_in_bbox(const ExtendedBlock &block,
    //                                      GPPointCloud &out) {
    //     int n = 0;
    //     for (auto it = block.cbegin(); it != block.cend(); ++it) {
    //         n += get_gp_points_in_bbox(*it, out);
    //     }
    //     return n;
    // }

    Block *SemanticBKIOctoMap::search(BlockHashKey key) const {
        auto block = block_arr.find(key);
        if (block == block_arr.end()) {
            return nullptr;
        } else {
            return block->second;
        }
    }

    SemanticOcTreeNode SemanticBKIOctoMap::search(point3f p) const {
        Block *block = search(block_to_hash_key(p));
        if (block == nullptr) {
          return SemanticOcTreeNode();
        } else {
          return SemanticOcTreeNode(block->search(p));
        }
    }

    SemanticOcTreeNode SemanticBKIOctoMap::search(float x, float y, float z) const {
        return search(point3f(x, y, z));
    }
}
