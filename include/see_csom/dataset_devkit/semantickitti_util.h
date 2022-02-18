#pragma once

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

class SemanticKITTIData {
  public:
    SemanticKITTIData(ros::NodeHandle& nh,
             double resolution, double block_depth,
             double sf2, double ell,
             int num_class, double free_thresh,
             double occupied_thresh, float var_thresh, 
	           double ds_resolution,
             double free_resolution, double max_range,
             std::string map_topic,
             float prior)
      : nh_(nh)
      , resolution_(resolution)
      , ds_resolution_(ds_resolution)
      , free_resolution_(free_resolution)
      , max_range_(max_range) {
        map_ = new see_csom::SemanticBKIOctoMap(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
        m_pub_ = new see_csom::MarkerArrayPub(nh_, map_topic, resolution);
      	init_trans_to_ground_ << 1, 0, 0, 0,
                                 0, 0, 1, 0,
                                 0,-1, 0, 1,
                                 0, 0, 0, 1;
        // SemanticKitti的标签标号，对应颜色在markerarray_pub.h中
        label2label =  { 0,  1,  10, 11, 13, 15, 16, 18, 20, 30,  31,  32,  40,  44,  48,  49,  50,
                             51, 52, 60, 70, 71, 72, 80, 81, 99, 252, 256, 253, 254, 255, 257, 258, 259 };
        label3label =  { 0,  0,  10, 11, 20, 15, 20, 18, 20, 30,  31,  32,  40,  44,  48,  49,  50,
                             51, 0, 40, 70, 71, 72, 80, 81, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        label4label =  { 0,  10,  11, 15, 18,  20, 30,  31,  32,  40,  44,  48,  49,  50, 70, 71, 72, 80, 81};
        label5label =  { 0,  0,  1, 2, 5, 3, 5, 4,  5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0, 9, 15, 16, 17, 18,
                                      19, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      }

    bool read_lidar_poses(const std::string lidar_pose_name) {
      if (std::ifstream(lidar_pose_name)) {
        std::ifstream fPoses;
        fPoses.open(lidar_pose_name.c_str());
        while (!fPoses.eof()) {
          std::string s;
          std::getline(fPoses, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 3; ++i)
              for (int j = 0; j < 4; ++j)
                ss >> t_matrix(i, j);
            lidar_poses_.push_back(t_matrix);
          }
        }
        fPoses.close();
        return true;
        } else {
         ROS_ERROR_STREAM("Cannot open evaluation list file " << lidar_pose_name);
         return false;
      }
    } 

    bool process_scans(std::string input_data_dir, std::string input_label_dir, int scan_num, bool query, bool visualize) {
      see_csom::point3f origin;
      ros::Time start = ros::Time::now();
      for (int scan_id  = 0; scan_id < scan_num; ++scan_id) {
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", scan_id);
        std::string scan_name = input_data_dir + std::string(scan_id_c) + ".bin";
        std::string label_name = input_label_dir + std::string(scan_id_c) + ".label";
        pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = kitti2pcl(scan_name, label_name);
        Eigen::Matrix4d transform = lidar_poses_[scan_id];
        Eigen::Matrix4d calibration;

        // 04
        // calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
        //                 -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
        //                  0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
       	//                  0                ,  0                ,  0                ,  1.000000000000000;

        //08
        calibration << -1.857739385241e-03,  -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                          -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
                          9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
                          0                                        ,0                                           ,0                                            ,1.00;
	      
        Eigen::Matrix4d new_transform = init_trans_to_ground_ * transform * calibration;
        pcl::transformPointCloud (*cloud, *cloud, new_transform);
        origin.x() = transform(0, 3);
        origin.y() = transform(1, 3);
        origin.z() = transform(2, 3);
        map_->insert_pointcloud_test(*cloud, origin, ds_resolution_, free_resolution_, max_range_);
        std::cout << "Inserted point cloud at " << scan_name << std::endl;
        
      }
      ros::Time end = ros::Time::now();
      ROS_INFO_STREAM("SEE-CSOM finished in " << (end - start).toSec() << "s");
      publish_map();
      return 1;
    }

    void publish_map() {
      m_pub_->clear_map(resolution_);
      // float x_max = 0.0;
      // float y_max = 0.0;
      // float z_max = 0.0;
      // float x_min = 100.0;
      // float y_min = 100.0;
      // float z_min = 100.0;
      for (auto it = map_->begin_leaf(); it != map_->end_leaf(); ++it) {
        // if (it.get_node().get_state() == see_csom::State::OCCUPIED) {
        if ((it.get_node().get_state() == see_csom::State::OCCUPIED) && (it.get_node().get_semantics() > 1)){
          see_csom::point3f p = it.get_loc();
          if (p.z() > -3)
          {
            // if (p.x() > x_max)
            //     x_max = p.x();
            // if (p.y() > y_max)
            //     y_max = p.y();
            // if (p.z() > z_max)
            //     z_max = p.z();
            // if (p.x() < x_min)
            //     x_min = p.x();
            // if (p.y() < y_min)
            //     y_min = p.y();
            // if (p.z() < z_min)
            //     z_min = p.z();
            m_pub_->insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), it.get_node().get_semantics(), 2);
          }
        }
      }
      // std::cout << "x: " << x_max << "    y: "<<y_max<< "  z: "<<z_max <<std::endl;
      // std::cout << "x: " << x_min << "    y: "<<y_min<< "  z: "<<z_min <<std::endl;

      // 04真值
      // x: 48.75    y: 61.05  z: 3.45
      // x: -49.95    y: -48.75  z: -2.85
      // 最后使用
      // x: -48  48  320
      // y: -48  60  360
      // z: -3.0  3.6  22

      // 08真值
      // x: 49.95    y: 55.65  z: 3.75
      // x: -49.95    y: -50.25  z: -2.85
      // 最后使用
      // x: -51  51  340
      // y: -51  57  360
      // z: -3.0  3.9  23
      map_->print_vaule_2();
      m_pub_->publish();
    }

    void set_up_evaluation(const std::string gt_label_dir, const std::string evaluation_result_dir) {
      gt_label_dir_ = gt_label_dir;
      evaluation_result_dir_ = evaluation_result_dir;
    }

    void query_scan(std::string input_data_dir, int scan_id) {
      // std::cout << scan_id <<std::endl;
      char scan_id_c[256];
      sprintf(scan_id_c, "%06d", scan_id);
      std::string scan_name = input_data_dir + std::string(scan_id_c) + ".bin";
      std::string gt_name = gt_label_dir_ + std::string(scan_id_c) + ".label";
      // std::string result_name = evaluation_result_dir_ + std::string(scan_id_c) + ".txt";
      std::string mylabel_name = evaluation_result_dir_ + std::string(scan_id_c) + ".label";
      pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = kitti2pcl(scan_name, gt_name);
      Eigen::Matrix4d transform = lidar_poses_[scan_id];
      Eigen::Matrix4d calibration;
      
      // 04
      calibration <<  -0.001857739385241, -0.999965951350955, -0.008039975204516, -0.004784029760483,
                      -0.006481465826011,  0.008051860151134, -0.999946608177406, -0.073374294642306,
                       0.999977309828677, -0.001805528627661, -0.006496203536139, -0.333996806443304,
                       0                ,  0                ,  0                ,  1.000000000000000;
      
      Eigen::Matrix4d new_transform = init_trans_to_ground_ * transform * calibration;
      pcl::transformPointCloud (*cloud, *cloud, new_transform);

      // std::ofstream result_file;
      // result_file.open(result_name);
      FILE* fp_label = std::fopen(mylabel_name.c_str(), "w");

      // std::cout << cloud->points.size() << std::endl;
      for (int i = 0; i < cloud->points.size(); ++i) {
        see_csom::SemanticOcTreeNode node = map_->search(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        int pred_label = 0;
        pred_label = node.get_semantics();
        int pred_label_raw;

        // result_file << cloud->points[i].label << " " << pred_label << "\n";

        if (pred_label > 0)
        {
          if (pred_label > 34)
          {
            std::cout << ">0:  "<<pred_label << std::endl;
          }
          else{
            pred_label_raw = label2label[pred_label-1];
          }
        }
        else
          pred_label_raw = 0;
        std::uint32_t my_label = (std::uint32_t)pred_label_raw;
        if (fwrite(&my_label, sizeof(std::uint32_t), 1, fp_label) == 0) break;
      }
      // result_file.close();
      std::fclose(fp_label);
    }

  
  private:
    ros::NodeHandle nh_;
    double resolution_;
    double ds_resolution_;
    double free_resolution_;
    double max_range_;
    see_csom::SemanticBKIOctoMap* map_;
    see_csom::MarkerArrayPub* m_pub_;
    ros::Publisher color_octomap_publisher_;
    tf::TransformListener listener_;
    std::ofstream pose_file_;
    std::vector<Eigen::Matrix4d> lidar_poses_;
    std::string gt_label_dir_;
    std::string evaluation_result_dir_;
    Eigen::Matrix4d init_trans_to_ground_;
    std::vector<int> label2label;  // semantickitti的标签类别
    std::vector<int> label3label;  // semantickitti的标签类别
    std::vector<int> label4label;  // semantickitti的标签类别
    std::vector<int> label5label;  // semantickitti的标签类别

    int check_element_in_vector(const long long element, const std::vector<long long>& vec_check) {
      for (int i = 0; i < vec_check.size(); ++i)
        if (element == vec_check[i])
          return i;
      return -1;
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr kitti2pcl(std::string fn, std::string fn_label) {
      FILE* fp_label = std::fopen(fn_label.c_str(), "r");
      if (!fp_label) {
        std::perror("File opening failed");
      }
      std::fseek(fp_label, 0L, SEEK_END); // 把指针移动到文件结尾
      std::rewind(fp_label); // 把指针移动到开头
      FILE* fp = std::fopen(fn.c_str(), "r");
      if (!fp) {
        std::perror("File opening failed");
      }
      std::fseek(fp, 0L, SEEK_END);
      size_t sz = std::ftell(fp); // 当前位置
      std::rewind(fp);
      int n_hits = sz / (sizeof(float) * 4);
      pcl::PointCloud<pcl::PointXYZL>::Ptr pc(new pcl::PointCloud<pcl::PointXYZL>);
      for (int i = 0; i < n_hits; i++) {
        pcl::PointXYZL point;
        float intensity;
        std::uint32_t label_int32;
        if (fread(&point.x, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.y, sizeof(float), 1, fp) == 0) break;
        if (fread(&point.z, sizeof(float), 1, fp) == 0) break;
        if (fread(&intensity, sizeof(float), 1, fp) == 0) break;
        if (fread(&label_int32, sizeof(std::uint32_t), 1, fp_label) == 0) break;

        int label_int = label_int32 & 0xFFFF;
        // label中的标签都换成1-34表示。总共35个标签，0代表空闲。
        std::vector<int>::iterator ite = find(label2label.begin(), label2label.end(), label_int);
        int num_19 = std::distance(std::begin(label2label), ite);

        // label_int = label3label[num_19];  // 34个标签转换为19个标签的数值
        // ite = find(label4label.begin(), label4label.end(), label_int);
        // point.label = std::distance(std::begin(label4label), ite)+1;

        point.label = label5label[num_19] +1;

        pc->push_back(point);
      }
      std::fclose(fp);
      std::fclose(fp_label);
      return pc;
    }
};
