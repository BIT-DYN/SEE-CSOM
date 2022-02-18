#include <string>
#include <iostream>
#include <ros/ros.h>
#include "bkioctomap.h"
#include "markerarray_pub.h"

void load_pcd(std::string filename, see_csom::point3f &origin, see_csom::PCLPointCloud &cloud) {
    pcl::PCLPointCloud2 cloud2;
    Eigen::Vector4f _origin;
    Eigen::Quaternionf orientaion;
    pcl::io::loadPCDFile(filename, cloud2, _origin, orientaion);
    pcl::fromPCLPointCloud2(cloud2, cloud);
    origin.x() = _origin[0];
    origin.y() = _origin[1];
    origin.z() = _origin[2];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "stanford_node");
    ros::NodeHandle nh("~");
    
    std::string map_topic("/see_csom");
    std::string dir;
    std::string prefix;
    int block_depth = 4;
    double sf2 = 1.0;
    double ell = 1.0;
    float prior = 1.0f;
    float var_thresh = 1.0f;
    double free_thresh = 0.3;
    double occupied_thresh = 0.7;
    double resolution = 0.1;
    int num_class = 2;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    int scan_num = 0;
    double max_range = -1;

    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("prefix", prefix, prefix);
    nh.param<int>("block_depth", block_depth, block_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("prior", prior, prior);
    nh.param<float>("var_thresh", var_thresh, var_thresh);
    nh.param<double>("free_thresh", free_thresh, free_thresh);
    nh.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", max_range, max_range);
   
    ROS_INFO_STREAM("Parameters:" << std::endl <<
            "dir: " << dir << std::endl <<
            "prefix: " << prefix << std::endl <<
            "block_depth: " << block_depth << std::endl <<
            "sf2: " << sf2 << std::endl <<
            "ell: " << ell << std::endl <<
            "prior: " << prior << std::endl <<
            "var_thresh: " << var_thresh << std::endl <<
            "free_thresh: " << free_thresh << std::endl <<
            "occupied_thresh: " << occupied_thresh << std::endl <<
            "resolution: " << resolution << std::endl <<
            "num_class: " << num_class << std::endl <<
            "free_resolution: " << free_resolution << std::endl <<
            "ds_resolution: " << ds_resolution << std::endl <<
            "scan_sum: " << scan_num << std::endl <<
            "max_range: " << max_range
            );


    
    ///////////////////// SEE-CSOM //////////////////////
    see_csom::SemanticBKIOctoMap map(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
    ros::Time start = ros::Time::now();
    for (int scan_id = 1; scan_id <= scan_num; ++scan_id) {
        see_csom::PCLPointCloud cloud;
        see_csom::point3f origin;
        std::string filename(dir + "/" +  std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, cloud);
        map.insert_pointcloud_test(cloud, origin, ds_resolution, free_resolution, max_range);
        ROS_INFO_STREAM("Scan " << scan_id << " done");
    }
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("SEE-CSOM finished in " << (end - start).toSec() << "s");
 
    
    /////////////////////// Publish Map //////////////////////
    see_csom::MarkerArrayPub m_pub(nh, map_topic, resolution);
    // float x_max = -100.0;
    // float y_max = -100.0;
    // float z_max = -100.0;
    // float x_min = 100.0;
    // float y_min = 100.0;
    // float z_min = 100.0;
    for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
        if (it.get_node().get_state() == see_csom::State::OCCUPIED) {
            see_csom::point3f p = it.get_loc();

            //  if (p.x() > x_max)
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
            
            int semantics = it.get_node().get_semantics();
            m_pub.insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), semantics, 6);
        }
    }
    // std::cout << "x: " << x_max << "    y: "<<y_max<< "  z: "<<z_max <<std::endl;
    // std::cout << "x: " << x_min << "    y: "<<y_min<< "  z: "<<z_min <<std::endl;

    // 会议室
    // 真值最大和最小
    // x: 19.65    y: 6.35  z: 2.8
    // x: 14.35    y: 0.7  z: 0
    // BKI的最大最小
    // x: 19.825    y: 6.525  z: 2.925
    // x: 14.275    y: 0.575  z: -0.125
    // CSM的最大最小
    // x: 19.65    y: 6.35  z: 2.75
    // x: 14.35    y: 0.7  z: 0
    // DYN的最大最小，都需要减去0.025
    // x: 19.725    y: 6.425  z: 2.825
    // x: 14.375    y: 0.675  z: -0.025
    //选取的方框为，直接平移吧，把BKI和DYN向小的地方平移0.025(找-0.025出真值的状态)
    // x: 14.0 ~ 20.0      120
    // y: 0.5 ~ 6.5      120
    // z: -0.2 ~ 3          64

    // 休息室
    // 真值最大和最小
    // x: 10.95    y: -1.25  z: 3.05
    // x: 2.65    y: -8.6  z: 0
    // BKI的最大最小
    // x: 11.025    y: -1.075  z: 3.225
    // x: 2.475   y: -8.725  z: -0.125
    // CSM的最大最小
    // x: 10.9    y: -1.25  z: 3.05
    // x:2.65    y: -8.55  z: 0
    // DYN的最大最小，都需要减去0.025
    // x: 10.925    y: -1.175  z: 3.125
    // x: 2.575    y: -8.625  z: -0.025
    //选取的方框为，直接平移吧，把BKI和DYN向小的地方平移0.025(找-0.025出真值的状态)
    // x: 2.0 ~ 12.0      200
    // y: -9.0 ~ -1.0      160
    // z: -0.5 ~ 3.5          60


    map.print_vaule();

    m_pub.publish();
    ros::spin();

    return 0;
}
