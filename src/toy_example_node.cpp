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
    ros::init(argc, argv, "toy_example_node");
    ros::NodeHandle nh("~");
    
    std::string map_topic_csm("/semantic_csm");
    std::string map_topic("/see_csom");
    std::string var_topic_csm("/semantic_csm_variance");
    std::string var_topic("/see_csom_variance");
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

    /////////////////////// Semantic CSM //////////////////////
    see_csom::SemanticBKIOctoMap map_csm(resolution, 1, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
    ros::Time start = ros::Time::now();
    for (int scan_id = 1; scan_id <= scan_num; ++scan_id) {
        see_csom::PCLPointCloud cloud;
        see_csom::point3f origin;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, cloud);
        map_csm.insert_pointcloud_test(cloud, origin, resolution, free_resolution, max_range);
        ROS_INFO_STREAM("Scan " << scan_id << " done");
    }
    ros::Time end = ros::Time::now();
    ROS_INFO_STREAM("Semantic CSM finished in " << (end - start).toSec() << "s");

    /////////////////////// Publish Map //////////////////////
    float max_var = std::numeric_limits<float>::min();
    float min_var = std::numeric_limits<float>::max(); 
    see_csom::MarkerArrayPub m_pub_csm(nh, map_topic_csm, resolution);
    for (auto it = map_csm.begin_leaf(); it != map_csm.end_leaf(); ++it) {
        if (it.get_node().get_state() == see_csom::State::OCCUPIED) {
            see_csom::point3f p = it.get_loc();
            int semantics = it.get_node().get_semantics();
            m_pub_csm.insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), semantics, 0);
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);
            if (vars[semantics] > max_var)
		          max_var = vars[semantics];
		        if (vars[semantics] < min_var)
		          min_var = vars[semantics];
        }
    }
    m_pub_csm.publish();
    std::cout << "max_var: " << max_var << std::endl;
    std::cout << "min_var: " << min_var << std::endl;
    
    /////////////////////// Variance Map //////////////////////
    see_csom::MarkerArrayPub v_pub_csm(nh, var_topic_csm, resolution);
    for (auto it = map_csm.begin_leaf(); it != map_csm.end_leaf(); ++it) {
        if (it.get_node().get_state() == see_csom::State::OCCUPIED) {
            see_csom::point3f p = it.get_loc();
            int semantics = it.get_node().get_semantics();
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);
            v_pub_csm.insert_point3d_variance(p.x(), p.y(), p.z(), min_var, max_var, it.get_size(), vars[semantics]);
        }
    }
    v_pub_csm.publish();

    
    /////////////////////// SEE-CSOM //////////////////////
    see_csom::SemanticBKIOctoMap map(resolution, block_depth, num_class, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
    start = ros::Time::now();
    for (int scan_id = 1; scan_id <= scan_num; ++scan_id) {
        see_csom::PCLPointCloud cloud;
        see_csom::point3f origin;
        std::string filename(dir + "/" + prefix + "_" + std::to_string(scan_id) + ".pcd");
        load_pcd(filename, origin, cloud);
        map.insert_pointcloud_test(cloud, origin, resolution, free_resolution, max_range);
        ROS_INFO_STREAM("Scan " << scan_id << " done");
    }
    end = ros::Time::now();
    ROS_INFO_STREAM("SEE-CSOM finished in " << (end - start).toSec() << "s");
 
    
    /////////////////////// Publish Map //////////////////////
    max_var = std::numeric_limits<float>::min();
    min_var = std::numeric_limits<float>::max(); 
    see_csom::MarkerArrayPub m_pub(nh, map_topic, resolution);
    for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
        if (it.get_node().get_state() == see_csom::State::OCCUPIED) {
            see_csom::point3f p = it.get_loc();
            int semantics = it.get_node().get_semantics();
            m_pub.insert_point3d_semantics(p.x(), p.y(), p.z(), it.get_size(), semantics, 0);
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);
            if (vars[semantics] > max_var)
		          max_var = vars[semantics];
		        if (vars[semantics] < min_var)
		          min_var = vars[semantics];
        }
    }
    m_pub.publish();
    std::cout << "max_var: " << max_var << std::endl;
    std::cout << "min_var: " << min_var << std::endl;

    /////////////////////// Variance Map //////////////////////
    see_csom::MarkerArrayPub v_pub(nh, var_topic, resolution);
    for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
        if (it.get_node().get_state() == see_csom::State::OCCUPIED) {
            see_csom::point3f p = it.get_loc();
            int semantics = it.get_node().get_semantics();
            std::vector<float> vars(num_class);
            it.get_node().get_vars(vars);
            v_pub.insert_point3d_variance(p.x(), p.y(), p.z(), min_var, max_var, it.get_size(), vars[semantics]);
        }
    }
    v_pub.publish();

    ros::spin();

    return 0;
}
