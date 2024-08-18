/*******************************************************
 * Copyright (C) 2020, RAM-LAB, Hong Kong University of Science and Technology
 *
 * This file is part of M-LOAM (https://ram-lab.com/file/jjiao/m-loam).
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Jianhao JIAO (jiaojh1994@gmail.com)
 *******************************************************/

// Usage: rosrun mloam test_generate_full_map 
//            -save_path=xx/pose_graph/ -num_laser=1 -save_interval=1

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <iostream>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <iomanip>

#include <pcl/common/common.h> 
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include "save_statistics.hpp"
#include "common/common.hpp"
#include "common/gps_tools.hpp"
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/utility.h"
#include "utility/visualization.h"
#include "utility/cloud_visualizer.h"
#include "mloam_pcl/point_with_time.hpp"

using namespace std;

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(data_source, "bag", "the data source: bag or bag");
DEFINE_string(output_path, "", "the path ouf saving results");

Estimator estimator;

SaveStatistics save_statistics;

std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_buf;
std::mutex m_buf;

nav_msgs::Path laser_gt_path;
Pose pose_world_ref_ini;

common::gpsTools gps_tools;
nav_msgs::Path gps_path;
ros::Publisher pub_gps_odom, pub_gps_path, pub_gps;
ros::Publisher pub_laser_gt_odom, pub_laser_gt_path;

bool b_pause = false;

int frame_drop_cnt = 0;

void dataProcessCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    m_buf.lock();
    cloud_buf.push(cloud_msg);
    m_buf.unlock();
}

void gtCallback(const nav_msgs::OdometryConstPtr &gt_odom_msg)
{
    Pose pose_world_gt(*gt_odom_msg);
    Pose pose_world_base_world_gt(Eigen::Quaterniond(1, 0, 0, 0),
                                  Eigen::Vector3d(0, 0, 0));
    Pose pose_world_base_gt(pose_world_base_world_gt * pose_world_gt);

    if (laser_gt_path.poses.size() == 0) pose_world_ref_ini = pose_world_base_gt;
    Pose pose_ref_ini_cur(pose_world_ref_ini.inverse() * pose_world_base_gt);

    nav_msgs::Odometry laser_gt_odom;
    laser_gt_odom.header.frame_id = "/world";
    laser_gt_odom.child_frame_id = "/gt";
    laser_gt_odom.header.stamp = gt_odom_msg->header.stamp;
    laser_gt_odom.pose.pose.orientation.x = pose_ref_ini_cur.q_.x();
    laser_gt_odom.pose.pose.orientation.y = pose_ref_ini_cur.q_.y();
    laser_gt_odom.pose.pose.orientation.z = pose_ref_ini_cur.q_.z();
    laser_gt_odom.pose.pose.orientation.w = pose_ref_ini_cur.q_.w();
    laser_gt_odom.pose.pose.position.x = pose_ref_ini_cur.t_(0);
    laser_gt_odom.pose.pose.position.y = pose_ref_ini_cur.t_(1);
    laser_gt_odom.pose.pose.position.z = pose_ref_ini_cur.t_(2);
    publishTF(laser_gt_odom);

    geometry_msgs::PoseStamped laser_gt_pose;
    laser_gt_pose.header.frame_id = "/world";
    laser_gt_pose.header.stamp = gt_odom_msg->header.stamp;
    laser_gt_pose.pose = laser_gt_odom.pose.pose;
    laser_gt_path.header = laser_gt_pose.header;
    laser_gt_path.poses.push_back(laser_gt_pose);
    pub_laser_gt_path.publish(laser_gt_path);
}

void gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msgs)
{
    gps_tools.updateGPSpose(*gps_msgs);
    
    nav_msgs::Odometry gps_odom;
    gps_odom.header.frame_id = "/world";
    gps_odom.child_frame_id = "/gps";
    gps_odom.header.stamp = gps_msgs->header.stamp;
    gps_odom.pose.pose.orientation.x = 0;
    gps_odom.pose.pose.orientation.y = 0;
    gps_odom.pose.pose.orientation.z = 0;
    gps_odom.pose.pose.orientation.w = 1;
    gps_odom.pose.pose.position.x = gps_tools.gps_pos_.x();
    gps_odom.pose.pose.position.y = gps_tools.gps_pos_.y();
    gps_odom.pose.pose.position.z = gps_tools.gps_pos_.z();
    for (size_t i = 0; i < 36; i++) gps_odom.pose.covariance[i] = gps_tools.gps_cur_cov_[i];
    pub_gps_odom.publish(gps_odom);
    publishTF(gps_odom);

    geometry_msgs::PoseStamped gps_pose;
    gps_pose.header = gps_odom.header;
    gps_pose.pose = gps_odom.pose.pose;
    gps_path.header = gps_pose.header;
    gps_path.poses.push_back(gps_pose);
    pub_gps_path.publish(gps_path);
}

void pauseCallback(std_msgs::StringConstPtr msg)
{
    printf("%s\n", msg->data.c_str());
    b_pause = !b_pause;
}

pcl::PointCloud<pcl::PointXYZ> getCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> laser_cloud;
    pcl::fromROSMsg(*cloud_msg, laser_cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laser_cloud, laser_cloud, indices);
    return laser_cloud;
}

void sync_process()
{
    while (1)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ> > v_laser_cloud(NUM_OF_LASER);
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!cloud_buf.empty())
        {
            time = cloud_buf.front()->header.stamp.toSec();
            header = cloud_buf.front()->header;
            v_laser_cloud[0] = getCloudFromMsg(cloud_buf.front());
            printf("size of finding laser_cloud: %lu\n", v_laser_cloud[0].size());
            cloud_buf.pop();
        }
        while (!cloud_buf.empty())
        {
            frame_drop_cnt++;
            cloud_buf.pop();
            std::cout << common::GREEN << "drop lidar frame in odometry for real time performance"
                      << common::RESET << std::endl;
        }
        m_buf.unlock();

        bool empty_check = false;
        if (v_laser_cloud[0].size() == 0) empty_check = true;

        if (!empty_check) estimator.inputCloud(time, v_laser_cloud);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

int main(int argc, char **argv)
{
    if (argc < 5)
    {
        printf("please intput: rosrun mloam mloam_node_rv_kitti -help\n");
        return 1;
    }
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "mloam_node_rv_kitti");
    ros::NodeHandle nh("~");

    // ******************************************
    printf("config_file: %s\n", FLAGS_config_file.c_str());
    readParameters(FLAGS_config_file);
    estimator.setParameter();
    registerPub(nh);

    MLOAM_RESULT_SAVE = FLAGS_result_save;
    OUTPUT_FOLDER = FLAGS_output_path;
    MLOAM_ODOM_PATH = OUTPUT_FOLDER + "traj/stamped_mloam_odom_estimate_" + to_string(ODOM_GF_RATIO) + ".txt";
    MLOAM_GT_PATH = OUTPUT_FOLDER + "traj/stamped_groundtruth.txt";
    MLOAM_GPS_PATH = OUTPUT_FOLDER + "traj/stamped_gps.txt";
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "others/extrinsic_parameter.txt";
    EX_CALIB_EIG_PATH = OUTPUT_FOLDER + "others/calib_eig.txt";
    printf("save result (0/1): %d\n", MLOAM_RESULT_SAVE);
    ROS_WARN("waiting for cloud...");

    string data_source = FLAGS_data_source;
    ros::Subscriber sub_pause = nh.subscribe<std_msgs::String>("/mloam_pause", 5, pauseCallback);

    // ******************************************
    // use bag as the data source
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(CLOUD_TOPIC[0], 5, dataProcessCallback);
    ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("/novatel718d/pos", 10, gpsCallback);
    ros::Subscriber sub_gt = nh.subscribe<nav_msgs::Odometry>("/base_pose_gt", 10, gtCallback);
    pub_laser_gt_path = nh.advertise<nav_msgs::Path>("/laser_gt_path", 10);
    pub_gps_odom = nh.advertise<nav_msgs::Odometry>("/gps/odom", 10);
    pub_gps_path = nh.advertise<nav_msgs::Path>("/gps/path", 10);

    std::thread sync_thread(sync_process);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << common::YELLOW << "odometry drop frame: " << frame_drop_cnt << common::RESET << std::endl;
    if (MLOAM_RESULT_SAVE)
    {
        std::cout << common::RED << "saving odometry results" << common::RESET << std::endl;
        save_statistics.saveSensorPath(MLOAM_GT_PATH, laser_gt_path);
        save_statistics.saveSensorPath(MLOAM_GPS_PATH, gps_path);
        save_statistics.saveOdomStatistics(EX_CALIB_EIG_PATH, EX_CALIB_RESULT_PATH, MLOAM_ODOM_PATH, estimator);
        save_statistics.saveOdomTimeStatistics(OUTPUT_FOLDER + "time/time_mloam_odometry_" + std::to_string(ODOM_GF_RATIO) + ".txt", estimator);
    }
    sync_thread.join();
    return 0;
}

//
