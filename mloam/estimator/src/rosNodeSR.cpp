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
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <iostream>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <iomanip>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include "save_statistics.hpp"
#include "common/common.hpp"
#include "common/random_generator.hpp"
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/utility.h"
#include "utility/visualization.h"
#include "utility/cloud_visualizer.h"

using namespace std;

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");
DEFINE_bool(inject_meas_noise, false, "inject measurement noise on the raw data");
DEFINE_int32(mc_trial, 0, "monte carlo trial number");

Estimator estimator;
SaveStatistics save_statistics;

// message buffer
std::vector<std::queue<sensor_msgs::PointCloud2ConstPtr>> all_cloud_buf(2);
std::mutex m_buf;

// laser path groundtruth
nav_msgs::Path laser_gt_path;
ros::Publisher pub_laser_gt_path;
Pose pose_world_ref_ini;

int frame_drop_cnt = 0;

common::RandomGeneratorFloat<float> rgi;

void dataProcessCallback(const sensor_msgs::PointCloud2ConstPtr &cloud0_msg,
                         const sensor_msgs::PointCloud2ConstPtr &cloud1_msg)
{
    m_buf.lock();
    all_cloud_buf[0].push(cloud0_msg);
    all_cloud_buf[1].push(cloud1_msg);
    m_buf.unlock();
}

pcl::PointCloud<pcl::PointXYZ> getCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ> laser_cloud;
    pcl::fromROSMsg(*cloud_msg, laser_cloud);
    // roiCloudFilter(laser_cloud, ROI_RANGE);
    return laser_cloud;
}

void sync_process()
{
    while (1)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>> v_laser_cloud(NUM_OF_LASER);
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!all_cloud_buf[0].empty() && !all_cloud_buf[1].empty())
        {
            time = all_cloud_buf[0].front()->header.stamp.toSec();
            header = all_cloud_buf[0].front()->header;
            stringstream ss;
            for (size_t i = 0; i < NUM_OF_LASER; i++)
            {
                v_laser_cloud[i] = getCloudFromMsg(all_cloud_buf[i].front());
                ss << v_laser_cloud[i].size() << " ";
            }
            for (size_t i = 0; i < all_cloud_buf.size(); i++)
                all_cloud_buf[i].pop();
            printf("size of finding laser_cloud: %s\n", ss.str().c_str());
        }
        while (!all_cloud_buf[0].empty())
        {
            frame_drop_cnt++;
            for (size_t i = 0; i < all_cloud_buf.size(); i++)
            {
                if (!all_cloud_buf[i].empty())
                {
                    all_cloud_buf[i].pop();
                }
            }
            std::cout << common::GREEN << "drop lidar frame in odometry for real time performance"
                      << common::RESET << std::endl;
        }
        m_buf.unlock();

        bool empty_check = false;
        for (size_t i = 0; i < NUM_OF_LASER; i++)
            empty_check = (v_laser_cloud[i].size() == 0);

        if (!empty_check)
        {
            // if (FLAGS_inject_meas_noise)
            // {
            //     std::cout << "Injecting measurement noise" << std::endl;
            //     for (size_t i = 0; i < NUM_OF_LASER; i++)
            //     {
            //         for (pcl::PointXYZ &point : v_laser_cloud[i])
            //         {
            //             // add measurement noise
            //             float *n_xyz = rgi.geneRandUniformArray(0, 0.05, 3);
            //             point.x += n_xyz[0];
            //             point.y += n_xyz[1];
            //             point.z += n_xyz[2];
            //             delete n_xyz;
            //         }
            //     }
            // }
            estimator.inputCloud(time, v_laser_cloud);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data != 0)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
}

void odom_gt_callback(const nav_msgs::Odometry &odom_msg)
{
    Pose pose_world_base(odom_msg);
    Pose pose_base_ref(Eigen::Quaterniond(0.976, -0.216, 0, 0), Eigen::Vector3d(0, 0.266, 0.734));
    Pose pose_world_ref(pose_world_base * pose_base_ref);
    if (laser_gt_path.poses.size() == 0) pose_world_ref_ini = pose_world_ref;
    Pose pose_ref_ini_cur(pose_world_ref_ini.inverse() * pose_world_ref);

    nav_msgs::Odometry laser_odom;
    laser_odom.header = odom_msg.header;
    laser_odom.header.frame_id = "/world";
    laser_odom.child_frame_id = "/gt";
    laser_odom.pose.pose.orientation.x = pose_ref_ini_cur.q_.x();
    laser_odom.pose.pose.orientation.y = pose_ref_ini_cur.q_.y();
    laser_odom.pose.pose.orientation.z = pose_ref_ini_cur.q_.z();
    laser_odom.pose.pose.orientation.w = pose_ref_ini_cur.q_.w();
    laser_odom.pose.pose.position.x = pose_ref_ini_cur.t_(0);
    laser_odom.pose.pose.position.y = pose_ref_ini_cur.t_(1);
    laser_odom.pose.pose.position.z = pose_ref_ini_cur.t_(2);
    publishTF(laser_odom);

    geometry_msgs::PoseStamped laser_pose;
    laser_pose.header = odom_msg.header;
    laser_pose.header.frame_id = "/world";
    laser_pose.pose = laser_odom.pose.pose;
    laser_gt_path.header = laser_pose.header;
    laser_gt_path.poses.push_back(laser_pose);
    pub_laser_gt_path.publish(laser_gt_path);
}

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        printf("please intput: rosrun mloam mloam_node_sr -help\n");
        return 1;
    }
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "mloam_node_sr");
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
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "others/extrinsic_parameter.txt";
    EX_CALIB_EIG_PATH = OUTPUT_FOLDER + "others/calib_eig.txt";
    printf("save result (0/1): %d\n", MLOAM_RESULT_SAVE);
    std::cout << common::YELLOW << "waiting for cloud..." << common::RESET << std::endl;

    // ******************************************
    typedef sensor_msgs::PointCloud2 LidarMsgType;
    typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType> LidarSyncPolicy;
    typedef message_filters::Subscriber<LidarMsgType> LidarSubType;

    std::vector<LidarSubType *> sub_lidar(2);
    NUM_OF_LASER = NUM_OF_LASER < 2 ? NUM_OF_LASER : 2;
    for (size_t i = 0; i < NUM_OF_LASER; i++) sub_lidar[i] = new LidarSubType(nh, CLOUD_TOPIC[i], 1);
    for (size_t i = NUM_OF_LASER; i < 2; i++) sub_lidar[i] = new LidarSubType(nh, CLOUD_TOPIC[0], 1);
    message_filters::Synchronizer<LidarSyncPolicy> *lidar_synchronizer =
        new message_filters::Synchronizer<LidarSyncPolicy>(
            LidarSyncPolicy(10), *sub_lidar[0], *sub_lidar[1]);
    lidar_synchronizer->registerCallback(boost::bind(&dataProcessCallback, _1, _2));

    ros::Subscriber sub_restart = nh.subscribe("/mlod_restart", 5, restart_callback);
    ros::Subscriber sub_odom_gt = nh.subscribe("/base_odom_gt", 5, odom_gt_callback);
    pub_laser_gt_path = nh.advertise<nav_msgs::Path>("/laser_gt_path", 5);

    std::thread sync_thread(sync_process);
    std::thread cloud_visualizer_thread;
    if (PCL_VIEWER)
    {
        cloud_visualizer_thread = std::thread(&PlaneNormalVisualizer::Spin, &estimator.plane_normal_vis_);
    }
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
        save_statistics.saveOdomStatistics(EX_CALIB_EIG_PATH, EX_CALIB_RESULT_PATH, MLOAM_ODOM_PATH, estimator);
        save_statistics.saveOdomTimeStatistics(OUTPUT_FOLDER + "time/time_mloam_odometry_" + std::to_string(ODOM_GF_RATIO) + ".txt", estimator);
    }

    cloud_visualizer_thread.join();
    sync_thread.join();
    return 0;
}




//
