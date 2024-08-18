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

#define MAX_BUF_LENGTH 5

using namespace std;

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(data_source, "bag", "the data source: bag or bag");
DEFINE_string(data_path, "", "the data path");
DEFINE_string(output_path, "", "the path ouf saving results");
DEFINE_int32(start_idx, 0, "the start idx of the data");
DEFINE_int32(end_idx, 0, "the end idx of the data");
DEFINE_int32(delta_idx, 1, "the delta idx of reading the data");
DEFINE_bool(time_now, true, "use current time or data time");

Estimator estimator;

SaveStatistics save_statistics;

std::vector<std::queue<sensor_msgs::PointCloud2ConstPtr> > all_cloud_buf(5);
std::mutex m_buf;

nav_msgs::Path laser_gt_path;
Pose pose_world_ref_ini;

common::gpsTools gps_tools;
nav_msgs::Path gps_path;
ros::Publisher pub_gps_odom, pub_gps_path, pub_gps;
ros::Publisher pub_laser_gt_odom, pub_laser_gt_path;

bool b_pause = false;

int frame_drop_cnt = 0;

void dataProcessCallback(const sensor_msgs::PointCloud2ConstPtr &cloud0_msg,
                         const sensor_msgs::PointCloud2ConstPtr &cloud1_msg,
                         const sensor_msgs::PointCloud2ConstPtr &cloud2_msg,
                         const sensor_msgs::PointCloud2ConstPtr &cloud3_msg,
                         const sensor_msgs::PointCloud2ConstPtr &cloud4_msg)
{
    m_buf.lock();
    all_cloud_buf[0].push(cloud0_msg);
    all_cloud_buf[1].push(cloud1_msg);
    all_cloud_buf[2].push(cloud2_msg);
    all_cloud_buf[3].push(cloud3_msg);
    all_cloud_buf[4].push(cloud4_msg);
    m_buf.unlock();
}

void gtCallback(const nav_msgs::OdometryConstPtr &gt_odom_msg)
{
    Pose pose_world_stereo_gt(*gt_odom_msg);
    Pose pose_world_base_world_stereo(Eigen::Quaterniond(0.99977, 0.0026139, -0.021008, 0.003888),
                                      Eigen::Vector3d(0.61413, -0.3347, -0.24461));
    Pose pose_world_base_gt(pose_world_base_world_stereo * pose_world_stereo_gt);

    if (laser_gt_path.poses.size() == 0)
        pose_world_ref_ini = pose_world_base_gt;
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

pcl::PointCloud<pcl::PointXYZIWithTime> getCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZIWithTime> laser_cloud;
    pcl::fromROSMsg(*cloud_msg, laser_cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laser_cloud, laser_cloud, indices);
    return laser_cloud;
}

void sync_process()
{
    while (1)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZIWithTime> > v_laser_cloud(NUM_OF_LASER);
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!all_cloud_buf[0].empty() && !all_cloud_buf[1].empty() && !all_cloud_buf[2].empty() 
         && !all_cloud_buf[3].empty() && !all_cloud_buf[4].empty())
        {
            time = all_cloud_buf[0].front()->header.stamp.toSec();
            header = all_cloud_buf[0].front()->header;
            stringstream ss;
            for (size_t i = 0; i < NUM_OF_LASER; i++) 
            {
                v_laser_cloud[i] = getCloudFromMsg(all_cloud_buf[i].front());
                ss << v_laser_cloud[i].size() << " ";
            }
            for (size_t i = 0; i < all_cloud_buf.size(); i++) all_cloud_buf[i].pop();
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
            if (v_laser_cloud[i].size() == 0) empty_check = true;

        if (!empty_check) estimator.inputCloud(time, v_laser_cloud);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

int main(int argc, char **argv)
{
    if (argc < 5)
    {
        printf("please intput: rosrun mloam mloam_node_rv -help\n");
        return 1;
    }
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "mloam_node_rv");
    ros::NodeHandle nh("~");

    // ******************************************
    printf("config_file: %s\n", FLAGS_config_file.c_str());
    readParameters(FLAGS_config_file);
    estimator.setParameter();
    registerPub(nh);

    MLOAM_RESULT_SAVE = FLAGS_result_save;
    OUTPUT_FOLDER = FLAGS_output_path;
    MLOAM_ODOM_PATH = OUTPUT_FOLDER + "traj/stamped_mloam_odom_estimate_" + to_string(ODOM_GF_RATIO) + ".txt";
    MLOAM_GPS_PATH = OUTPUT_FOLDER + "traj/stamped_gps.txt";
    MLOAM_GT_PATH = OUTPUT_FOLDER + "traj/stamped_groundtruth.txt";
    EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "others/extrinsic_parameter.txt";
    EX_CALIB_EIG_PATH = OUTPUT_FOLDER + "others/calib_eig.txt";
    printf("save result (0/1): %d\n", MLOAM_RESULT_SAVE);
    size_t START_IDX = FLAGS_start_idx;
    size_t END_IDX = FLAGS_end_idx;
    size_t DELTA_IDX = FLAGS_delta_idx;
    bool TIME_NOW = FLAGS_time_now;
    std::cout << common::YELLOW << "waiting for cloud..." << common::RESET << std::endl;

    string data_source = FLAGS_data_source;
    ros::Subscriber sub_pause = nh.subscribe<std_msgs::String>("/mloam_pause", 5, pauseCallback);

    // ******************************************
    // use bag as the data source
    if (!data_source.compare("bag"))
    {
        typedef sensor_msgs::PointCloud2 LidarMsgType;
        typedef message_filters::sync_policies::ApproximateTime<LidarMsgType, LidarMsgType, LidarMsgType, LidarMsgType, LidarMsgType> LidarSyncPolicy;
        typedef message_filters::Subscriber<LidarMsgType> LidarSubType;

        std::vector<LidarSubType *> sub_lidar(5);
        NUM_OF_LASER = NUM_OF_LASER < 5 ? NUM_OF_LASER : 5;
        for (size_t i = 0; i < NUM_OF_LASER; i++) sub_lidar[i] = new LidarSubType(nh, CLOUD_TOPIC[i], 5);
        for (size_t i = NUM_OF_LASER; i < 5; i++) sub_lidar[i] = new LidarSubType(nh, CLOUD_TOPIC[0], 5);
        message_filters::Synchronizer<LidarSyncPolicy> *lidar_synchronizer =
            new message_filters::Synchronizer<LidarSyncPolicy>(
                LidarSyncPolicy(10), *sub_lidar[0], *sub_lidar[1], *sub_lidar[2], *sub_lidar[3], *sub_lidar[4]);
        lidar_synchronizer->registerCallback(boost::bind(&dataProcessCallback, _1, _2, _3, _4, _5));

        ros::Subscriber sub_gt = nh.subscribe<nav_msgs::Odometry>("/base_pose_gt", 10, gtCallback);
        ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("/novatel718d/pos", 10, gpsCallback);
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

        // std::cout << common::YELLOW << "odometry drop frame: " << frame_drop_cnt << common::RESET << std::endl;
        LOG(INFO) << "odometry drop frame: " << frame_drop_cnt;
        if (MLOAM_RESULT_SAVE)
        {
            std::cout << common::RED << "saving odometry results" << common::RESET << std::endl;
            save_statistics.saveSensorPath(MLOAM_GT_PATH, laser_gt_path);
            save_statistics.saveSensorPath(MLOAM_GPS_PATH, gps_path);
            save_statistics.saveOdomStatistics(EX_CALIB_EIG_PATH, EX_CALIB_RESULT_PATH, MLOAM_ODOM_PATH, estimator);
            save_statistics.saveOdomTimeStatistics(OUTPUT_FOLDER + "time/time_mloam_odometry_" + std::to_string(ODOM_GF_RATIO) + ".txt", estimator);
        }
        sync_thread.join();
    } else
    // use pcd as the data source
    if (!data_source.compare("pcd"))
    {
        string data_path = FLAGS_data_path;
        printf("read sequence from: %s\n", FLAGS_data_path);

        std::vector<ros::Publisher> pub_laser_cloud_list(NUM_OF_LASER);
        for (size_t i = 0; i < NUM_OF_LASER; i++) pub_laser_cloud_list[i] = nh.advertise<sensor_msgs::PointCloud2>(CLOUD_TOPIC[i], 10);
        pub_laser_gt_odom = nh.advertise<nav_msgs::Odometry>("/current_odom", 10);
        pub_laser_gt_path = nh.advertise<nav_msgs::Path>("/laser_gt_path", 10);
        pub_gps_odom = nh.advertise<sensor_msgs::NavSatFix>("/gps/odom", 10);
        pub_gps_path = nh.advertise<nav_msgs::Path>("/gps/path", 10);

        // *************************************
        // read cloud list
        FILE *file;
        double cloud_time;
        vector<double> cloud_time_list;
        {
            file = std::fopen((data_path + "cloud_0/timestamps.txt").c_str(), "r");
            if (!file)
            {
                printf("cannot find file: %stimes.txt\n", data_path.c_str());
                ROS_BREAK();
                return 0;
            }
            while (fscanf(file, "%lf", &cloud_time) != EOF)
            {
                cloud_time_list.push_back(cloud_time);
            }
            std::fclose(file);    
        }  

        // *************************************
        // read data
        double base_time = ros::Time::now().toSec();
        for (size_t i = START_IDX; i < std::min(END_IDX, cloud_time_list.size()); i+=DELTA_IDX)
        {	
            if (ros::ok())
            {
                double cloud_time;
                if (TIME_NOW)
                    cloud_time = base_time + cloud_time_list[i] - cloud_time_list[START_IDX];
                else
                    cloud_time = cloud_time_list[i];
                printf("process data: %d\n", i);
                stringstream ss;
                ss << setfill('0') << setw(6) << i;

                // load cloud
                printf("size of finding cloud: ");
                std::vector<pcl::PointCloud<pcl::PointXYZ> > laser_cloud_list(NUM_OF_LASER);
                for (size_t j = 0; j < NUM_OF_LASER; j++)
                {
                    stringstream ss_file;
                    ss_file << "cloud_" << j << "/data/" << ss.str() << ".pcd";
                    string cloud_path = data_path + ss_file.str();
                    //printf("%lu  %f \n", i, cloud_time);
                    // printf("%s\n", cloud_path.c_str());
                    if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_path, laser_cloud_list[j]) == -1)
                    {
                        printf("Couldn't read file %s\n", cloud_path.c_str());
                        ROS_BREAK();
                        return 0;
                    }
                    printf("%lu ", laser_cloud_list[j].size());
                }
                printf("\n");

                // load gps
                FILE *gps_file;
                string gps_file_path = data_path + "gps/data/" + ss.str() + ".txt";
                gps_file = std::fopen(gps_file_path.c_str() , "r");
                if(!gps_file)
                {
                    // printf("cannot find file: %s\n", gps_file_path.c_str());
                    // ROS_BREAK();
                    // return 0;          
                } else
                {
                    double lat, lon, alt;
                    double posx_accuracy, posy_accuracy, posz_accuracy;
                    int navstat, numsats;
                    fscanf(gps_file, "%d %d ", &navstat, &numsats);
                    fscanf(gps_file, "%lf %lf %lf ", &lat, &lon, &alt);
                    fscanf(gps_file, "%lf %lf %lf ", &posx_accuracy, &posy_accuracy, &posz_accuracy);
                    std::fclose(gps_file);

                    sensor_msgs::NavSatFix gps_position;
                    gps_position.header.frame_id = "gps";
                    gps_position.header.stamp = ros::Time(cloud_time);
                    gps_position.status.status = navstat;
                    gps_position.status.service = numsats;
                    gps_position.latitude = lat;
                    gps_position.longitude = lon;
                    gps_position.altitude = alt;
                    gps_position.position_covariance[0] = posx_accuracy;
                    gps_position.position_covariance[4] = posy_accuracy;
                    gps_position.position_covariance[8] = posz_accuracy;
                    pub_gps.publish(gps_position);
                }
                
                // load odom
                FILE *gt_odom_file;
                string gt_odom_file_path = data_path + "gt_odom/data/" + ss.str() + ".txt";
                gt_odom_file = std::fopen(gt_odom_file_path.c_str() , "r");
                if(!gt_odom_file)
                {
                    // printf("cannot find file: %s\n", gt_odom_file_path.c_str());
                    // ROS_BREAK();
                    // return 0;          
                } else
                {
                    double posx, posy, posz;
                    double orix, oriy, oriz, oriw;
                    fscanf(gt_odom_file, "%lf %lf %lf ", &posx, &posy, &posz);
                    fscanf(gt_odom_file, "%lf %lf %lf %lf ", &orix, &oriy, &oriz, &oriw);
                    std::fclose(gt_odom_file);

                    // Eigen::Vector3d t_world_base(posx, posy, posz);
                    // Eigen::Quaterniond q_world_base(oriw, orix, oriy, oriz);
                    // Pose pose_world_base(q_world_base, t_world_base);
                    // Pose pose_base_ref(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
                    // Pose pose_world_ref(pose_world_base * pose_base_ref);
                    
                    // if (laser_gt_path.poses.size() == 0) pose_world_ref_ini = pose_world_ref;
                    // Pose pose_ref_ini_cur(pose_world_ref_ini.inverse() * pose_world_ref);

                    Eigen::Vector3d t_world_stereo_gt(posx, posy, posz);
                    Eigen::Quaterniond q_world_stereo_gt(oriw, orix, oriy, oriz);
                    Pose pose_world_stereo_gt(q_world_stereo_gt, t_world_stereo_gt);
                    Pose pose_world_base_world_stereo(Eigen::Quaterniond(0.99977, 0.0026139, -0.021008, 0.003888),
                                                      Eigen::Vector3d(0.61413, -0.3347, -0.24461));
                    Pose pose_world_base_gt(pose_world_base_world_stereo * pose_world_stereo_gt);

                    if (laser_gt_path.poses.size() == 0)
                        pose_world_ref_ini = pose_world_base_gt;
                    Pose pose_ref_ini_cur(pose_world_ref_ini.inverse() * pose_world_base_gt);

                    nav_msgs::Odometry laser_gt_odom;
                    laser_gt_odom.header.frame_id = "/world";
                    laser_gt_odom.child_frame_id = "/gt";
                    laser_gt_odom.header.stamp = ros::Time(cloud_time);
                    laser_gt_odom.pose.pose.orientation.x = pose_ref_ini_cur.q_.x();
                    laser_gt_odom.pose.pose.orientation.y = pose_ref_ini_cur.q_.y();
                    laser_gt_odom.pose.pose.orientation.z = pose_ref_ini_cur.q_.z();
                    laser_gt_odom.pose.pose.orientation.w = pose_ref_ini_cur.q_.w();
                    laser_gt_odom.pose.pose.position.x = pose_ref_ini_cur.t_(0);
                    laser_gt_odom.pose.pose.position.y = pose_ref_ini_cur.t_(1);
                    laser_gt_odom.pose.pose.position.z = pose_ref_ini_cur.t_(2);
                    pub_laser_gt_odom.publish(laser_gt_odom);
                    publishTF(laser_gt_odom);

                    geometry_msgs::PoseStamped laser_gt_pose;
                    laser_gt_pose.header.frame_id = "/world";
                    laser_gt_pose.header.stamp = ros::Time(cloud_time);
                    laser_gt_pose.pose = laser_gt_odom.pose.pose;
                    laser_gt_path.header = laser_gt_pose.header;
                    laser_gt_path.poses.push_back(laser_gt_pose);
                    pub_laser_gt_path.publish(laser_gt_path);
                }

                estimator.inputCloud(cloud_time, laser_cloud_list);           
                ros::Rate loop_rate(10);
                if (b_pause)
                {
                    while (true)
                    {
                        ros::spinOnce();
                        if ((!b_pause) || (!ros::ok())) break;
                        loop_rate.sleep();
                    }
                } else
                {
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            } else
            {
                break;
            }
        }

        if (MLOAM_RESULT_SAVE)
        {
            std::cout << common::RED << "saving odometry results" << common::RESET << std::endl;
            save_statistics.saveSensorPath(MLOAM_GT_PATH, laser_gt_path);
            save_statistics.saveOdomStatistics(EX_CALIB_EIG_PATH, EX_CALIB_RESULT_PATH, MLOAM_ODOM_PATH, estimator);
            save_statistics.saveOdomTimeStatistics(OUTPUT_FOLDER + "time_odometry" + std::to_string(ODOM_GF_RATIO) + ".txt", estimator);
        }
    }

    return 0;
}

//
