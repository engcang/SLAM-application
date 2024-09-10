/**
 * This file is part of SLICT.
 *
 * Copyright (C) 2020 Thien-Minh Nguyen <thienminh.nguyen at ntu dot edu dot
 * sg>, School of EEE Nanyang Technological Univertsity, Singapore
 *
 * For more information please see <https://britsknguyen.github.io>.
 * or <https://github.com/brytsknguyen/slict>.
 * If you use this code, please cite the respective publications as
 * listed on the above websites.
 *
 * SLICT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SLICT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SLICT.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// Created by Thien-Minh Nguyen on 01/08/22.
//

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <condition_variable>
#include <deque>
#include <thread>

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <cv_bridge/cv_bridge.h>

/* All needed for kdtree of custom point type----------*/
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
/* All needed for kdtree of custom point type----------*/

/* All needed for filter of custom point type----------*/
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/impl/uniform_sampling.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/crop_box.hpp>
/* All needed for filter of custom point type----------*/

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include "tf/transform_broadcaster.h"
#include "slict/globalMapsPublish.h"
#include "std_msgs/Float32.h"

// UFO
#include <ufo/map/code/code_unordered_map.h>
#include <ufo/map/point_cloud.h>
#include <ufo/map/surfel_map.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

// Factor
#include <factor/Point2PlaneDisFactorCT.h>
#include <PreintBase.h>
#include <factor/PreintFactor.h>
#include "PoseLocalParameterization.h"
#include "factor/RelOdomFactor.h"

// Custom for package
#include "utility.h"
#include "slict/FeatureCloud.h"
#include "slict/OptStat.h"
// #include "CloudMatcher.hpp"

#include <chrono>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace std::chrono;


class Estimator
{

private:

    // Node handler
    ros::NodeHandlePtr nh_ptr;
    ros::Time program_start_time;
    bool autoexit = false;

    // Subscribers
    ros::Subscriber data_sub;

    // Service
    ros::ServiceServer global_maps_srv;       // For requesting the global map to be published

    // Synchronized data buffer
    mutex packet_buf_mtx;
    deque<slict::FeatureCloud::ConstPtr> packet_buf;

    bool ALL_INITED  = false;
    int  WINDOW_SIZE = 4;
    int  N_SUB_SEG   = 4;

    Vector3d GRAV = Vector3d(0, 0, 9.82);

    // Sliding window data (prefixed "Sw" )
    struct TimeSegment
    {
        TimeSegment(double start_time_, double final_time_)
            : start_time(start_time_), final_time(final_time_)
        {};

        double dt()
        {
            return (final_time - start_time);
        }

        double start_time;
        double final_time;
    };
    deque<deque<TimeSegment>> SwTimeStep;
    deque<CloudXYZITPtr>      SwCloud;
    deque<CloudXYZITPtr>      SwCloudDsk;
    deque<CloudXYZITPtr>      SwCloudDskDS;
    deque<deque<LidarCoef>>   SwLidarCoef;
    deque<map<int, int>>      SwDepVsAssoc;
    deque<deque<ImuSequence>> SwImuBundle;      // ImuSample defined in utility.h
    deque<deque<ImuProp>>     SwPropState;

    // Surfelized scans
    using ufoSurfelMap = ufo::map::SurfelMap;
    using ufoNode = ufo::map::Node;

    // Check list for adjusting the computation
    map<int, int> DVA;
    int total_lidar_coef;

    // States at the segments' start and final times; ss: segment's start time, sf: segment's final time
    deque<deque<Quaternd>> ssQua, sfQua;
    deque<deque<Vector3d>> ssPos, sfPos;
    deque<deque<Vector3d>> ssVel, sfVel;
    deque<deque<Vector3d>> ssBia, sfBia;
    deque<deque<Vector3d>> ssBig, sfBig;

    // Sensor weight
    double GYR_N = 5.0e-2;
    double GYR_W = 3.0e-3;
    double ACC_N = 6.0e-1;
    double ACC_W = 8.0e-2;

    double lidar_weight = 30;

    double qpprop_weight = 0.1;
    double vel_weight = 1.0;
    double rot_weight = 1.0;

    int last_fixed_knot = 0;
    int first_fixed_knot = 0;

    double leaf_size = 0.1;
    double assoc_spacing = 0.2;
    int surfel_map_depth = 5;
    int surfel_min_point = 5;
    int surfel_min_depth = 0;
    int surfel_query_depth = 3;
    double surfel_intsect_rad = 0.5;
    double surfel_min_plnrty = 0.8;

    // Size of k-nearest neighbourhood for the knn search
    double dis_to_surfel_max = 0.05;
    double score_min = 0.1;
    
    // Lidar downsample rate
    int lidar_ds_rate = 1;

    // Optimization parameters
    double lidar_loss_thres = 1.0;

    // Solver config
    ceres::TrustRegionStrategyType trustRegType;     // LEVENBERG_MARQUARDT, DOGLEG
    ceres::DenseLinearAlgebraLibraryType linAlgbLib; // EIGEN, LAPACK, CUDA
    double max_solve_time = 0.5;
    int max_iterations = 200;

    // Sensors used
    bool fuse_lidar      = true;
    bool fuse_imu        = true;
    
    bool snap_to_0180    = false;
    bool regularize_imu  = true;
    int  fix_mode        = 1;
    double imu_init_time = 0.1;
    int max_outer_iters  = 1;
    int max_lidar_factor = 5000;

    // Map
    CloudPosePtr         KfCloudPose;
    deque<CloudXYZITPtr> KfCloudinB;
    deque<CloudXYZITPtr> KfCloudinW;

    int    ufomap_version = 0;
    mutex  global_map_mtx;
    TicToc tt_ufmupdt;
    CloudXYZITPtr globalMap;
    ufoSurfelMap  surfelMap;

    // Loop closure
    bool loop_en = true;
    int loop_kf_nbr = 5;            // Number of neighbours to check for loop closure
    int loop_time_mindiff = 10;     // Only check for loop when keyframes have this much difference
    struct LoopPrior
    {
        LoopPrior(int prevPoseId_, int currPoseId_, double JKavr_, double IcpFn_, mytf tf_Bp_Bc_)
            : prevPoseId(prevPoseId_), currPoseId(currPoseId_), JKavr(JKavr_), IcpFn(IcpFn_), tf_Bp_Bc(tf_Bp_Bc_) {};

        int prevPoseId = -1;
        int currPoseId = -1;
        double JKavr = -1;
        double IcpFn = -1;
        mytf tf_Bp_Bc;
    };
    deque<LoopPrior> loopPairs;     // Array to store loop priors

    int icpMaxIter = 20;            // Maximum iterations for ICP
    double icpFitnessThres = 0.3;   // Fitness threshold for ICP check
    double histDis = 15.0;          // Maximum correspondence distance for icp
    double lastICPFn = -1;

    int rib_edge = 5;
    double odom_q_noise = 0.1;
    double odom_p_noise = 0.1;
    double loop_weight  = 0.02;

    TicToc tt_loopBA;               // Timer to check the loop and BA time
    struct BAReport
    {
        int turn = -1;
        double pgopt_time = 0;
        int pgopt_iter = 0;
        int factor_relpose = 0;
        int factor_loop = 0;
        double J0 = 0;
        double JK = 0;
        double J0_relpose = 0;
        double JK_relpose = 0;
        double J0_loop = 0;
        double JK_loop = 0;
        double rebuildmap_time = 0;
    };
    BAReport baReport;

    struct KeyframeCand
    {
        KeyframeCand(double start_time_, double end_time_, CloudXYZITPtr kfCloud_)
            : start_time(start_time_), end_time(end_time_), kfCloud(kfCloud_) {};
        double start_time;
        double end_time;
        CloudXYZITPtr kfCloud;
    };
    
    // Publisher for global map.
    ros::Publisher global_map_pub;
    bool publish_map = false;

    // Keyframe params
    double kf_min_dis = 0.5;
    double kf_min_angle = 10;
    double margPerc = 0;
    
    // Publisher for latest keyframe
    ros::Publisher kfcloud_pub;
    ros::Publisher kfpose_pub;

    // Log
    string log_dir = "/home/tmn";
    std::ofstream loop_log_file;

public:
    // Destructor
    ~Estimator() {}

    Estimator(ros::NodeHandlePtr &nh_ptr_) : nh_ptr(nh_ptr_)
    {
        Initialize();
    }

    void Initialize()
    {
        program_start_time = ros::Time::now();

        autoexit = GetBoolParam("/autoexit", false);

        // Maximum number of threads
        printf("Maximum number of threads: %d\n", MAX_THREADS);

        // Window size length
        if (nh_ptr->getParam("/WINDOW_SIZE", WINDOW_SIZE))
            printf("WINDOW_SIZE declared: %d\n", WINDOW_SIZE);
        else
        {
            printf("WINDOW_SIZE not found. Exiting\n");
            exit(-1);
        }

        if (nh_ptr->getParam("/N_SUB_SEG", N_SUB_SEG))
            printf("N_SUB_SEG declared: %d\n", N_SUB_SEG);
        else
        {
            printf("N_SUB_SEG not found. Exiting\n");
            exit(-1);
        }

        // Initialize the states in the sliding window
        ssQua = sfQua = deque<deque<Quaternd>>(WINDOW_SIZE, deque<Quaternd>(N_SUB_SEG, Quaternd::Identity()));
        ssPos = sfPos = deque<deque<Vector3d>>(WINDOW_SIZE, deque<Vector3d>(N_SUB_SEG, Vector3d(0, 0, 0)));
        ssVel = sfVel = deque<deque<Vector3d>>(WINDOW_SIZE, deque<Vector3d>(N_SUB_SEG, Vector3d(0, 0, 0)));
        ssBia = sfBia = deque<deque<Vector3d>>(WINDOW_SIZE, deque<Vector3d>(N_SUB_SEG, Vector3d(0, 0, 0)));
        ssBig = sfBig = deque<deque<Vector3d>>(WINDOW_SIZE, deque<Vector3d>(N_SUB_SEG, Vector3d(0, 0, 0)));

        // Gravity constant
        double GRAV_ = 9.82;
        nh_ptr->param("/GRAV", GRAV_, 9.82);
        GRAV = Vector3d(0, 0, GRAV_);
        printf("GRAV constant: %f\n", GRAV_);

        nh_ptr->getParam("/GYR_N", GYR_N);
        nh_ptr->getParam("/GYR_W", GYR_W);
        nh_ptr->getParam("/ACC_N", ACC_N);
        nh_ptr->getParam("/ACC_W", ACC_W);

        printf("Gyro variance: "); printf("%f ", GYR_N); cout << endl;
        printf("Bgyr variance: "); printf("%f ", GYR_W); cout << endl;
        printf("Acce variance: "); printf("%f ", ACC_N); cout << endl;
        printf("Bacc variance: "); printf("%f ", ACC_W); cout << endl;

        // Sensor weightage
        nh_ptr->getParam("/lidar_weight", lidar_weight);

        // Downsample size
        nh_ptr->getParam("/leaf_size",          leaf_size);
        nh_ptr->getParam("/assoc_spacing",      assoc_spacing);
        nh_ptr->getParam("/surfel_map_depth",   surfel_map_depth);
        nh_ptr->getParam("/surfel_min_point",   surfel_min_point);
        nh_ptr->getParam("/surfel_min_depth",   surfel_min_depth);
        nh_ptr->getParam("/surfel_query_depth", surfel_query_depth);
        nh_ptr->getParam("/surfel_intsect_rad", surfel_intsect_rad);
        nh_ptr->getParam("/surfel_min_plnrty",  surfel_min_plnrty);

        printf("leaf_size:          %f\n", leaf_size);
        printf("assoc_spacing:      %f\n", assoc_spacing);
        printf("surfel_map_depth:   %d\n", surfel_map_depth);
        printf("surfel_min_point:   %d\n", surfel_min_point);
        printf("surfel_min_depth:   %d\n", surfel_min_depth);
        printf("surfel_query_depth: %d\n", surfel_query_depth);
        printf("surfel_intsect_rad: %f\n", surfel_intsect_rad);
        printf("surfel_min_plnrty:  %f\n", surfel_min_plnrty);

        // Number of neigbours to check for in association
        nh_ptr->getParam("/dis_to_surfel_max", dis_to_surfel_max);
        nh_ptr->getParam("/score_min", score_min);
        // Lidar feature downsample rate
        // nh_ptr->getParam("/ds_rate", ds_rate);

        // Keyframe params
        nh_ptr->getParam("/kf_min_dis", kf_min_dis);
        nh_ptr->getParam("/kf_min_angle", kf_min_angle);

        // Optimization parameters
        nh_ptr->getParam("/lidar_loss_thres", lidar_loss_thres);

        // Solver
        string trustRegType_;
        nh_ptr->param("/trustRegType", trustRegType_, string("lm"));
        if (trustRegType_ == "lm")
            trustRegType = ceres::LEVENBERG_MARQUARDT;
        else if( trustRegType_ == "dogleg")
            trustRegType = ceres::DOGLEG;
        else
            trustRegType = ceres::LEVENBERG_MARQUARDT;
        printf(KYEL "/trustRegType: %d. %s\n" RESET, trustRegType, trustRegType_.c_str());

        string linAlgbLib_;
        nh_ptr->param("/linAlgbLib", linAlgbLib_, string("cuda"));
        if (linAlgbLib_ == "eigen")
            linAlgbLib = ceres::DenseLinearAlgebraLibraryType::EIGEN;
        else if(linAlgbLib_ == "lapack")
            linAlgbLib = ceres::DenseLinearAlgebraLibraryType::LAPACK;
        // else if(linAlgbLib_ == "cuda")
        //     linAlgbLib = ceres::DenseLinearAlgebraLibraryType::CUDA;
        else
            linAlgbLib = ceres::DenseLinearAlgebraLibraryType::EIGEN;
        printf(KYEL "/linAlgbLib: %d. %s\n" RESET, linAlgbLib, linAlgbLib_.c_str());

        nh_ptr->param("/max_solve_time", max_solve_time,  0.5);
        nh_ptr->param("/max_iterations", max_iterations,  200);

        // Fusion option
        fuse_lidar     = GetBoolParam("/fuse_lidar",     true);
        fuse_imu       = GetBoolParam("/fuse_imu",       true);

        snap_to_0180   = GetBoolParam("/snap_to_0180",   false);
        regularize_imu = GetBoolParam("/regularize_imu", true);

        nh_ptr->param("/fix_mode",         fix_mode,         1);
        nh_ptr->param("/imu_init_time",    imu_init_time,    0.1);
        nh_ptr->param("/max_outer_iters",  max_outer_iters,  1);
        nh_ptr->param("/max_lidar_factor", max_lidar_factor, 4000);

        printf("max_outer_iters: %d.\n"
               "fix_mode:        %d.\n"
               "max_iterations:  %d.\n"
               "imu_init_time:   %f\n",
                max_outer_iters, fix_mode, max_iterations, imu_init_time);
        
        // Loop parameters
        loop_en = GetBoolParam("/loop_en", true);
        nh_ptr->param("/loop_kf_nbr", loop_kf_nbr, 5);
        nh_ptr->param("/loop_time_mindiff", loop_time_mindiff, 10);

        nh_ptr->param("/icpMaxIter", icpMaxIter, 20);
        nh_ptr->param("/icpFitnessThres", icpFitnessThres, 0.3);
        nh_ptr->param("/histDis", histDis, 15.0);

        nh_ptr->param("/rib_edge", rib_edge, 5);
        nh_ptr->param("/odom_q_noise", odom_q_noise, 0.1);
        nh_ptr->param("/odom_p_noise", odom_p_noise, 0.1);
        nh_ptr->param("/loop_weight", loop_weight, 0.1);
        
        // Map inertialization
        KfCloudPose = CloudPosePtr(new CloudPose());
        globalMap = CloudXYZITPtr(new CloudXYZIT());
        surfelMap = ufoSurfelMap(leaf_size, surfel_map_depth);

        // Advertise the global map
        global_map_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/global_map", 10);

        publish_map = GetBoolParam("/publish_map", true);

        // Subscribe to the lidar-imu package
        data_sub = nh_ptr->subscribe("/sensors_sync", 100, &Estimator::DataHandler, this);

        // Advertise the outputs
        kfcloud_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/kfcloud", 10);
        kfpose_pub  = nh_ptr->advertise<sensor_msgs::PointCloud2>("/kfpose", 10);

        // Advertise the service
        global_maps_srv = nh_ptr->advertiseService("/global_maps_publish", &Estimator::PublishGlobalMaps, this);

        // Log file
        log_dir = nh_ptr->param("/log_dir", log_dir);
        std::filesystem::create_directories(log_dir);

        loop_log_file.open(log_dir + "/loop_log.csv");
        loop_log_file.precision(std::numeric_limits<double>::digits10 + 1);
        // loop_log_file.close();
    }

    bool GetBoolParam(string param, bool default_value)
    {
        int param_;
        nh_ptr->param(param, param_, default_value == true ? 1 : 0);
        return (param_ == 0 ? false : true);
    }

    void DataHandler(const slict::FeatureCloud::ConstPtr &msg)
    {
        lock_guard<mutex> lock(packet_buf_mtx);
        packet_buf.push_back(msg);
    }

    bool ProcessData()
    {
        while (ros::ok())
        {
            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            // Check for time out to exit the program
            static TicToc tt_time_out;

            static double data_time_out = -1;
            if ( (data_time_out != -1) && (tt_time_out.Toc()/1000.0 - data_time_out) > 20 && (packet_buf.size() == 0) && autoexit)
            {
                printf(KYEL "Data timeout, Buf: %d. exit!\n" RESET, packet_buf.size());
                SaveTrajLog();
                exit(0);
            }

            /* #region STEP 0: Loop if there is no new data ---------------------------------------------------------*/
            
            TicToc tt_loop;

            if (packet_buf.empty())
            {
                this_thread::sleep_for(chrono::milliseconds(5));
                continue;
            }

            /* #endregion STEP 0: Loop if there is no new data ------------------------------------------------------*/

            /* #region STEP 1: Extract the data packet --------------------------------------------------------------*/
            
            TicToc tt_preopt;

            slict::FeatureCloud::ConstPtr packet;
            {
                lock_guard<mutex> lock(packet_buf_mtx);
                packet = packet_buf.front();
                packet_buf.pop_front();
            }

            // Reset the time out clock
            data_time_out = tt_time_out.Toc()/1000.0;

            /* #endregion STEP 1: Extract the data packet -----------------------------------------------------------*/

            /* #region STEP 2: Initialize orientation and Map -------------------------------------------------------*/
            
            if (!ALL_INITED)
            {
                InitSensorData(packet);
                if (!ALL_INITED)
                    continue;
            }

            /* #endregion STEP 2: Initialize orientation and Map ----------------------------------------------------*/

            /* #region STEP 3: Insert the data to the buffers -------------------------------------------------------*/
            
            TicToc tt_insert;

            // Extend the time steps
            AddNewTimeStep(SwTimeStep, packet);

            // Copy the pointcloud
            SwCloud.push_back(CloudXYZITPtr(new CloudXYZIT()));
            pcl::fromROSMsg(packet->extracted_cloud, *SwCloud.back());

            // Downsample the scan
            if(leaf_size > 0)
            {
                pcl::UniformSampling<PointXYZIT> downsampler;
                downsampler.setRadiusSearch(leaf_size);
                downsampler.setInputCloud(SwCloud.back());
                downsampler.filter(*SwCloud.back());
            }

            // Create the container for the latest pointcloud
            SwCloudDsk.push_back(CloudXYZITPtr(new CloudXYZIT()));

            // Create the container for the downsampled deskewed pointcloud
            SwCloudDskDS.push_back(CloudXYZITPtr(new CloudXYZIT()));

            // Buffer to store the coefficients of the lidar factors
            SwLidarCoef.push_back(deque<LidarCoef>());

            // Buffer to count the number of associations per voxel
            SwDepVsAssoc.push_back(map<int, int>());

            // Add buffer the IMU samples at the last state
            AddImuToBuff(SwTimeStep, SwImuBundle, packet, regularize_imu);
            
            // Check the imu distribution
            // for(int i = 0; i < SwImuBundle.size(); i++)
            // {
            //     printf(KBLU "Step: %2d / %2d. Time: [%.6f, %.6f]\n" RESET, i, SwTimeStep.size(), SwTimeStep[i].front().start_time, SwTimeStep[i].back().final_time);

            //     for(int j = 0; j < SwTimeStep[i].size(); j++)
            //     {
            //         printf(KGRN "Substep %2d / %2d. Time: [%.6f, %.6f]\n" RESET, j, N_SUB_SEG, SwTimeStep[i][j].start_time, SwTimeStep[i][j].final_time);

            //         for(int k = 0; k < SwImuBundle[i][j].size(); k++)
            //             printf(KYEL "ImuSample %2d. Time: %.6f. Dt: %.3f. GYR: %9.6f, %9.6f, %9.6f. ACC: %9.6f, %9.6f, %9.6f\n" RESET,
            //                          k, SwImuBundle[i][j][k].t, k == 0 ? 0.0 : SwImuBundle[i][j][k].t - SwImuBundle[i][j][k-1].t,
            //                          SwImuBundle[i][j][k].gyro(0), SwImuBundle[i][j][k].gyro(1), SwImuBundle[i][j][k].gyro(2),
            //                          SwImuBundle[i][j][k].acce(0), SwImuBundle[i][j][k].acce(1), SwImuBundle[i][j][k].acce(2));
            //     }
            // }

            // Imu propagated states
            SwPropState.push_back(deque<ImuProp>(N_SUB_SEG));

            tt_insert.Toc();

            /* #endregion STEP 3: Insert the data to the buffers ----------------------------------------------------*/

            /* #region STEP 4: IMU Propagation on the last segments -------------------------------------------------*/

            TicToc tt_imuprop;

            for(int i = 0; i < SwImuBundle.back().size(); i++)
            {
                auto &imuSubSeq = SwImuBundle.back()[i];
                auto &subSegment = SwTimeStep.back()[i];

                // ASSUMPTION: IMU data is not interrupted (any sub-segment should have IMU data), can be relaxed ?
                // printf("Step: %2d / %2d. imuSubSeq.size(): %d\n", i, SwImuBundle.back().size(), imuSubSeq.size());
                ROS_ASSERT(!imuSubSeq.empty());

                // ASSUMPTION: an IMU sample is interpolated at each segment's start and end point
                // ROS_ASSERT_MSG(imuSubSeq.front().t == subSegment.start_time && imuSubSeq.back().t == subSegment.final_time,
                //                "IMU Time: %f, %f. Seg. Time: %f, %f\n",
                //                imuSubSeq.front().t, imuSubSeq.back().t, subSegment.start_time, subSegment.final_time);

                SwPropState.back()[i] = ImuProp(ssQua.back()[i], ssPos.back()[i], ssVel.back()[i],
                                                ssBig.back()[i], ssBia.back()[i], GRAV, imuSubSeq);

                sfQua.back()[i] = SwPropState.back()[i].Q.back();
                sfPos.back()[i] = SwPropState.back()[i].P.back();
                sfVel.back()[i] = SwPropState.back()[i].V.back();

                // Initialize start state of next segment with the final propogated state in the previous segment
                if (i <= SwImuBundle.back().size() - 2)
                {
                    ssQua.back()[i+1] = sfQua.back()[i];
                    ssPos.back()[i+1] = sfPos.back()[i];
                    ssVel.back()[i+1] = sfVel.back()[i];
                }
            }
            
            tt_imuprop.Toc();

            /* #endregion STEP 4: IMU Propagation on the last segments ----------------------------------------------*/

            /* #region STEP 5: DESKEW the pointcloud ----------------------------------------------------------------*/
            
            // TicToc tt_deskew;

            DeskewByImu(SwPropState.back(), SwTimeStep.back(),
                        SwCloud.back(), SwCloudDsk.back(), SwCloudDskDS.back(), assoc_spacing);

            // printf("Deskew Time Begin: %f\n", tt_deskew.Toc());

            /* #endregion STEP 5: DESKEW the pointcloud -------------------------------------------------------------*/

            // Loop if sliding window has not reached required length
            if (SwTimeStep.size() < WINDOW_SIZE)
            {
                printf(KGRN "Buffer size %02d / %02d\n" RESET, SwTimeStep.size(), WINDOW_SIZE);
                continue;
            }
            else
            {
                static bool first_shot = true;
                if (first_shot)
                {
                    first_shot = false;
                    printf(KGRN "Buffer size %02d / %02d. WINDOW SIZE reached.\n" RESET, SwTimeStep.size(), WINDOW_SIZE);
                }
            }

            /* #region STEP 6: Associate scan with map --------------------------------------------------------------*/

            static int last_ufomap_version = ufomap_version;
            
            // Reset the association if ufomap has been updated
            static bool first_round = true;
            if (ufomap_version != last_ufomap_version)
            {
                first_round   = true;
                last_ufomap_version = ufomap_version;

                printf(KYEL "UFOMAP RESET.\n" RESET);
            }

            for (int i = first_round ? 0 : WINDOW_SIZE - 1; i < WINDOW_SIZE; i++)
            {
                // TicToc tt_assoc;

                SwDepVsAssoc[i].clear();
                AssociateCloudWithMap(SwTimeStep[i], surfelMap, mytf(sfQua[i].back(), sfPos[i].back()),
                                      SwCloud[i], SwCloudDsk[i], SwCloudDskDS[i], SwLidarCoef[i], SwDepVsAssoc[i]);

                // printf("Assoc Time Begin: %f\n", tt_assoc.Toc());
            }
            
            first_round = false;
            // find_new_node = false;

            /* #endregion STEP 6: Associate scan with map -----------------------------------------------------------*/

            /* #region STEP 7: LIO optimizaton ----------------------------------------------------------------------*/

            tt_preopt.Toc();

            static int optNum = 0; optNum++;
            slict::OptStat optreport[max_outer_iters];
            for (auto &report : optreport)
                report.OptNum = optNum;

            string printout, DVAReport;

            int outer_iter = max_outer_iters;
            while(outer_iter > 0)
            {
                slict::OptStat &report = optreport[outer_iter-1]; 
                
                // Calculate the downsampling rate at each depth            
                makeDVAReport(SwDepVsAssoc, DVA, total_lidar_coef, DVAReport);
                lidar_ds_rate = (max_lidar_factor == -1 ? 1 : max(1, (int)std::floor((double)total_lidar_coef / max_lidar_factor)));

                LIOOptimization(report);

                /* #region Post optimization ------------------------------------------------------------------------*/

                TicToc tt_posproc;

                // Redo associate
                outer_iter--;

                // Backward propogate of the state
                #pragma omp parallel for num_threads(MAX_THREADS)
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    for(int j = 0; j < SwTimeStep[i].size(); j++)
                    {
                        SwPropState[i][j] = ImuProp(sfQua[i][j], sfPos[i][j], sfVel[i][j],
                                                    sfBig[i][j], sfBia[i][j], GRAV, SwImuBundle[i][j], -1);

                        if (i == 0 && j == 0)
                        {
                            ssQua[i][j] = SwPropState[i][j].Q.front();
                            ssPos[i][j] = SwPropState[i][j].P.front();
                            ssVel[i][j] = SwPropState[i][j].V.front();
                        }
                    }
                }

                // Redo the deskew
                for(int i = 0; i < WINDOW_SIZE; i++)
                {
                    // TicToc tt_deskew;
                    
                    DeskewByImu(SwPropState[i], SwTimeStep[i], SwCloud[i], SwCloudDsk[i], SwCloudDskDS[i], assoc_spacing);
                    
                    // printf("Deskew Time Loop: %f\n", tt_deskew.Toc());
                }

                // Redo associations and loop if optimization iterations exceed the threshold
                bool convergent = (report.iters < max_iterations && report.tslv/1000.0 < max_solve_time);
                
                // Redo the map association
                for (int i = 0; i < WINDOW_SIZE; i++)
                {
                    // TicToc tt_assoc;

                    SwDepVsAssoc[i].clear();
                    AssociateCloudWithMap(SwTimeStep[i], surfelMap, mytf(sfQua[i].back(), sfPos[i].back()),
                                          SwCloud[i], SwCloudDsk[i], SwCloudDskDS[i], SwLidarCoef[i], SwDepVsAssoc[i]);

                    // printf("Assoc Time Loop: %f\n", tt_assoc.Toc());
                }
                
                tt_posproc.Toc();

                /* #endregion Post optimization ---------------------------------------------------------------------*/

                /* #region Write the report -------------------------------------------------------------------------*/
                
                // Update the report
                report.header.stamp = ros::Time(SwTimeStep.back().back().final_time);
                report.OptNumSub = outer_iter + 1;
                report.keyfrm = KfCloudPose->size();
                report.margPerc = margPerc;
                report.fixed_knot_min = first_fixed_knot;
                report.fixed_knot_max = last_fixed_knot;

                report.tpreopt  = tt_preopt.GetLastStop();
                report.tpostopt = tt_posproc.GetLastStop();
                report.tlp      = tt_loop.Toc();

                static double last_tmapping = -1;
                if (last_tmapping != tt_ufmupdt.GetLastStop())
                {
                    report.tmapimg = tt_ufmupdt.GetLastStop();
                    last_tmapping  = tt_ufmupdt.GetLastStop();
                }
                else
                    report.tmapimg = -1;


                Vector3d eul_est = Util::Quat2YPR(Quaternd(report.Qest.w, report.Qest.x, report.Qest.y, report.Qest.z));
                Vector3d eul_imu = Util::Quat2YPR(Quaternd(report.Qest.w, report.Qest.x, report.Qest.y, report.Qest.z));
                Vector3d Vest = Vector3d(report.Vest.x, report.Vest.y, report.Vest.z);
                Vector3d Vimu = Vector3d(report.Vimu.x, report.Vimu.y, report.Vimu.z);

                /* #region */
                printout +=
                    myprintf("Op#.Oi#: %04d. %2d /%2d. Itr: %2d / %2d. tpo: %4.0f. tbc: %4.0f. tslv: %4.0f. tpp: %4.0f. tlp: %4.0f. tufm: %4.0f. tlpBa: %4.0f. "
                             "trun: %.3f\n"
                             "Ftr: Ldr: %5d / %5d / %5d. IMU: %5d. Vel: %2d. Buf: %2d. Kfr: %d. Marg%%: %3.0f. Kfca: %d. "
                            //  "Knots: %d -> %d. Fixed: %d -> %d. "
                             "Map: %d\n"
                             "J0:  %15.3f, Ldr: %15.3f. IMU: %15.3f. Vel: %9.3f.\n"
                             "JK:  %15.3f, Ldr: %15.3f. IMU: %15.3f. Vel: %9.3f.\n"
                            //  "BiaG: %7.2f, %7.2f, %7.2f. BiaA: %7.2f, %7.2f, %7.2f\n"
                            //  "Eimu: %7.2f, %7.2f, %7.2f. Pimu: %7.2f, %7.2f, %7.2f. Vimu: %7.2f, %7.2f, %7.2f.\n"
                             "Eest: %7.2f, %7.2f, %7.2f. Pest: %7.2f, %7.2f, %7.2f. Vest: %7.2f, %7.2f, %7.2f. Spd: %.3f. Dif: %.3f.\n"
                             "DVA:  %s\n"
                             "\n",
                             // Time and iterations
                             report.OptNum, report.OptNumSub, max_outer_iters,
                             report.iters, max_iterations,
                             report.tpreopt,           // time preparing before LIOOptimization
                             report.tbuildceres,       // time building the ceres problem before solving
                             report.tslv,              // time solving ceres problem
                             report.tpostopt,          // time for post processing
                             report.tlp,               // time packet was extracted up to now
                             report.tmapimg,           // time of last insertion of data to ufomap
                             tt_loopBA.GetLastStop(),  // time checking loop closure
                             report.trun,
                             // Sliding window stats
                             report.surfFactors, max_lidar_factor, total_lidar_coef, report.imuFactors, report.velFactors,
                             report.mfcBuf = packet_buf.size(), report.keyfrm, report.margPerc, report.kfcand,
                             // active_knots.begin()->first, active_knots.rbegin()->first,
                             // report.fixed_knot_min, report.fixed_knot_max,
                             surfelMap.size(),
                             // Optimization initial costs
                             report.J0, report.J0Surf, report.J0Imu, report.J0Vel,
                             // Optimization final costs
                             report.JK, report.JKSurf, report.JKImu, report.JKVel,
                            //  // Bias Estimate
                            //  ssBig.back().back().x(), ssBig.back().back().y(), ssBig.back().back().z(),
                            //  ssBia.back().back().x(), ssBia.back().back().y(), ssBia.back().back().z(),
                             // Pose Estimate from propogation
                            //  eul_imu.x(), eul_imu.y(), eul_imu.z(),
                            //  report.Pimu.x, report.Pimu.y, report.Pimu.z,
                            //  report.Vimu.x, report.Vimu.y, report.Vimu.z,
                             // Pose Estimate from Optimization
                             eul_est.x(), eul_est.y(), eul_est.z(),
                             report.Pest.x, report.Pest.y, report.Pest.z,
                             report.Vest.x, report.Vest.y, report.Vest.z,
                             Vest.norm(), (Vest - Vimu).norm(),
                             // Report on the assocations at different scales
                             DVAReport.c_str());
                /* #endregion */        

                // Attach the report from loop closure
                /* #region */
                printout +=
                    myprintf("%sBA# %4d. LoopEn: %d. LastFn: %6.3f. Itr: %3d. tslv: %4.0f. trbm: %4.0f. Ftr: RP: %4d. Lp: %4d.\n"
                             "J:  %6.3f -> %6.3f. rP: %6.3f -> %6.3f. Lp: %6.3f -> %6.3f\n\n" RESET,
                             // Stats
                             baReport.turn % 2 == 0 ? KBLU : KGRN, baReport.turn, loop_en, lastICPFn,
                             baReport.pgopt_iter, baReport.pgopt_time, baReport.rebuildmap_time,
                             baReport.factor_relpose, baReport.factor_loop,
                             // Costs
                             baReport.J0, baReport.JK,
                             baReport.J0_relpose, baReport.JK_relpose,
                             baReport.J0_loop, baReport.JK_loop);
                /* #endregion */        

                // Publish the optimization results
                static ros::Publisher opt_stat_pub = nh_ptr->advertise<slict::OptStat>("/opt_stat", 1);
                opt_stat_pub.publish(report);

                /* #endregion Write the report ----------------------------------------------------------------------*/

                // Break the loop early if the optimization finishes quickly.
                if (convergent)
                    break;
            }

            /* #endregion STEP 7: LIO optimizaton -------------------------------------------------------------------*/

            /* #region STEP 8: Recruit Keyframe ---------------------------------------------------------------------*/

            NominateKeyframe();

            /* #endregion STEP 8: Recruit Keyframe ------------------------------------------------------------------*/

            /* #region STEP 9: Loop Closure and BA ------------------------------------------------------------------*/

            tt_loopBA.Tic();

            if (loop_en)
            {
                DetectLoop();
                BundleAdjustment(baReport);
            }

            tt_loopBA.Toc();

            /* #endregion STEP 9: Loop Closure and BA ---------------------------------------------------------------*/
            
            /* #region STEP 10: Report and Vizualize ----------------------------------------------------------------*/ 
            
            // Export the summaries
            cout << printout;

            VisualizeSwTraj();

            VisualizeLoop();

            /* #endregion STEP 10: Report and Vizualize -------------------------------------------------------------*/ 

            /* #region STEP 11: Slide window forward ----------------------------------------------------------------*/

            // Slide the window forward
            SlideWindowForward();

            /* #endregion STEP 11: Slide window forward -------------------------------------------------------------*/
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            static ros::Publisher calc_time_pub = nh_ptr->advertise<std_msgs::Float32>("/calc_time", 100);
            std_msgs::Float32 calc_time;
            calc_time.data = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
            calc_time_pub.publish(calc_time);            
        }
    }

    void InitSensorData(slict::FeatureCloud::ConstPtr &packet)
    {
        static bool IMU_INITED = false;
        static bool LIDAR_INITED = false;

        if (!IMU_INITED)
        {
            const vector<sensor_msgs::Imu> &imu_bundle = packet->imu_msgs;

            static vector<Vector3d> gyr_buf;
            static vector<Vector3d> acc_buf;
            static double first_imu_time = imu_bundle.front().header.stamp.toSec();

            // Push the sample into buffer;
            for (auto imu_sample : imu_bundle)
            {
                if (imu_sample.header.seq == 0)
                {
                    gyr_buf.push_back(Vector3d(imu_sample.angular_velocity.x,
                                               imu_sample.angular_velocity.y,
                                               imu_sample.angular_velocity.z));
                    acc_buf.push_back(Vector3d(imu_sample.linear_acceleration.x,
                                               imu_sample.linear_acceleration.y,
                                               imu_sample.linear_acceleration.z));
                }
            }

            // Average the IMU measurements and initialize the states
            if (!gyr_buf.empty() &&
                fabs(imu_bundle.front().header.stamp.toSec() - first_imu_time) > imu_init_time)
            {
                // Calculate the gyro bias
                Vector3d gyr_avr(0, 0, 0);
                for (auto gyr_sample : gyr_buf)
                    gyr_avr += gyr_sample;

                gyr_avr /= gyr_buf.size();

                // Calculate the original orientation
                Vector3d acc_avr(0, 0, 0);
                for (auto acc_sample : acc_buf)
                    acc_avr += acc_sample;

                acc_avr /= acc_buf.size();
                
                Quaternd q_init(Util::grav2Rot(acc_avr));
                Vector3d ypr = Util::Quat2YPR(q_init);

                printf("Gyro Bias: %.3f, %.3f, %.3f. Samples: %d. %d\n",
                        gyr_avr(0), gyr_avr(1), gyr_avr(2), gyr_buf.size(), acc_buf.size());
                printf("Init YPR:  %.3f, %.3f, %.3f.\n", ypr(0), ypr(1), ypr(2));

                // Initialize the original quaternion state
                ssQua = sfQua = deque<deque<Quaternd>>(WINDOW_SIZE, deque<Quaternd>(N_SUB_SEG, q_init));
                ssBig = sfBig = deque<deque<Vector3d>>(WINDOW_SIZE, deque<Vector3d>(N_SUB_SEG, gyr_avr));
                ssBia = sfBia = deque<deque<Vector3d>>(WINDOW_SIZE, deque<Vector3d>(N_SUB_SEG, Vector3d(0, 0, 0)));

                IMU_INITED = true;
            }
        }
    
        if (!LIDAR_INITED)
        {
            static CloudXYZITPtr kfCloud0(new CloudXYZIT());
            
            // CloudXYZIT temp;
            pcl::fromROSMsg(packet->extracted_cloud, *kfCloud0);

            // *kfCloud0 += temp;

            if(IMU_INITED)
            {   
                // Downsample the cached kf data
                pcl::UniformSampling<PointXYZIT> downsampler;
                downsampler.setRadiusSearch(leaf_size);
                downsampler.setInputCloud(kfCloud0);
                downsampler.filter(*kfCloud0);

                // Admit the pointcloud to buffer
                AdmitKeyframe(packet->header.stamp.toSec(), sfQua[0].back(), Vector3d(0, 0, 0), kfCloud0, kfCloud0);

                // Write the file for quick visualization
                PCDWriter writer; writer.writeASCII(log_dir + "/KfCloudPose.pcd", *KfCloudPose, 18);

                LIDAR_INITED = true;
            }
        }

        if (IMU_INITED && LIDAR_INITED)
            ALL_INITED = true;
    }

    void AddNewTimeStep(deque<deque<TimeSegment>> &timeStepDeque, slict::FeatureCloud::ConstPtr &packet)
    {
        // Add new sequence of sub time step
        timeStepDeque.push_back(deque<TimeSegment>());

        // Calculate the sub time steps
        double start_time, final_time, sub_timestep;
        if (timeStepDeque.size() == 1)
        {
            start_time = packet->scanEndTime - 0.1;
            final_time = packet->scanEndTime;
            sub_timestep = (final_time - start_time)/N_SUB_SEG;
        }
        else
        {
            start_time = timeStepDeque.rbegin()[1].back().final_time;
            final_time = packet->scanEndTime;
            sub_timestep = (final_time - start_time)/N_SUB_SEG;
        }

        for(int i = 0; i < N_SUB_SEG; i++)
            timeStepDeque.back().push_back(TimeSegment(start_time + i*sub_timestep,
                                                       start_time + (i+1)*sub_timestep));
    }

    void AddImuToBuff(deque<deque<TimeSegment>> &timeStepDeque, deque<deque<ImuSequence>> &imuBundleDeque,
                      slict::FeatureCloud::ConstPtr &packet, bool regularize_imu)
    {
        // Extend the imu deque
        imuBundleDeque.push_back(deque<ImuSequence>(N_SUB_SEG));

        // Extract and regularize imu data at on the latest buffer
        ImuSequence newImuSequence;
        ExtractImuData(newImuSequence, packet, regularize_imu); // Only select the primary IMU at this stage

        // Extract subsequence of imu sample data, interpolate at sub steps
        if(timeStepDeque.size() == 1)
        {
            // Duplicate the first sample for continuity
            newImuSequence.push_front(newImuSequence.front());
            newImuSequence.front().t = timeStepDeque.front().front().start_time;
        }
        else
            // Borrow the last sample time in the previous interval for continuity
            newImuSequence.push_front(imuBundleDeque.rbegin()[1].back().back());

        // Extract the samples in each sub interval          
        for(int i = 0; i < timeStepDeque.back().size(); i++)
        {
            double start_time = timeStepDeque.back()[i].start_time;
            double final_time = timeStepDeque.back()[i].final_time;
            double dt = final_time - start_time;
            
            imuBundleDeque.back()[i] = newImuSequence.subsequence(start_time, final_time);
            for(int j = 0; j < imuBundleDeque.back()[i].size(); j++)
            {
                imuBundleDeque.back()[i][j].u = start_time;
                imuBundleDeque.back()[i][j].s = (imuBundleDeque.back()[i][j].t - start_time)/dt;
            }
        }
    }

    void ExtractImuData(ImuSequence &imu_sequence, slict::FeatureCloud::ConstPtr &packet, bool regularize_timestamp)
    {
        // Copy the messages to the deque
        for(auto &imu : packet->imu_msgs)
        {
            imu_sequence.push_back(ImuSample(imu.header.stamp.toSec(),
                                             Vector3d(imu.angular_velocity.x,
                                                      imu.angular_velocity.y,
                                                      imu.angular_velocity.z),
                                             Vector3d(imu.linear_acceleration.x,
                                                      imu.linear_acceleration.y,
                                                      imu.linear_acceleration.z)));
        }

        if (regularize_timestamp)
        {
            if (imu_sequence.size() <= 2)
                return;

            double t0 = imu_sequence.front().t;
            double tK = imu_sequence.back().t;

            double dt = (tK - t0)/(imu_sequence.size() - 1);
            
            for(int i = 0; i < imu_sequence.size(); i++)
                imu_sequence[i].t = t0 + dt*i;
        }
    }

    void DeskewByImu(deque<ImuProp> &imuProp, deque<TimeSegment> timeSeg,
                     CloudXYZITPtr &inCloud, CloudXYZITPtr &outCloud, CloudXYZITPtr &outCloudDS, double ds_rate)
    {
        if (!fuse_imu)
        {
            *outCloud = *inCloud;
            
            pcl::UniformSampling<PointXYZIT> downsampler;
            downsampler.setRadiusSearch(ds_rate);
            downsampler.setInputCloud(outCloud);
            downsampler.filter(*outCloudDS);

            return;
        }

        int cloud_size = inCloud->size();
        outCloud->resize(cloud_size);
        outCloudDS->resize(cloud_size);

        double &start_time = timeSeg.front().start_time;
        double &final_time = timeSeg.back().final_time;
        
        #pragma omp parallel for num_threads(MAX_THREADS)
        for (int i = 0; i < cloud_size; i++)
        {
            auto &inPoint = inCloud->points[i];

            double ts = inPoint.t + start_time;
            
            // Find the corresponding subsegment
            int seg_idx = -1;
            for(int j = 0; j < timeSeg.size(); j++)
            {
                if(timeSeg[j].start_time <= ts && ts <= timeSeg[j].final_time)
                {
                    seg_idx = j;
                    break;
                }
                else if (timeSeg[j].start_time - 0.001 <= ts && ts < timeSeg[j].start_time)
                {
                    ts = timeSeg[j].start_time;
                    inPoint.t = ts - start_time;
                    seg_idx = j;
                    break;
                }
                else if (timeSeg[j].final_time < ts && ts <= timeSeg[j].final_time + 0.001)
                {
                    ts = timeSeg[j].final_time;
                    inPoint.t = ts - start_time;
                    seg_idx = j;
                    break;
                }
            }

            if(seg_idx == -1)
            {
                printf(KYEL "Point time %f not in segment: [%f, %f]. Discarding\n" RESET, ts, start_time, final_time);
                outCloud->points[i].x = 0; outCloud->points[i].y = 0; outCloud->points[i].z = 0;
                outCloud->points[i].intensity = 0;
                outCloud->points[i].t = -1; // Mark this point is invalid
                continue;
            }

            // Transform all points to the end of the scan
            myTf T_Bk_Bs = imuProp.back().getBackTf().inverse()*imuProp[seg_idx].getTf(ts);

            Vector3d point_at_end_time = T_Bk_Bs.rot * Vector3d(inPoint.x, inPoint.y, inPoint.z) + T_Bk_Bs.pos;

            outCloud->points[i].x = point_at_end_time.x();
            outCloud->points[i].y = point_at_end_time.y();
            outCloud->points[i].z = point_at_end_time.z();
            outCloud->points[i].intensity = inPoint.intensity;
            outCloud->points[i].t = inPoint.t;

            outCloudDS->points[i] = outCloud->points[i];
            outCloudDS->points[i].intensity = i;
        }

        pcl::UniformSampling<PointXYZIT> downsampler;
        CloudXYZITPtr tempDSCloud(new CloudXYZIT);
        downsampler.setRadiusSearch(ds_rate);
        downsampler.setInputCloud(outCloudDS);
        downsampler.filter(*tempDSCloud);

        // If downsampled pointcloud has too few points, relax the ds_rate
        int ds_sized = tempDSCloud->size();
        if(ds_sized < 2*max_lidar_factor/WINDOW_SIZE)
        {
            downsampler.setRadiusSearch(0.1);
            downsampler.setInputCloud(outCloudDS);
            downsampler.filter(*outCloudDS);

            // printf(KYEL "Too few points after downsampling. Relaxing assoc_spacing to 0.2. Points: %d -> %d" RESET,
            //              ds_sized, outCloudDS->size());
        }
        else
            outCloudDS = tempDSCloud;
    }

    void LIOOptimization(slict::OptStat &report)
    {
        TicToc tt_buildceres;

        // Create and solve the Ceres Problem
        ceres::Problem problem;
        ceres::Solver::Options options;

        // Set up the options
        // options.minimizer_type = ceres::TRUST_REGION;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.trust_region_strategy_type = trustRegType;
        options.dense_linear_algebra_library_type = linAlgbLib;
        options.max_num_iterations = max_iterations;
        options.max_solver_time_in_seconds = max_solve_time;
        options.num_threads = MAX_THREADS;
        options.minimizer_progress_to_stdout = false;

        // Create optimization params
        map<double, double*> PARAM_POSE;
        map<double, double*> PARAM_VELO;
        map<double, double*> PARAM_BIAS;

        // State ID, to be used to match the state estimates with the time instances on the sliding window
        struct StateID
        {
            ~StateID() {};

            StateID()
            {
                step_idx = 0; segm_idx = 0; state_idx = -1; end_type  =  0;
            };

            StateID(int &step_idx_, int &segm_idx_, int state_idx_, int end_type_)
                : step_idx(step_idx_), segm_idx(segm_idx_), state_idx(state_idx_), end_type(end_type_) {};

            int step_idx;   // Time step that the state belongs to
            int segm_idx;   // Time segment in the step that the state belongs to
            int state_idx;  // Index of the state
            int end_type;   // The end of the seg: '0' for start, '1' for final.
                            // Only the first should stateID should be 0
        };
        map<double, StateID> timeToIdx;
        map<double, double> timeStartToFinal;

        struct Loader
        {
            void CopyStateToParam(Vector3d &p_, Quaternd &q_, Vector3d &v_,
                                  Vector3d &ba, Vector3d &bg,
                                  double *&pose, double *&velo, double *&bias)
            {
                pose[0] = p_.x(); pose[1] = p_.y(); pose[2] = p_.z();
                pose[3] = q_.x(); pose[4] = q_.y(); pose[5] = q_.z(); pose[6] = q_.w();

                velo[0] = v_.x(); velo[1] = v_.y(); velo[2] = v_.z();
                
                bias[0] = ba.x(); bias[1] = ba.y(); bias[2] = ba.z();
                bias[3] = bg.x(); bias[4] = bg.y(); bias[5] = bg.z();
            }

            void CopyParamToState(double *&pose, double *&velo, double *&bias,
                                  Vector3d &p_, Quaternd &q_, Vector3d &v_,
                                  Vector3d &ba, Vector3d &bg)
            {
                p_.x() = pose[0]; p_.y() = pose[1]; p_.z() = pose[2];
                q_.x() = pose[3]; q_.y() = pose[4]; q_.z() = pose[5]; q_.w() = pose[6];

                v_.x() = velo[0]; v_.y() = velo[1]; v_.z() = velo[2];

                ba.x() = bias[0]; ba.y() = bias[1]; ba.z() = bias[2];
                bg.x() = bias[3]; bg.y() = bias[4]; bg.z() = bias[5];
            }

        } loader;

        // Load values from state to params
        for(int i = 0; i < WINDOW_SIZE; i++)
        {
            for(int j = 0; j < SwTimeStep[i].size(); j++)
            {
                // Add the start time of the first segment in the first step
                if (i == 0 && j == 0)
                {
                    double timeInstance = SwTimeStep[i][j].start_time;
                    timeToIdx[timeInstance] = StateID(i, j, PARAM_POSE.size(), 0);

                    // Intialize the pose states
                    PARAM_POSE[timeInstance] = new double[7];
                    PARAM_VELO[timeInstance] = new double[3];
                    PARAM_BIAS[timeInstance] = new double[6];

                    loader.CopyStateToParam(ssPos[i][j], ssQua[i][j], ssVel[i][j], ssBia[i][j], ssBig[i][j],
                                            PARAM_POSE[timeInstance], PARAM_VELO[timeInstance], PARAM_BIAS[timeInstance]);
                }

                // Add the end time of each segment
                double timeInstance = SwTimeStep[i][j].final_time;
                timeToIdx[timeInstance] = StateID(i, j, PARAM_POSE.size(), 1);
                timeStartToFinal[SwTimeStep[i][j].start_time] = SwTimeStep[i][j].final_time;

                // Intialize the pose states
                PARAM_POSE[timeInstance] = new double[7];
                PARAM_VELO[timeInstance] = new double[3];
                PARAM_BIAS[timeInstance] = new double[6];
                
                loader.CopyStateToParam(sfPos[i][j], sfQua[i][j], sfVel[i][j], sfBia[i][j], sfBig[i][j],
                                        PARAM_POSE[timeInstance], PARAM_VELO[timeInstance], PARAM_BIAS[timeInstance]);
            }
        }

        // Add the parameter blocks
        for(auto &state_idx : timeToIdx)
        {
            double timeInstace = state_idx.first;
            
            problem.AddParameterBlock(PARAM_POSE[timeInstace], 7, new PoseLocalParameterization());
            problem.AddParameterBlock(PARAM_VELO[timeInstace], 3);
            problem.AddParameterBlock(PARAM_BIAS[timeInstace], 6);

            problem.SetParameterLowerBound(PARAM_BIAS[timeInstace], 0, -0.1);
            problem.SetParameterUpperBound(PARAM_BIAS[timeInstace], 0,  0.1);

            problem.SetParameterLowerBound(PARAM_BIAS[timeInstace], 1, -0.1);
            problem.SetParameterUpperBound(PARAM_BIAS[timeInstace], 1,  0.1);

            problem.SetParameterLowerBound(PARAM_BIAS[timeInstace], 2, -0.1);
            problem.SetParameterUpperBound(PARAM_BIAS[timeInstace], 2,  0.1);
        }

        // problem.SetParameterBlockConstant(PARAM_POSE[SwTimeStep.front().front().start_time]);

        // Cloud to show points being associated
        CloudXYZITPtr assocCloud(new CloudXYZIT());

        map<double, int> timeVsFactor;
        // Add the lidar factors
        vector<ceres::internal::ResidualBlock *> res_ids_surf;
        double cost_surf_init = -1, cost_surf_final = -1;
        if(fuse_lidar)
        {
            static int skip = -1;
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                for (auto &point : SwCloudDskDS[i]->points)
                {
                    int idx = (int)(point.intensity);
                    
                    ROS_ASSERT(idx == SwLidarCoef[i][idx*surfel_query_depth].ptIdx);

                    for(int depth = 0; depth < surfel_query_depth; depth++)
                    {
                        LidarCoef &coef = SwLidarCoef[i][idx*surfel_query_depth + depth];

                        if (coef.t < 0 || coef.u < 0)
                            continue;
                        
                        skip++;
                        if (skip % lidar_ds_rate != 0)
                            continue;

                        ceres::LossFunction *lidar_loss_function = lidar_loss_thres < 0 ? NULL : new ceres::HuberLoss(lidar_loss_thres);

                        double tss = coef.u;
                        double tsf = timeStartToFinal[tss];

                        ROS_ASSERT(timeToIdx.find(tss) != timeToIdx.end());
                        ROS_ASSERT(timeToIdx.find(tsf) != timeToIdx.end());
                        ROS_ASSERT(tss != tsf);

                        Point2PlaneDisFactorCT *f = new Point2PlaneDisFactorCT(coef.f, coef.n, coef.s, coef.dt, lidar_weight);
                        ceres::internal::ResidualBlock *res_id
                            = problem.AddResidualBlock(f, lidar_loss_function, PARAM_POSE[tss], PARAM_POSE[tsf]);
                        res_ids_surf.push_back(res_id);

                        PointXYZIT point; point.x = coef.finW.x(); point.y = coef.finW.y(); point.z = coef.finW.z();
                        assocCloud->push_back(point);

                        timeVsFactor[tsf] += 1;
                    }
                }
            }
        }

        // for(auto &time : timeVsFactor)
        // {
        //     int step  = timeToIdx[time.first].step_idx;
        //     int segm  = timeToIdx[time.first].segm_idx;
        //     int count = time.second;
        
        //     printf("Step %2d Seg %2d Time %f Count %3d\n", step, segm, time.first, count);
        // }

        // Create and add the new preintegration factors
        vector<ceres::internal::ResidualBlock *> res_ids_pimu;
        double cost_pimu_init = -1, cost_pimu_final = -1;
        deque<deque<PreintBase *>> local_preints(WINDOW_SIZE, deque<PreintBase *>(N_SUB_SEG));
        if(fuse_imu)
        {
            // Create the preint factors
            #pragma omp parallel for num_threads(WINDOW_SIZE)
            for(int i = 0; i < WINDOW_SIZE; i++)
            {
                #pragma omp parallel for num_threads(N_SUB_SEG)
                for(int j = 0; j < N_SUB_SEG; j++)
                {
                    if (SwImuBundle[i][j].size() < 3)
                    {
                        printf(KRED "Degenerate IMU sequence %d.%d. Size: %d, Time seg: [%.6f, %.6f]. CloudSize: %d.\n" RESET,
                               i, j, SwImuBundle[i][j].size(), SwTimeStep[i][j].start_time, SwTimeStep[i][j].final_time, SwCloud[i]->size());
                        continue;
                    }

                    double tss = SwImuBundle[i][j][0].u;
                    double tsf = timeStartToFinal[tss];

                    ROS_ASSERT(timeToIdx.find(tss) != timeToIdx.end());
                    ROS_ASSERT(timeToIdx.find(tsf) != timeToIdx.end());
                    ROS_ASSERT(tss != tsf);
                    ROS_ASSERT(SwImuBundle[i][j][0].t == tss);

                    local_preints[i][j] = new PreintBase(SwImuBundle[i][j][0].acce, SwImuBundle[i][j][0].gyro,
                                                         ssBia[i][j], ssBig[i][j], false,
                                                         ACC_N, ACC_W, GYR_N, GYR_W, GRAV, i);

                    for(int k = 1; k < SwImuBundle[i][j].size(); k++)
                    {
                        double dt = SwImuBundle[i][j][k].t - SwImuBundle[i][j][k-1].t;
                        local_preints[i][j]->push_back(dt, SwImuBundle[i][j][k].acce, SwImuBundle[i][j][k].gyro);
                    }
                }
            }

            // Add the factors
            for(int i = 0; i < WINDOW_SIZE; i++)
            {
                for(int j = 0; j < N_SUB_SEG; j++)
                {
                    if (SwImuBundle[i][j].size() < 3)
                    {
                        printf(KRED "Degenerate IMU sequence %d.%d. Size: %d, Time seg: [%.6f, %.6f]. CloudSize: %d.\n" RESET,
                               i, j, SwImuBundle[i][j].size(), SwTimeStep[i][j].start_time, SwTimeStep[i][j].final_time, SwCloud[i]->size());
                        continue;
                    }

                    double tss = SwImuBundle[i][j][0].u;
                    double tsf = timeStartToFinal[tss];

                    ROS_ASSERT(timeToIdx.find(tss) != timeToIdx.end());
                    ROS_ASSERT(timeToIdx.find(tsf) != timeToIdx.end());
                    ROS_ASSERT(tss != tsf);
                    ROS_ASSERT(SwImuBundle[i][j][0].t == tss);

                    // local_preints.push_back( new PreintBase(SwImuBundle[i][j][0].acce,
                    //                                         SwImuBundle[i][j][0].gyro,
                    //                                         ssBia[i][j], ssBig[i][j],
                    //                                         false, ACC_N, ACC_W,
                    //                                                GYR_N, GYR_W, GRAV, i));

                    // for(int k = 1; k < SwImuBundle[i][j].size(); k++)
                    // {
                    //     double dt = SwImuBundle[i][j][k].t - SwImuBundle[i][j][k-1].t;
                    //     local_preints.back()->push_back(dt, SwImuBundle[i][j][k].acce, SwImuBundle[i][j][k].gyro);
                    // }

                    PreintFactor *preint_factor = new PreintFactor(local_preints[i][j]);
                    ceres::internal::ResidualBlock *res_id
                        = problem.AddResidualBlock(preint_factor, NULL,
                                                   PARAM_POSE[tss], PARAM_VELO[tss], PARAM_BIAS[tss],
                                                   PARAM_POSE[tsf], PARAM_VELO[tsf], PARAM_BIAS[tsf]);
                    res_ids_pimu.push_back(res_id);
                }
            }
        }
    
        Util::ComputeCeresCost(res_ids_surf, cost_surf_init, problem);
        Util::ComputeCeresCost(res_ids_pimu, cost_pimu_init, problem);

        // printf("Be4Opt:\n");
        // for(int i = 0; i < WINDOW_SIZE; i++)
        // {
        //     for(int j = 0; j < SwTimeStep[i].size(); j++)
        //     {
        //         printf("State %2d / %2d: Time: %6.3f. P: %6.3f, %6.3f, %6.3f. V: %6.3f, %6.3f, %6.3f\n"
        //                  "               Time: %6.3f. P: %6.3f, %6.3f, %6.3f. V: %6.3f, %6.3f, %6.3f\n",
        //                i, j, SwTimeStep[i][j].start_time, ssPos[i][j].x(), ssPos[i][j].y(), ssPos[i][j].z(), ssVel[i][j].x(), ssVel[i][j].y(), ssVel[i][j].z(),
        //                      SwTimeStep[i][j].final_time, sfPos[i][j].x(), sfPos[i][j].y(), sfPos[i][j].z(), sfVel[i][j].x(), sfVel[i][j].y(), sfVel[i][j].z());
        //     }
        // }

        tt_buildceres.Toc();
        
        TicToc tt_solve;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        tt_solve.Toc();

        Util::ComputeCeresCost(res_ids_surf, cost_surf_final, problem);
        Util::ComputeCeresCost(res_ids_pimu, cost_pimu_final, problem);

        // Load values from params to state
        for(int i = 0; i < WINDOW_SIZE; i++)
        {
            for(int j = 0; j < SwTimeStep[i].size(); j++)
            {
                // Load the state at the start time of each segment
                double ss_time = SwTimeStep[i][j].start_time;
                loader.CopyParamToState(PARAM_POSE[ss_time], PARAM_VELO[ss_time], PARAM_BIAS[ss_time],
                                        ssPos[i][j], ssQua[i][j], ssVel[i][j], ssBia[i][j], ssBig[i][j]);    

                // Load the state at the final time of each segment
                double sf_time = SwTimeStep[i][j].final_time;
                loader.CopyParamToState(PARAM_POSE[sf_time], PARAM_VELO[sf_time], PARAM_BIAS[sf_time],
                                        sfPos[i][j], sfQua[i][j], sfVel[i][j], sfBia[i][j], sfBig[i][j]);
            }
        }

        // printf("AftOpt:\n");
        // for(int i = 0; i < WINDOW_SIZE; i++)
        // {
        //     for(int j = 0; j < SwTimeStep[i].size(); j++)
        //     {
        //         printf("State %2d / %2d: Time: %6.3f. P: %6.3f, %6.3f, %6.3f. V: %6.3f, %6.3f, %6.3f\n"
        //                  "               Time: %6.3f. P: %6.3f, %6.3f, %6.3f. V: %6.3f, %6.3f, %6.3f\n",
        //                i, j, SwTimeStep[i][j].start_time, ssPos[i][j].x(), ssPos[i][j].y(), ssPos[i][j].z(), ssVel[i][j].x(), ssVel[i][j].y(), ssVel[i][j].z(),
        //                      SwTimeStep[i][j].final_time, sfPos[i][j].x(), sfPos[i][j].y(), sfPos[i][j].z(), sfVel[i][j].x(), sfVel[i][j].y(), sfVel[i][j].z());
        //     }
        // }

        // Delete the params
        for(auto &param : PARAM_POSE) delete param.second;
        for(auto &param : PARAM_VELO) delete param.second;
        for(auto &param : PARAM_BIAS) delete param.second;
        
        /* #region Load data to the report ------------------------------------------------------*/

        report.surfFactors = res_ids_surf.size();
        report.J0Surf = cost_surf_init;
        report.JKSurf = cost_surf_final;
        
        report.imuFactors = res_ids_pimu.size();
        report.J0Imu = cost_pimu_init;
        report.JKImu = cost_pimu_final;

        report.velFactors = 0;  //res_ids_vel.size();
        report.J0Vel = -1;      //cost_vel_init;
        report.JKVel = -1;      //cost_vel_final;

        report.J0 = summary.initial_cost;
        report.JK = summary.final_cost;
        
        report.Qest.x = sfQua.back().back().x();
        report.Qest.y = sfQua.back().back().y();
        report.Qest.z = sfQua.back().back().z();
        report.Qest.w = sfQua.back().back().w();

        report.Pest.x = sfPos.back().back().x();
        report.Pest.y = sfPos.back().back().y();
        report.Pest.z = sfPos.back().back().z();

        report.Vest.x = sfVel.back().back().x();
        report.Vest.y = sfVel.back().back().y();
        report.Vest.z = sfVel.back().back().z();

        report.Qimu.x = SwPropState.back().back().Q.back().x();
        report.Qimu.y = SwPropState.back().back().Q.back().y();
        report.Qimu.z = SwPropState.back().back().Q.back().z();
        report.Qimu.w = SwPropState.back().back().Q.back().w();

        report.Pimu.x = SwPropState.back().back().P.back().x();
        report.Pimu.y = SwPropState.back().back().P.back().y();
        report.Pimu.z = SwPropState.back().back().P.back().z();

        report.Vimu.x = SwPropState.back().back().V.back().x();
        report.Vimu.y = SwPropState.back().back().V.back().y();
        report.Vimu.z = SwPropState.back().back().V.back().z();

        // Calculate the relative pose to the last keyframe
        PointPose lastKf = KfCloudPose->back();
        myTf tf_W_Blast(lastKf);

        report.lastKfId = (int)(lastKf.intensity);
        myTf tf_Blast_Bcurr = tf_W_Blast.inverse()*myTf(sfQua.back().back(), sfPos.back().back());

        report.Qref.x = tf_Blast_Bcurr.rot.x();
        report.Qref.y = tf_Blast_Bcurr.rot.y();
        report.Qref.z = tf_Blast_Bcurr.rot.z();
        report.Qref.w = tf_Blast_Bcurr.rot.w();
        
        report.Pref.x = tf_Blast_Bcurr.pos.x();
        report.Pref.y = tf_Blast_Bcurr.pos.y();
        report.Pref.z = tf_Blast_Bcurr.pos.z();
        
        report.iters = summary.iterations.size();
        report.tbuildceres = tt_buildceres.GetLastStop();
        report.tslv  = tt_solve.GetLastStop();
        report.trun  = (ros::Time::now() - program_start_time).toSec();

        report.BANum            = baReport.turn;
        report.BAItr            = baReport.pgopt_iter;
        report.BALoopTime       = tt_loopBA.GetLastStop();
        report.BASolveTime      = baReport.pgopt_time;
        report.BARelPoseFactors = baReport.factor_relpose;
        report.BALoopFactors    = baReport.factor_loop;
        report.BAJ0             = baReport.J0;
        report.BAJK             = baReport.JK;
        report.BAJ0RelPose      = baReport.J0_relpose;
        report.BAJKRelPose      = baReport.JK_relpose;
        report.BAJ0Loop         = baReport.J0_loop;
        report.BAJKLoop         = baReport.JK_loop;

        // Publish the assoc cloud
        static ros::Publisher assoc_cloud_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/assoc_cloud", 100);
        Util::publishCloud(assoc_cloud_pub, *assocCloud, ros::Time(SwTimeStep.back().back().final_time), string("world"));
        
        /* #endregion #region Load data to the report -------------------------------------------*/
    }

    void NominateKeyframe()
    {
        int mid_step = 0;//max(0, int(std::floor(WINDOW_SIZE/2.0)));

        static double last_kf_time = SwTimeStep[mid_step].back().final_time;

        double kf_cand_time = SwTimeStep[mid_step].back().final_time;

        CloudPosePtr kfTempPose(new CloudPose());
        *kfTempPose = *KfCloudPose;

        static KdTreeFLANN<PointPose> kdTreeKeyFrames;
        kdTreeKeyFrames.setInputCloud(kfTempPose);

        myTf tf_W_Bcand(sfQua[mid_step].back(), sfPos[mid_step].back());
        PointPose kf_cand = tf_W_Bcand.Pose6D(kf_cand_time);

        int knn_nbrkf = min(10, (int)kfTempPose->size());
        vector<int> knn_idx(knn_nbrkf); vector<float> knn_sq_dis(knn_nbrkf);
        kdTreeKeyFrames.nearestKSearch(kf_cand, knn_nbrkf, knn_idx, knn_sq_dis);
        
        bool far_distance = knn_sq_dis.front() > kf_min_dis*kf_min_dis;
        bool far_angle = true;
        for(int i = 0; i < knn_idx.size(); i++)
        {
            int kf_idx = knn_idx[i];

            // Collect the angle difference
            Quaternionf Qa(kfTempPose->points[kf_idx].qw,
                           kfTempPose->points[kf_idx].qx,
                           kfTempPose->points[kf_idx].qy,
                           kfTempPose->points[kf_idx].qz);

            Quaternionf Qb(kf_cand.qw, kf_cand.qx, kf_cand.qy, kf_cand.qz);

            // If the angle is more than 10 degrees, add this to the key pose
            if (fabs(Util::angleDiff(Qa, Qb)) < kf_min_angle)
            {
                far_angle = false;
                break;
            }
        }
        bool kf_timeout = fabs(kf_cand_time - last_kf_time) > 2.0 && (knn_sq_dis.front() > 0.1*0.1);

        if (far_distance || far_angle || kf_timeout)
        {
            last_kf_time = kf_cand_time;

            int pointsTotal = SwCloudDsk[mid_step]->size();
            ROS_ASSERT(SwLidarCoef[mid_step].size() == pointsTotal*surfel_query_depth);

            // Create a local map
            CloudXYZITPtr localMap(new CloudXYZIT());

            for(int i = 0; i < knn_idx.size(); i++)
                *localMap += *KfCloudinW[knn_idx[i]];

            KdTreeFLANN<PointXYZIT> kdTreeLocMap; kdTreeLocMap.setInputCloud(localMap);

            // Evaluate points to admit to the surfel map
            CloudXYZITPtr tempCloud(new CloudXYZIT()); tempCloud->resize(pointsTotal);
            #pragma omp parallel for num_threads(MAX_THREADS)
            for(int i = 0; i < pointsTotal; i++)
            {
                // Use the 1 index because we didn't do any association on 0
                LidarCoef &coef = SwLidarCoef[mid_step][i*surfel_query_depth + surfel_min_depth];
                
                // Confirm the idx
                int idx = coef.ptIdx; ROS_ASSERT(idx == i);

                // Preload the point
                tempCloud->points[idx] = SwCloudDsk[mid_step]->points[idx];
                Vector3d pointInW_ = tf_W_Bcand*Vector3d(tempCloud->points[idx].x, tempCloud->points[idx].y, tempCloud->points[idx].z);
                PointXYZIT pointInW; pointInW.x = pointInW_.x(); pointInW.y = pointInW_.y(); pointInW.z = pointInW_.z();

                // Check to see if it is in some near neighbourhood
                vector<int> nbr_idx; vector<float> nbr_sq_dis;
                kdTreeLocMap.nearestKSearch(pointInW, surfel_min_point, nbr_idx, nbr_sq_dis);

                // if (i == 4269)
                //     printf("Point: %.3f, %.3f, %.3f. Nbr: %.3f, %.3f, %.3f. Dis: %.3f. DisCal: %.3f\n",
                //             pointInW_.x(), pointInW_.y(), pointInW_.z(),
                //             localMap->points[nbr_idx[0]].x, localMap->points[nbr_idx[0]].y, localMap->points[nbr_idx[0]].z,
                //             nbr_sq_dis[0], (pointInW_ - Vector3d(localMap->points[nbr_idx[0]].x,  localMap->points[nbr_idx[0]].y,  localMap->points[nbr_idx[0]].z)).norm());
                
                if(nbr_sq_dis.front() > pow(0.5*leaf_size, 2)) // An isolated point, fit for admitting
                    tempCloud->points[idx].t = -1;
            }

            CloudXYZITPtr marginalizedCloud(new CloudXYZIT());
            int margCount = 0;
            for(int i = 0; i < pointsTotal; i++)
            {
                // If point didn't find association, marginalize them
                if (tempCloud->points[i].t < 0)
                {
                    marginalizedCloud->push_back(tempCloud->points[i]); margCount++;
                    continue;
                }

                // Add points that are close to the plane
                // if (coef.d2P < leaf_size)
                // {
                //     marginalizedCloud->push_back(SwCloudDsk[mid_step]->points[idx]);
                //     continue;
                // }

                // Add points to surfels that have few points
                // if (coef.surfNp < pow(8, coef.scale))
                // {
                //     marginalizedCloud->push_back(SwCloudDsk[mid_step]->points[idx]);
                //     continue;
                // }
            }

            margPerc = 100.0*margCount / pointsTotal;
            AdmitKeyframe(SwTimeStep[mid_step].back().final_time, sfQua[mid_step].back(), sfPos[mid_step].back(),
                          SwCloudDsk[mid_step], marginalizedCloud);
        }
    }

    void AdmitKeyframe(double t, Quaternd q, Vector3d p, CloudXYZITPtr &cloud, CloudXYZITPtr &marginalizedCloud)
    {
        tt_ufmupdt.Tic();

        KfCloudinB.push_back(CloudXYZITPtr(new CloudXYZIT()));
        KfCloudinW.push_back(CloudXYZITPtr(new CloudXYZIT()));

        *KfCloudinB.back() = *cloud;
        pcl::transformPointCloud(*KfCloudinB.back(), *KfCloudinW.back(), p, q);

        KfCloudPose->push_back(myTf(q, p).Pose6D(t));
        KfCloudPose->points.back().intensity = KfCloudPose->size()-1;   // Use intensity to store keyframe id

        // for(int i = 0; i < KfCloudinB.size(); i++)
        //     printf("KF %d. Size: %d. %d\n",
        //             i, KfCloudinB[i]->size(), KfCloudinW[i]->size());

        // printf("Be4 add: GMap: %d.\n", globalMap->size(), KfCloudinW.back()->size());
        
        // Add keyframe pointcloud to global map
        {
            lock_guard<mutex> lock(global_map_mtx);
            *globalMap += *KfCloudinW.back();
        }

        // Add keyframe pointcloud to surfel map
        if (marginalizedCloud != cloud)
        {
            pcl::transformPointCloud(*marginalizedCloud, *marginalizedCloud, p, q);
            insertCloudToSurfelMap(surfelMap, *marginalizedCloud);
        }
        else
            insertCloudToSurfelMap(surfelMap, *KfCloudinW.back());

        // printf("Af4 add: GMap: %d.\n", globalMap->size(), KfCloudinW.back()->size());

        // Filter global map
        if (KfCloudPose->size() > 1 && publish_map)
        {
            pcl::UniformSampling<PointXYZIT> downsampler;
            downsampler.setRadiusSearch(leaf_size);
            downsampler.setInputCloud(globalMap);
            downsampler.filter(*globalMap);
        }

        Util::publishCloud(kfcloud_pub, marginalizedCloud == cloud
                                        ? *KfCloudinW.back() : *marginalizedCloud,
                                        ros::Time(t), string("world"));
        Util::publishCloud(kfpose_pub,  *KfCloudPose, ros::Time(t), string("world"));

        if (publish_map)
            Util::publishCloud(global_map_pub, *globalMap, ros::Time(t), string("world"));

        tt_ufmupdt.Toc();    
    }

    void AssociateCloudWithMap(deque<TimeSegment> &timeStep, ufoSurfelMap const &Map, mytf tf_W_B,
                               CloudXYZITPtr &CloudSkewed, CloudXYZITPtr &CloudDeskewed, CloudXYZITPtr &CloudDeskewedDS,
                               deque<LidarCoef> &CloudCoef, map<int, int> &stat)
    {
        int pointsCount = CloudSkewed->points.size();
        if (CloudCoef.size() != pointsCount * surfel_query_depth)
        {
            // Initialize the coefficent buffer
            CloudCoef = deque<LidarCoef>(pointsCount * surfel_query_depth);

            // Check the fitness of points with respect to each surfel
            #pragma omp parallel for num_threads(MAX_THREADS)
            for (int i = 0; i < pointsCount; i++)
            {
                for (int depth = 0; depth < surfel_query_depth; depth++)
                {
                    int idx = i*surfel_query_depth + depth;
                    // CloudCoef[idx].t      = -1;
                    CloudCoef[idx].ptIdx  = i;
                    // CloudCoef[idx].f      = Vector3d(pointRaw.x, pointRaw.y, pointRaw.z);
                    // CloudCoef[idx].fdsk   = Vector3d(pointInB.x, pointInB.y, pointInB.z);
                    // CloudCoef[idx].finW   = Vector3d(pointInW.x, pointInW.y, pointInW.z);
                    // CloudCoef[idx].n      = Vector4d(0, 0, 0, 0);
                    // CloudCoef[idx].u      = -1;
                    // CloudCoef[idx].plnrty = 0;
                }
            }
        }

        #pragma omp parallel for num_threads(MAX_THREADS)
        for(auto &point : CloudDeskewedDS->points)
        {
            int i = (int)(point.intensity);

            for (int depth = 0; depth < surfel_query_depth; depth++)
            {
                int idx = i*surfel_query_depth + depth;
                ROS_ASSERT(i == CloudCoef[idx].ptIdx);
                CloudCoef[idx].t = -1;
                CloudCoef[idx].d2P = -1;
            }
            
            // Set default value
            PointXYZIT pointRaw = CloudSkewed->points[i];
            PointXYZIT pointInB = CloudDeskewed->points[i];
            PointXYZIT pointInW = Util::transform_point(tf_W_B, pointInB);

            if(!Util::PointIsValid(pointInB) || pointInB.t < 0)
            {
                // printf(KRED "Invalid surf point!: %f, %f, %f\n" RESET, pointInB.x, pointInB.y, pointInB.z);
                pointInB.x = 0; pointInB.y = 0; pointInB.z = 0; pointInB.intensity = 0;
                continue;
            }

            // Query the surfel map with predicates
            namespace ufopred = ufo::map::predicate;
            auto pred = ufopred::HasSurfel()
                     && ufopred::DepthMin(surfel_min_depth)
                     && ufopred::DepthMax(surfel_query_depth - 1)
                     && ufopred::NumSurfelPointsMin(surfel_min_point)
                     && ufopred::SurfelPlanarityMin(0.2)    // At this stage, we still search for low planarity surfels to avoid losing track in narrow passages
                     && ufopred::Intersects(ufo::geometry::Sphere(ufo::map::Point3(pointInW.x, pointInW.y, pointInW.z), surfel_intsect_rad));

            vector<int> closest_depth(surfel_query_depth, -1);
            vector<int> closest_npoints(surfel_query_depth, -1);
            vector<double> closest_d2pln(surfel_query_depth, -1);
            vector<double> closest_plnrt(surfel_query_depth, -1);
            vector<Vector4d> closest_plane(surfel_query_depth, Vector4d(0, 0, 0, 0));
            vector<Vector3d> closest_eigen(surfel_query_depth, Vector3d(0, 0, 0));

            for (auto const &node : Map.query(pred))
            {
                auto const& surfel = Map.getSurfel(node);

                double planarity = surfel.getPlanarity();
                // If node depth is higher than the second level, only admit highly planar ones
                if (node.depth() > 1 && planarity < surfel_min_plnrty)  // Keeping low planarity surfels in the second level as backup for losing track
                    continue;

                int depth     = node.depth();
                int numPoint  = surfel.getNumPoints();
                Vector3d mean = ufo::math::toEigen(surfel.getMean());
                Vector3d norm = ufo::math::toEigen(surfel.getNormal());
                Vector3d eig  = ufo::math::toEigen(surfel.getEigenValues());

                if(planarity < 0 || planarity > 1.0)
                {
                    Vector3d sum = ufo::math::toEigen(surfel.getSum());
                    auto sumSq = surfel.getSumSquares();

                    printf("%sInvalid planarity: %f. Depth: %d. Numpoint: %d. Sum: %f, %f, %f. Sumsq: %f, %f, %f, %f, %f, %f. Eig: %f, %f, %f\n" RESET,
                           node.depth() == 0 ? KRED : KMAG,
                           planarity, node.depth(), numPoint,
                           sum.x(), sum.y(), sum.z(),
                           sumSq[0], sumSq[1], sumSq[2], sumSq[3], sumSq[4], sumSq[5],
                           eig(0), eig(1), eig(2)
                          );
                    continue;
                }

                // ROS_ASSERT_MSG(planarity >= 0 && planarity <= 1.0, "plnrty: %f\n", planarity);
                double d2pln = fabs(norm.dot(Vector3d(pointInW.x, pointInW.y, pointInW.z) - mean));

                if (closest_d2pln[depth] == -1 || d2pln < closest_d2pln[depth])
                {
                    closest_depth[depth]   = depth;
                    closest_npoints[depth] = numPoint;
                    closest_d2pln[depth]   = d2pln;
                    closest_plnrt[depth]   = planarity;
                    closest_eigen[depth]   = eig;
                    closest_plane[depth]  << norm, -norm.dot(mean);
                }
            }

            bool point_associated = false;
            for (int depth = 0; depth < surfel_query_depth; depth++)
            {
                // Write down the d2p for the original point
                if (depth == surfel_min_depth)
                    CloudCoef[i*surfel_query_depth + depth].d2P = closest_d2pln[depth];

                if (closest_d2pln[depth] > dis_to_surfel_max || closest_d2pln[depth] == -1 || point_associated)
                    continue;

                double score = (1 - 0.9 * closest_d2pln[depth] / Util::pointDistance(pointInB))*closest_plnrt[depth];

                // Weightage based on how close the point is to the plane
                if (score > score_min)
                {
                    LidarCoef &coef = CloudCoef[i*surfel_query_depth + depth];

                    coef.t      = timeStep.front().start_time + pointRaw.t;
                    coef.n      = score*closest_plane[depth];
                    coef.scale  = depth;
                    coef.surfNp = closest_npoints[depth];
                    coef.plnrty = closest_plnrt[depth];
                    coef.d2P    = closest_d2pln[depth];
                    coef.f      = Vector3d(pointRaw.x, pointRaw.y, pointRaw.z);
                    coef.fdsk   = Vector3d(pointInB.x, pointInB.y, pointInB.z);
                    coef.finW   = Vector3d(pointInW.x, pointInW.y, pointInW.z);
                    
                    for(auto &seg : timeStep)
                    {
                        if(seg.start_time <= coef.t && coef.t <= seg.final_time)
                        {
                            coef.dt = seg.dt();
                            coef.u  = seg.start_time;
                            coef.s  = (coef.t - seg.start_time)/seg.dt();
                            break;
                        }
                    }

                    point_associated = true;
                }
            }
        }

        // Copy temp data to the buffer
        for(int i = 0; i < pointsCount; i++)
        {
            // // Sort the coefficients by planarity
            // struct comparePlanarity
            // {
            //     bool const operator()(LidarCoef a, LidarCoef b) const
            //     {
            //         return (a.plnrty > b.plnrty);
            //     }
            // };
            // std::sort(CloudCoefTemp.begin() + i*surfel_query_depth,
            //           CloudCoefTemp.begin() + (i+1)*surfel_query_depth, comparePlanarity());
            
            for (int depth = 0; depth < surfel_query_depth; depth++)
            {
                auto &coef = CloudCoef[i*surfel_query_depth + depth];
                if (coef.t >= 0)
                {
                    // CloudCoef.push_back(coef);
                    stat[coef.scale] += 1;
                    // break;
                }
            }
        }
    }

    void makeDVAReport(deque<map<int, int>> &stats, map<int, int> &DVA, int &total, string &DVAReport)
    {
        DVA.clear();
        total = 0;
        DVAReport = "";

        for(auto &stepdva : stats)
        {
            for(auto &dva : stepdva)
            {
                total += dva.second;
                DVA[dva.first] += dva.second;
            }
        }
        
        for(auto &dva : DVA)
            DVAReport += myprintf("[%2d, %6d] ", dva.first, dva.second);
    }

    void DetectLoop()
    {
        // For visualization
        static ros::Publisher loop_kf_nbr_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/loop_kf_nbr", 100);
        CloudPosePtr loopKfNbr(new CloudPose());

        static ros::Publisher loop_currkf_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/loop_currkf", 100);
        CloudPosePtr loopCurrKf(new CloudPose());

        static ros::Publisher loop_prevkf_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/loop_prevkf", 100);
        CloudPosePtr loopPrevKf(new CloudPose());

        static ros::Publisher loop_currCloud_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/loop_curr_cloud", 100);
        static ros::Publisher loop_prevCloud_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/loop_prev_cloud", 100);
        static ros::Publisher loop_currCloud_refined_pub = nh_ptr->advertise<sensor_msgs::PointCloud2>("/loop_curr_refined_cloud", 100);

        // Extract the current pose
        int currPoseId = (int)(KfCloudPose->points.back().intensity);
        PointPose currPose = KfCloudPose->points[currPoseId];
        CloudXYZITPtr currCloudInB(new CloudXYZIT()); CloudXYZITPtr currCloudInW(new CloudXYZIT());
        *currCloudInB = *KfCloudinB[currPoseId];
        *currCloudInW = *KfCloudinW[currPoseId];

        // Search for the nearest neighbours
        vector<int> knn_idx(loop_kf_nbr); vector<float> knn_sq_dis(loop_kf_nbr);
        static KdTreeFLANN<PointPose> kdTreeKeyFrames;
        kdTreeKeyFrames.setInputCloud(KfCloudPose);
        kdTreeKeyFrames.nearestKSearch(currPose, loop_kf_nbr, knn_idx, knn_sq_dis);

        // Publish the current keyframe
        loopCurrKf->push_back(currPose);
        if (loopCurrKf->size() > 0)
            Util::publishCloud(loop_currkf_pub, *loopCurrKf, ros::Time(currPose.t), string("world"));

        // Find the oldest index in the neigborhood
        int prevPoseId = -1;
        PointPose prevPose;
        CloudXYZITPtr prevCloudInW(new CloudXYZIT());

        for (auto nbr_idx : knn_idx)
        {
            PointPose &kfPose = KfCloudPose->points[nbr_idx];
            loopKfNbr->push_back(kfPose);

            ROS_ASSERT(nbr_idx == (int)(kfPose.intensity));
            if (prevPoseId == -1 || nbr_idx < prevPoseId)
                prevPoseId = nbr_idx;
        }

        // Publish the nbr kf for visualization
        if (loopKfNbr->size() > 0)
            Util::publishCloud(loop_kf_nbr_pub, *loopKfNbr, ros::Time(currPose.t), string("world"));

        static int LAST_KF_COUNT = KfCloudPose->size();

        // Only do the check every 5 keyframes
        int newKfCount = KfCloudPose->size();
        if (newKfCount - LAST_KF_COUNT < 5 || newKfCount <= loop_kf_nbr)
            return;
        LAST_KF_COUNT = newKfCount;

        // If new loop is too close to last loop in time, skip
        if (!loopPairs.empty())
        {
            double time_since_lastloop = fabs(KfCloudPose->points.back().t - KfCloudPose->points[loopPairs.back().currPoseId].t);
            // printf("Time since last loop: %f\n", time_since_lastloop);

            if (time_since_lastloop < loop_time_mindiff)
                return;
        }

        double time_nbr_diff = fabs(KfCloudPose->points[currPoseId].t - KfCloudPose->points[prevPoseId].t);
        // printf("Time nbr diff: %f\n", time_nbr_diff);

        // Return if no neighbour found, or the two poses are too close in time
        if (prevPoseId == -1 || time_nbr_diff < loop_time_mindiff || abs(currPoseId - prevPoseId) < loop_kf_nbr)
            return;
        else
            prevPose = KfCloudPose->points[prevPoseId];

        // Previous pose detected, build the previous local map

        // Find the range of keyframe Ids
        int bId = prevPoseId; int fId = prevPoseId; int span = fId - bId;
        while(span < loop_kf_nbr)
        {
            bId = max(0, bId - 1);
            fId = min(fId + 1, currPoseId - 1);

            int new_span = fId - bId;

            if ( new_span == span || new_span >= loop_kf_nbr )
                break;
            else
                span = new_span;
        }

        // Extract the keyframe pointcloud around the reference pose
        for(int kfId = bId; kfId < fId; kfId++)
        {
            loopPrevKf->push_back(KfCloudPose->points[kfId]);
            *prevCloudInW += *KfCloudinW[kfId];
        }

        // Publish previous keyframe for vizualization
        if (loopPrevKf->size() > 0)
            Util::publishCloud(loop_prevkf_pub, *loopPrevKf, ros::Time(currPose.t), string("world"));

        // Downsample the pointclouds
        VoxelGrid<PointXYZIT> downsampler;
        double voxel_size = max(leaf_size, 0.4);
        downsampler.setLeafSize(voxel_size, voxel_size, voxel_size);

        downsampler.setInputCloud(prevCloudInW);
        downsampler.filter(*prevCloudInW);
        
        downsampler.setInputCloud(currCloudInB);
        downsampler.filter(*currCloudInB);

        // Publish the cloud for visualization
        Util::publishCloud(loop_prevCloud_pub, *prevCloudInW, ros::Time(currPose.t), string("world"));
        Util::publishCloud(loop_currCloud_pub, *currCloudInW, ros::Time(currPose.t), string("world"));

        // Check match by ICP
        myTf tf_W_Bcurr_start = myTf(currPose);
        myTf tf_W_Bcurr_final = tf_W_Bcurr_start; Matrix4f tfm_W_Bcurr_final;

        bool icp_passed = false; double icpFitnessRes = -1; double icpCheckTime = -1;
        icp_passed =    CheckICP(prevCloudInW, currCloudInB,
                                 tf_W_Bcurr_start.cast<float>().tfMat(), tfm_W_Bcurr_final,
                                 histDis, icpMaxIter, icpFitnessThres, icpFitnessRes, icpCheckTime);
        lastICPFn = icpFitnessRes;

        // Return if icp check fails
        if (!icp_passed)
            return;

        tf_W_Bcurr_final = myTf(tfm_W_Bcurr_final).cast<double>();

        printf("%sICP %s. T_W(%03d)_B(%03d). Fn: %.3f. icpTime: %.3f.\n"
               "Start: Pos: %f, %f, %f. YPR: %f, %f, %f\n"
               "Final: Pos: %f, %f, %f. YPR: %f, %f, %f\n"
                RESET,
                icp_passed ? KBLU : KRED, icp_passed ? "passed" : "failed", prevPoseId, currPoseId, icpFitnessRes, icpCheckTime,
                tf_W_Bcurr_start.pos.x(), tf_W_Bcurr_start.pos.y(), tf_W_Bcurr_start.pos.z(),
                tf_W_Bcurr_start.yaw(),   tf_W_Bcurr_start.pitch(), tf_W_Bcurr_start.roll(),
                tf_W_Bcurr_final.pos.x(), tf_W_Bcurr_final.pos.y(), tf_W_Bcurr_final.pos.z(),
                tf_W_Bcurr_final.yaw(),   tf_W_Bcurr_final.pitch(), tf_W_Bcurr_final.roll());

        // Add the loop to buffer
        loopPairs.push_back(LoopPrior(prevPoseId, currPoseId, 1e-3, icpFitnessRes,
                                      mytf(prevPose).inverse()*tf_W_Bcurr_final));

        // Publish the transform current cloud
        pcl::transformPointCloud(*currCloudInB, *currCloudInW, tf_W_Bcurr_final.pos, tf_W_Bcurr_final.rot);
        Util::publishCloud(loop_currCloud_refined_pub, *currCloudInW, ros::Time(currPose.t), string("world"));

    }

    bool CheckICP(CloudXYZITPtr &ref_pcl, CloudXYZITPtr &src_pcl, Matrix4f relPosIcpGuess, Matrix4f &relPosIcpEst,
                  double hisKFSearchRadius, int icp_max_iters, double icpFitnessThres, double &icpFitnessRes, double &ICPtime)
    {

        /* #region Calculate the relative pose constraint ---------------------------------------------------------------*/

        TicToc tt_icp;

        pcl::IterativeClosestPoint<PointXYZIT, PointXYZIT> icp;
        icp.setMaxCorrespondenceDistance(hisKFSearchRadius * 2);
        icp.setMaximumIterations(icp_max_iters);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        
        icp.setInputSource(src_pcl);
        icp.setInputTarget(ref_pcl);

        CloudXYZITPtr aligned_result(new CloudXYZIT());

        icp.align(*aligned_result, relPosIcpGuess);

        bool icp_passed   = false;
        bool icpconverged = icp.hasConverged();
        icpFitnessRes     = icp.getFitnessScore();
        relPosIcpEst      = icp.getFinalTransformation();

        ICPtime = tt_icp.Toc();

        if (!icpconverged || icpFitnessRes > icpFitnessThres)
        {
            // if (extended_report)
            // {
            //     printf(KRED "\tICP time: %9.3f ms. ICP %s. Fitness: %9.3f, threshold: %3.1f\n" RESET,
            //     tt_icp.GetLastStop(),
            //     icpconverged ? "converged" : "fails to converge",
            //     icpFitnessRes, icpFitnessThres);
            // }
        }
        else
        {
            // if (extended_report)
            // {
            //     printf(KBLU "\tICP time: %9.3f ms. ICP %s. Fitness: %9.3f, threshold: %3.1f\n" RESET,
            //         tt_icp.GetLastStop(),
            //         icpconverged ? "converged" : "fails to converge",
            //         icpFitnessRes, icpFitnessThres);
            // }

            icp_passed = true;
        }

        return icp_passed;

        /* #endregion Calculate the relative pose constraint ------------------------------------------------------------*/        

    }

    void BundleAdjustment(BAReport &report)
    {
        static int LAST_LOOP_COUNT = loopPairs.size();
        int newLoopCount = loopPairs.size();

        // Return if no new loop detected
        if (newLoopCount - LAST_LOOP_COUNT < 1)
            return;
        LAST_LOOP_COUNT = newLoopCount;

        // Solve the pose graph optimization problem
        OptimizePoseGraph(KfCloudPose, loopPairs, report);

        TicToc tt_rebuildmap;

        // Recompute the keyframe pointclouds
        #pragma omp parallel for num_threads(MAX_THREADS)
        for(int i = 0; i < KfCloudPose->size(); i++)
        {
            myTf tf_W_B(KfCloudPose->points[i]);
            pcl::transformPointCloud(*KfCloudinB[i], *KfCloudinW[i], tf_W_B.pos, tf_W_B.rot);
        }

        // Recompute the globalmap and ufomap
        {
            lock_guard<mutex> lock(global_map_mtx);
            globalMap->clear();
            surfelMap.clear();

            for(int i = 0; i < KfCloudPose->size(); i++)
                *globalMap += *KfCloudinW[i];

            // Downsample the global map
            pcl::UniformSampling<PointXYZIT> downsampler;
            downsampler.setRadiusSearch(leaf_size);
            downsampler.setInputCloud(globalMap);
            downsampler.filter(*globalMap);

            Util::publishCloud(global_map_pub, *globalMap, ros::Time(KfCloudPose->points.back().t), string("world"));

            // Build the surfelmap
            insertCloudToSurfelMap(surfelMap, *globalMap);
            
            // Increment the ufomap version
            ufomap_version++;
        }

        tt_rebuildmap.Toc();

        report.rebuildmap_time = tt_rebuildmap.GetLastStop();
    }

    void OptimizePoseGraph(CloudPosePtr &kfCloud, const deque<LoopPrior> &loops, BAReport &report)
    {
        TicToc tt_pgopt;

        static int BA_NUM = -1;

        int KF_NUM = kfCloud->size();

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.num_threads = omp_get_max_threads();
        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

        // Create params and load data
        double **PARAM_POSE = new double *[KF_NUM];
        for(int i = 0; i < KF_NUM; i++)
        {
            PARAM_POSE[i] = new double[7];

            PARAM_POSE[i][0] = kfCloud->points[i].x;
            PARAM_POSE[i][1] = kfCloud->points[i].y;
            PARAM_POSE[i][2] = kfCloud->points[i].z;
            PARAM_POSE[i][3] = kfCloud->points[i].qx;
            PARAM_POSE[i][4] = kfCloud->points[i].qy;
            PARAM_POSE[i][5] = kfCloud->points[i].qz;
            PARAM_POSE[i][6] = kfCloud->points[i].qw;

            ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
            problem.AddParameterBlock(PARAM_POSE[i], 7, local_parameterization);

            // Fix the last pose
            if (i == KF_NUM - 1)
                problem.SetParameterBlockConstant(PARAM_POSE[i]);
        }

        // Add relative pose factors
        vector<ceres::internal::ResidualBlock *> res_ids_relpose;
        double cost_relpose_init = -1, cost_relpose_final = -1;
        for(int i = 1; i < KF_NUM; i++)
        {
            for (int j = 1; j < rib_edge; j++)
            {
                int jj = j;

                // Make an edge to the first pose for the poses with 5 steps
                if (i - j <= 0)
                    jj = i;

                myTf pose_i = myTf(kfCloud->points[i]);
                myTf pose_j = myTf(kfCloud->points[i-jj]);

                RelOdomFactor* relodomfactor = new RelOdomFactor(pose_i.pos, pose_j.pos, pose_i.rot, pose_j.rot,
                                                                 odom_q_noise, odom_p_noise);
                ceres::internal::ResidualBlock *res_id =  problem.AddResidualBlock(relodomfactor, NULL, PARAM_POSE[i], PARAM_POSE[i-jj]);
                res_ids_relpose.push_back(res_id);
            }
        }

        // Add loop factors
        vector<ceres::internal::ResidualBlock *> res_ids_loop;
        double cost_loop_init = -1, cost_loop_final = -1;
        for(auto &loop_edge : loopPairs)
        {
            // printf("Loop Factor: prev %d, curr: %d\n", loop_edge.prev_idx, loop_edge.curr_idx);
            
            int &curr_idx = loop_edge.currPoseId;
            int &prev_idx = loop_edge.prevPoseId;

            double &JKavr = loop_edge.JKavr;
            double &IcpFn = loop_edge.IcpFn;

            myTf pose_i = myTf(kfCloud->points[prev_idx]);
            myTf pose_j = myTf(kfCloud->points[prev_idx])*loop_edge.tf_Bp_Bc;

            RelOdomFactor* relodomfactor = new RelOdomFactor(pose_i.pos, pose_j.pos, pose_i.rot, pose_j.rot,
                                                             odom_q_noise*loop_weight, odom_p_noise*loop_weight);
            ceres::internal::ResidualBlock *res_id = problem.AddResidualBlock(relodomfactor, NULL, PARAM_POSE[prev_idx], PARAM_POSE[curr_idx]);
            res_ids_loop.push_back(res_id);
        }
        
        Util::ComputeCeresCost(res_ids_relpose, cost_relpose_init, problem);
        Util::ComputeCeresCost(res_ids_loop, cost_loop_init, problem);
        
        ceres::Solve(options, &problem, &summary);

        Util::ComputeCeresCost(res_ids_relpose, cost_relpose_final, problem);
        Util::ComputeCeresCost(res_ids_loop, cost_loop_final, problem);

        // Return the keyframe result
        for(int i = 0; i < KF_NUM; i++)
        {
            kfCloud->points[i].x  = PARAM_POSE[i][0];
            kfCloud->points[i].y  = PARAM_POSE[i][1];
            kfCloud->points[i].z  = PARAM_POSE[i][2];
            kfCloud->points[i].qx = PARAM_POSE[i][3];
            kfCloud->points[i].qy = PARAM_POSE[i][4];
            kfCloud->points[i].qz = PARAM_POSE[i][5];
            kfCloud->points[i].qw = PARAM_POSE[i][6];
        }
        
        baReport.turn           = (BA_NUM++);
        baReport.pgopt_time     = tt_pgopt.Toc();
        baReport.pgopt_iter     = summary.iterations.size();
        baReport.factor_relpose = res_ids_relpose.size();
        baReport.factor_loop    = res_ids_loop.size();
        baReport.J0             = summary.initial_cost;
        baReport.JK             = summary.final_cost;
        baReport.J0_relpose     = cost_relpose_init;
        baReport.JK_relpose     = cost_relpose_final;
        baReport.J0_loop        = cost_loop_init;
        baReport.JK_loop        = cost_loop_final;
    }

    void VisualizeLoop()
    {
        // Visualize the loop
        static visualization_msgs::Marker loop_marker; static bool loop_marker_inited = false;
        static ros::Publisher loop_marker_pub = nh_ptr->advertise<visualization_msgs::Marker>("/loop_marker", 100);
        static std_msgs::ColorRGBA color;

        if (!loop_marker_inited)
        {
            // Set up the loop marker
            loop_marker_inited = true;
            loop_marker.header.frame_id = "world";
            loop_marker.ns       = "loop_marker";
            loop_marker.type     = visualization_msgs::Marker::LINE_LIST;
            loop_marker.action   = visualization_msgs::Marker::ADD;
            loop_marker.pose.orientation.w = 1.0;
            loop_marker.lifetime = ros::Duration(0);
            loop_marker.id       = 0;

            loop_marker.scale.x = 0.3; loop_marker.scale.y = 0.3; loop_marker.scale.z = 0.3;
            loop_marker.color.r = 0.0; loop_marker.color.g = 1.0; loop_marker.color.b = 1.0; loop_marker.color.a = 1.0;
            
            color.r = 0.0; color.g = 1.0; color.b = 1.0; color.a = 1.0;
        }

        loop_marker.points.clear();
        loop_marker.colors.clear();
        for(int i = 0; i < loopPairs.size(); i++)
        {
            int curr_idx = loopPairs[i].currPoseId;
            int prev_idx = loopPairs[i].prevPoseId;

            auto pose_curr = KfCloudPose->points[curr_idx];
            auto pose_prev = KfCloudPose->points[prev_idx];

            // Updating the line segments------------------------
            
            geometry_msgs::Point point;

            point.x = pose_curr.x;
            point.y = pose_curr.y;
            point.z = pose_curr.z;

            loop_marker.points.push_back(point);
            loop_marker.colors.push_back(color);

            point.x = pose_prev.x;
            point.y = pose_prev.y;
            point.z = pose_prev.z;

            loop_marker.points.push_back(point);
            loop_marker.colors.push_back(color);
        }
        // Publish the loop markers
        loop_marker_pub.publish(loop_marker);
    }

    void VisualizeSwTraj()
    {
        // Publish the traj
        static ros::Publisher swprop_viz_pub = nh_ptr->advertise<nav_msgs::Path>("/swprop_traj", 100);
        static ros::Publisher lastcloud_pub  = nh_ptr->advertise<sensor_msgs::PointCloud2>("/lastcloud", 100);

        // VisualizeSwTraj(swknots_viz_pub, GlobalTraj, SwTimeStep.front(), SwTimeStep.back(), "world");
        // VisualizeSwTraj(allknots_viz_pub, GlobalTraj, GlobalTraj->minTime(), SwTimeStep.back(), "world");

        double time_stamp = SwTimeStep.back().back().final_time;

        // Publish the propagated poses
        nav_msgs::Path prop_path;
        prop_path.header.frame_id = "world";
        prop_path.header.stamp = ros::Time(time_stamp);
        for(int i = 0; i < WINDOW_SIZE; i++)
        {
            for(int j = 0; j < SwPropState[i].size(); j++)
            {
                for (int k = 0; k < SwPropState[i][j].size(); k++)
                {
                    geometry_msgs::PoseStamped msg;
                    msg.header.frame_id = "world";
                    msg.header.stamp = ros::Time(SwPropState[i][j].t[k]);
                    msg.pose.position.x = SwPropState[i][j].P[k].x();
                    msg.pose.position.y = SwPropState[i][j].P[k].y();
                    msg.pose.position.z = SwPropState[i][j].P[k].z();
                    
                    prop_path.poses.push_back(msg);
                }
            }
        }
        swprop_viz_pub.publish(prop_path);

        // Publish the odom
        static ros::Publisher opt_odom_pub = nh_ptr->advertise<nav_msgs::Odometry>("/opt_odom", 100);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time(time_stamp);
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id  = "body";

        odom_msg.pose.pose.position.x = sfPos.back().back().x();
        odom_msg.pose.pose.position.y = sfPos.back().back().y();
        odom_msg.pose.pose.position.z = sfPos.back().back().z();

        odom_msg.pose.pose.orientation.x = sfQua.back().back().x();
        odom_msg.pose.pose.orientation.y = sfQua.back().back().y();
        odom_msg.pose.pose.orientation.z = sfQua.back().back().z();
        odom_msg.pose.pose.orientation.w = sfQua.back().back().w();

        opt_odom_pub.publish(odom_msg);

        // Publish the latest cloud transformed
        CloudXYZITPtr latestCloud(new CloudXYZIT());
        pcl::transformPointCloud(*SwCloudDsk.back(), *latestCloud, sfPos.back().back(), sfQua.back().back());
        Util::publishCloud(lastcloud_pub, *latestCloud, ros::Time(time_stamp), string("world"));

        // Publish the transform
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x,
                                        odom_msg.pose.pose.position.y,
                                        odom_msg.pose.pose.position.z));
        tf::Quaternion q(odom_msg.pose.pose.orientation.x,
                         odom_msg.pose.pose.orientation.y,
                         odom_msg.pose.pose.orientation.z,
                         odom_msg.pose.pose.orientation.w);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
    }

    void SlideWindowForward()
    {
        // Pop the states and replicate the final state
        ssQua.pop_front(); ssQua.push_back(deque<Quaternd>(N_SUB_SEG, sfQua.back().back()));
        ssPos.pop_front(); ssPos.push_back(deque<Vector3d>(N_SUB_SEG, sfPos.back().back()));
        ssVel.pop_front(); ssVel.push_back(deque<Vector3d>(N_SUB_SEG, sfVel.back().back()));
        ssBia.pop_front(); ssBia.push_back(deque<Vector3d>(N_SUB_SEG, sfBia.back().back()));
        ssBig.pop_front(); ssBig.push_back(deque<Vector3d>(N_SUB_SEG, sfBig.back().back()));

        sfQua.pop_front(); sfQua.push_back(ssQua.back());
        sfPos.pop_front(); sfPos.push_back(ssPos.back());
        sfVel.pop_front(); sfVel.push_back(ssVel.back());
        sfBia.pop_front(); sfBia.push_back(ssBia.back());
        sfBig.pop_front(); sfBig.push_back(ssBig.back());

        // Pop the buffers
        SwTimeStep.pop_front();
        SwCloud.pop_front();
        SwCloudDsk.pop_front();
        SwCloudDskDS.pop_front();
        SwLidarCoef.pop_front();
        SwDepVsAssoc.pop_front();
        SwImuBundle.pop_front();
        SwPropState.pop_front();
    }

    bool PublishGlobalMaps(slict::globalMapsPublish::Request &req, slict::globalMapsPublish::Response &res)
    {
        // Log and save the trajectory
        SaveTrajLog();

        // Publish the full map
        Util::publishCloud(global_map_pub, *globalMap, ros::Time(KfCloudPose->points.back().t), string("world"));

        res.result = 1;
        return true;
    }

    void SaveTrajLog()
    {
        printf(KYEL "Logging the map start ...\n" RESET);

        printf("Logging cloud pose: %s.\n", (log_dir + "/KfCloudPose.pcd").c_str());
        PCDWriter writer; writer.writeASCII(log_dir + "/KfCloudPose.pcd", *KfCloudPose, 18); 

        printf(KGRN "Logging the map completed.\n" RESET);

        printf(KYEL "Logging the loop ...\n" RESET);

        std::ofstream loop_log_file;
        loop_log_file.open(log_dir + "/loop_log.csv");
        loop_log_file.precision(std::numeric_limits<double>::digits10 + 1);

        for(auto &loop : loopPairs)
        {
            loop_log_file << loop.currPoseId << ", "
                          << loop.prevPoseId << ", "
                          << loop.JKavr << ", "
                          << loop.IcpFn << ", "
                          << loop.tf_Bp_Bc.pos(0) << ", "
                          << loop.tf_Bp_Bc.pos(1) << ", "
                          << loop.tf_Bp_Bc.pos(2) << ", "
                          << loop.tf_Bp_Bc.rot.x() << ", "
                          << loop.tf_Bp_Bc.rot.y() << ", "
                          << loop.tf_Bp_Bc.rot.z() << ", "
                          << loop.tf_Bp_Bc.rot.w() << endl;
        }

        loop_log_file.close();

        printf(KGRN "Logging the loop completed.\n" RESET);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Estimator");
    ros::NodeHandle nh("~");
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh);

    ROS_INFO(KGRN "----> Estimator Started." RESET);

    Estimator estimator(nh_ptr);

    thread process_data(&Estimator::ProcessData, &estimator); // For processing the image

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    estimator.SaveTrajLog();

    return 0;
}
