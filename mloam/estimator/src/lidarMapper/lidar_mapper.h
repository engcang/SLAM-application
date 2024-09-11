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

#pragma once

#include <glog/logging.h>
#include <gflags/gflags.h>

#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <map>
#include <cassert>
#include <algorithm>
#include <utility>
#include <omp.h>
#include <signal.h>
#include <time.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>

#include "common/common.hpp"
#include "common/types/type.h"
#include "common/publisher.hpp"
#include "common/color.hpp"

#include "mloam_msgs/Extrinsics.h"
#include "mloam_msgs/Keyframes.h"

#include "mloam_pcl/point_with_cov.hpp"
#include "mloam_pcl/voxel_grid_covariance_mloam.h"
#include "mloam_pcl/voxel_grid_covariance_mloam_impl.hpp"

#include "../save_statistics.hpp"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"
#include "../estimator/pose.h"
#include "../estimator/parameters.h"
#include "../featureExtract/feature_extract.hpp"
#include "../factor/lidar_map_factor.hpp"
#include "../factor/pose_local_parameterization.h"
#include "../factor/impl_loss_function.hpp"
#include "../factor/impl_callback.hpp"
#include "associate_uct.hpp"

#define GLOBALMAP_KF_RADIUS 1000.0
#define MAX_FEATURE_SELECT_TIME 20  // 10ms
#define MAX_RANDOM_QUEUE_TIME 20

DEFINE_bool(result_save, true, "save or not save the results");
DEFINE_string(config_file, "config.yaml", "the yaml config file");
DEFINE_string(output_path, "", "the path ouf saving results");
DEFINE_bool(with_ua, true, "with or without the awareness of uncertainty");
DEFINE_string(gf_method, "wo-gf", "good feature selection method: rnd, fps, gd-float, gd-fix");
DEFINE_double(gf_ratio_ini, 1.0, "with or without the good features selection");

FeatureExtract f_extract;

// ****************** main process of lidar mapper
void transformAssociateToMap();

void extractSurroundingKeyFrames();

void downsampleCurrentScan();

void scan2MapOptimization();

void transformUpdate();

void saveKeyframeAndInsertGraph();

void updateKeyframe();

void pubGlobalMap();

void pubPointCloud();

void pubOdometry();

void saveGlobalMap();

// ****************** other operation
void cloudUCTAssociateToMap(const PointICovCloud &cloud_local, PointICovCloud &cloud_global,
                            const Pose &pose_global, const vector<Pose> &pose_ext);

void evalHessian(const ceres::CRSMatrix &jaco, Eigen::Matrix<double, 6, 6> &mat_H);

void evalDegenracy(const Eigen::Matrix<double, 6, 6> &mat_H, PoseLocalParameterization *local_parameterization);

// ****************** active feature selection
class ActiveFeatureSelection
{
public:
    // ****************** good feature selection
    void evaluateFeatJacobianMatching(const Pose &pose_local,
                                      PointPlaneFeature &feature,
                                      const Eigen::Matrix3d &cov_matrix)
    {
        double **param = new double *[1];
        param[0] = new double [SIZE_POSE];
        param[0][0] = pose_local.t_(0);
        param[0][1] = pose_local.t_(1);
        param[0][2] = pose_local.t_(2);
        param[0][3] = pose_local.q_.x();
        param[0][4] = pose_local.q_.y();
        param[0][5] = pose_local.q_.z();
        param[0][6] = pose_local.q_.w();

        double *res = new double[1];
        double **jaco = new double *[1];
        jaco[0] = new double[1 * 7];
        if (feature.type_ == 's')
        {
            LidarMapPlaneNormFactor f(feature.point_, feature.coeffs_, cov_matrix);      
            f.Evaluate(param, res, jaco);
        } 
        else if (feature.type_ == 'c')
        {
            LidarMapEdgeFactor f(feature.point_, feature.coeffs_, cov_matrix);
            f.Evaluate(param, res, jaco);
        }

        double *rho = new double[3];
        double sqr_error = res[0] * res[0] + res[1] * res[1] + res[0] * res[0];
        loss_function_->Evaluate(sqr_error, rho);

        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> mat_jacobian(jaco[0]);
        feature.jaco_ = mat_jacobian.topLeftCorner<1, 6>();
        // feature.jaco_ *= sqrt(std::max(0.0, rho[1])); // TODO
        // LOG_EVERY_N(INFO, 2000) << "error: " << sqrt(sqr_error) << ", rho_der: " << rho[1] 
        //                         << ", logd: " << common::logDet(feature.jaco_.transpose() * feature.jaco_, true);

        delete[] res;
        delete[] rho;
        delete[] jaco[0];
        delete[] jaco;
        delete[] param[0];
        delete[] param;
    }

    void evalFullHessian(const pcl::KdTreeFLANN<PointIWithCov>::Ptr &kdtree_from_map,
                         const PointICovCloud &laser_map,
                         const PointICovCloud &laser_cloud,
                         const Pose &pose_local,
                         const char feature_type,
                         Eigen::Matrix<double, 6, 6> &mat_H,
                         int &feat_num)
    {
        size_t num_all_features = laser_cloud.size();
        std::vector<PointPlaneFeature> all_features(num_all_features);
        // std::vector<Eigen::MatrixXd> v_jaco;
        for (size_t i = 0; i < num_all_features; i++) 
        {
            size_t n_neigh = 5;
            bool b_match = true;
            if (feature_type == 's')
            {
                b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                          laser_map,
                                                          laser_cloud.points[i],
                                                          pose_local,
                                                          all_features[i],
                                                          i,
                                                          n_neigh,
                                                          false);
            } else if (feature_type == 'c')
            {
                b_match = f_extract.matchCornerPointFromMap(kdtree_from_map,
                                                            laser_map,
                                                            laser_cloud.points[i],
                                                            pose_local,
                                                            all_features[i],
                                                            i,
                                                            n_neigh,
                                                            false);
            }
            if (!b_match) continue;
            Eigen::Matrix3d cov_matrix;
            extractCov(laser_cloud.points[i], cov_matrix);
            evaluateFeatJacobianMatching(pose_local, all_features[i], cov_matrix);
            const Eigen::MatrixXd &jaco = all_features[i].jaco_;
            mat_H = mat_H + jaco.transpose() * jaco;
            // v_jaco.push_back(jaco);
            feat_num++;
        }
        // std::ofstream fout((OUTPUT_FOLDER + "jacobian.txt").c_str(), std::ios::out);
        // fout << "jacobian" << std::endl;
        // fout.precision(8);
        // for (size_t i = 0; i < v_jaco.size(); i++)
        //     fout << v_jaco[i] << std::endl;
        // fout.close();
    }

    void goodFeatureMatching(const pcl::KdTreeFLANN<PointIWithCov>::Ptr &kdtree_from_map,
                             const PointICovCloud &laser_map,
                             const PointICovCloud &laser_cloud,
                             const Pose &pose_local,
                             std::vector<PointPlaneFeature> &all_features,
                             std::vector<size_t> &sel_feature_idx,
                             const char feature_type,
                             const string gf_method,
                             const double gf_ratio,
                             Eigen::Matrix<double, 6, 6> &sub_mat_H)
    {
        size_t num_all_features = laser_cloud.size();
        all_features.resize(num_all_features);

        std::vector<size_t> all_feature_idx(num_all_features);
        std::vector<int> feature_visited(num_all_features, -1);
        std::iota(all_feature_idx.begin(), all_feature_idx.end(), 0);

        size_t num_use_features;
        num_use_features = static_cast<size_t>(num_all_features * gf_ratio);
        sel_feature_idx.resize(num_use_features);
        size_t num_sel_features = 0;

        bool b_match;
        double cur_det;
        size_t num_rnd_que;
        TicToc t_sel_feature;
        size_t n_neigh = 5;
        if (gf_method == "wo_gf")
        {
            for (size_t j = 0; j < all_feature_idx.size(); j++)
            {
                size_t que_idx = all_feature_idx[j];
                b_match = false;
                if (feature_type == 's')
                {
                    b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                              laser_map,
                                                              laser_cloud.points[que_idx],
                                                              pose_local,
                                                              all_features[que_idx],
                                                              que_idx,
                                                              n_neigh,
                                                              false);
                }
                else if (feature_type == 'c')
                {
                    b_match = f_extract.matchCornerPointFromMap(kdtree_from_map,
                                                                laser_map,
                                                                laser_cloud.points[que_idx],
                                                                pose_local,
                                                                all_features[que_idx],
                                                                que_idx,
                                                                n_neigh,
                                                                false);
                }
                if (b_match)
                {
                    Eigen::Matrix3d cov_matrix;
                    extractCov(laser_cloud.points[que_idx], cov_matrix);
                    evaluateFeatJacobianMatching(pose_local,
                                                 all_features[que_idx],
                                                 cov_matrix);
                    const Eigen::MatrixXd &jaco = all_features[que_idx].jaco_;
                    sub_mat_H += jaco.transpose() * jaco;

                    sel_feature_idx[num_sel_features] = que_idx;
                    num_sel_features++;
                }
            }
        }  
        else if (gf_method == "rnd")
        {
            while (true)
            {
                if ((num_sel_features >= num_use_features) ||
                    (all_feature_idx.size() == 0) ||
                    (t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME))
                    break;
                // if (num_sel_features >= num_use_features ||
                //     all_feature_idx.size() == 0)
                //     break;
                    
                size_t j = rgi_.geneRandUniform(0, all_feature_idx.size() - 1);
                size_t que_idx = all_feature_idx[j];
                b_match = false;
                if (feature_type == 's')
                {
                    b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                              laser_map,
                                                              laser_cloud.points[que_idx],
                                                              pose_local,
                                                              all_features[que_idx],
                                                              que_idx,
                                                              n_neigh,
                                                              false);
                }
                else if (feature_type == 'c')
                {
                    b_match = f_extract.matchCornerPointFromMap(kdtree_from_map,
                                                                laser_map,
                                                                laser_cloud.points[que_idx],
                                                                pose_local,
                                                                all_features[que_idx],
                                                                que_idx,
                                                                n_neigh,
                                                                false);
                }
                if (b_match)
                {
                    Eigen::Matrix3d cov_matrix;
                    extractCov(laser_cloud.points[que_idx], cov_matrix);
                    evaluateFeatJacobianMatching(pose_local,
                                                 all_features[que_idx],
                                                 cov_matrix);
                    const Eigen::MatrixXd &jaco = all_features[que_idx].jaco_;
                    sub_mat_H += jaco.transpose() * jaco;

                    sel_feature_idx[num_sel_features] = que_idx;
                    num_sel_features++;
                }
                all_feature_idx.erase(all_feature_idx.begin() + j);
            }
        }
        else if (gf_method == "fps")
        {
            size_t que_idx;
            size_t k = rgi_.geneRandUniform(0, all_feature_idx.size() - 1); // randomly select a starting point
            feature_visited[k] = 1;
            size_t cnt_visited = 1;
            PointIWithCov point_old = laser_cloud.points[k]; 
            b_match = false;
            if (feature_type == 's')
            {            
                b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                          laser_map,
                                                          point_old,
                                                          pose_local,
                                                          all_features[k],
                                                          k,
                                                          n_neigh,
                                                          false);
            }
            else if (feature_type == 'c')
            {
                b_match = f_extract.matchCornerPointFromMap(kdtree_from_map,
                                                            laser_map,
                                                            point_old,
                                                            pose_local,
                                                            all_features[k],
                                                            k,
                                                            n_neigh,
                                                            false);
            }
            if (b_match)
            {
                sel_feature_idx[num_sel_features] = k;
                num_sel_features++;
            }            

            std::vector<float> dist(num_all_features, 1e5); // record the minimum distance of each point in set A to each point in set B
            while (true)
            {
                if ((num_sel_features >= num_use_features) ||
                    (all_feature_idx.size() == 0) ||
                    (t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME))
                    break;
                // if ((num_sel_features >= num_use_features) ||
                //     (all_feature_idx.size() == 0) ||
                //     (cnt_visited == num_all_features))
                //     break;

                float best_d = -1;
                size_t best_j = 1;
                for (size_t j = 0; j < num_all_features; j++)
                {
                    if (feature_visited[j] == 1) continue;
                    const PointIWithCov &point_new = laser_cloud.points[j];
                    float d = sqrt(common::sqrSum(point_old.x - point_new.x,
                                                  point_old.y - point_new.y,
                                                  point_old.z - point_new.z));
                    float d2 = std::min(d, dist[j]);
                    dist[j] = d2;
                    best_j = d2 > best_d ? j : best_j;
                    best_d = d2 > best_d ? d2 : best_d;
                }
                que_idx = best_j;
                point_old = laser_cloud.points[que_idx];
                feature_visited[que_idx] = 1;
                cnt_visited++;

                b_match = false;
                if (feature_type == 's')
                {
                    b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                              laser_map,
                                                              point_old,
                                                              pose_local,
                                                              all_features[que_idx],
                                                              que_idx,
                                                              n_neigh,
                                                              false);
                }
                else if (feature_type == 'c')
                {
                    b_match = f_extract.matchCornerPointFromMap(kdtree_from_map,
                                                                laser_map,
                                                                point_old,
                                                                pose_local,
                                                                all_features[que_idx],
                                                                que_idx,
                                                                n_neigh,
                                                                false);
                }
                if (b_match)
                {
                    Eigen::Matrix3d cov_matrix;
                    extractCov(laser_cloud.points[que_idx], cov_matrix);
                    evaluateFeatJacobianMatching(pose_local,
                                                 all_features[que_idx],
                                                 cov_matrix);
                    const Eigen::MatrixXd &jaco = all_features[que_idx].jaco_;
                    sub_mat_H += jaco.transpose() * jaco;

                    sel_feature_idx[num_sel_features] = que_idx;
                    num_sel_features++;
                }            
            }
        }
        else if (gf_method == "gd_fix" || gf_method == "gd_float")
        {
            stringstream ss;
            while (true)
            {
                if ((num_sel_features >= num_use_features) ||
                    (all_feature_idx.size() == 0) || 
                    (t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME))
                    break;
                // if ((num_sel_features >= num_use_features * 0.8) && 
                //     (t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME))
                //     break;
                // if ((num_sel_features >= num_use_features) ||
                //     (all_feature_idx.size() == 0))
                //     break;
                // 2.3 = log(1\epi)
                size_t size_rnd_subset = static_cast<size_t>(1.0 * num_all_features / num_use_features); // 1.0/2.3
                // LOG_EVERY_N(INFO, 20) << "[goodFeatureMatching] size of matrix subset: " << size_rnd_subset;
                std::priority_queue<FeatureWithScore, std::vector<FeatureWithScore>, std::less<FeatureWithScore>> heap_subset;

                while (true)
                {
                    if (all_feature_idx.size() == 0) break;
                    num_rnd_que = 0;
                    size_t j;
                    while (num_rnd_que < MAX_RANDOM_QUEUE_TIME)
                    {
                        j = rgi_.geneRandUniform(0, all_feature_idx.size() - 1);
                        if (feature_visited[j] < int(num_sel_features))
                        {
                            feature_visited[j] = int(num_sel_features);
                            break;
                        }
                        num_rnd_que++;
                    }
                    if (num_rnd_que >= MAX_RANDOM_QUEUE_TIME || t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME)
                        break;

                    size_t que_idx = all_feature_idx[j];
                    if (all_features[que_idx].type_ == 'n')
                    {
                        b_match = false;
                        if (feature_type == 's')
                        {
                            b_match = f_extract.matchSurfPointFromMap(kdtree_from_map,
                                                                      laser_map,
                                                                      laser_cloud.points[que_idx],
                                                                      pose_local,
                                                                      all_features[que_idx],
                                                                      que_idx,
                                                                      n_neigh,
                                                                      false);
                        } 
                        else if (feature_type == 'c')
                        {
                            b_match = f_extract.matchCornerPointFromMap(kdtree_from_map,
                                                                        laser_map,
                                                                        laser_cloud.points[que_idx],
                                                                        pose_local,
                                                                        all_features[que_idx],
                                                                        que_idx,
                                                                        n_neigh,
                                                                        false);
                        }
                        if (b_match) 
                        {
                            Eigen::Matrix3d cov_matrix;
                            extractCov(laser_cloud.points[que_idx], cov_matrix);
                            evaluateFeatJacobianMatching(pose_local,
                                                         all_features[que_idx],
                                                         cov_matrix);
                        } else // not found constraints or outlier constraints
                        {
                            all_feature_idx.erase(all_feature_idx.begin() + j);
                            feature_visited.erase(feature_visited.begin() + j);
                            continue;
                        }
                    }

                    const Eigen::MatrixXd &jaco = all_features[que_idx].jaco_;
                    cur_det = common::logDet(sub_mat_H + jaco.transpose() * jaco, true);
                    heap_subset.push(FeatureWithScore(que_idx, cur_det, jaco));
                    if (heap_subset.size() >= size_rnd_subset)
                    {
                        const FeatureWithScore &fws = heap_subset.top();
                        std::vector<size_t>::iterator iter = std::find(all_feature_idx.begin(), all_feature_idx.end(), fws.idx_);
                        if (iter == all_feature_idx.end())
                        {
                            std::cerr << "[goodFeatureMatching]: not exist feature idx !" << std::endl;
                            break;
                        }
                        sub_mat_H += fws.jaco_.transpose() * fws.jaco_;

                        size_t position = iter - all_feature_idx.begin();
                        all_feature_idx.erase(all_feature_idx.begin() + position);
                        feature_visited.erase(feature_visited.begin() + position);
                        sel_feature_idx[num_sel_features] = fws.idx_;
                        num_sel_features++;
                        // printf("position: %lu, num: %lu\n", position, num_rnd_que);
                        break;
                    }
                }
                if (num_rnd_que >= MAX_RANDOM_QUEUE_TIME || t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME)
                    break;
            }
        } 
        if (num_rnd_que >= MAX_RANDOM_QUEUE_TIME || t_sel_feature.toc() > MAX_FEATURE_SELECT_TIME)
        {
            // std::cerr << "mapping [goodFeatureMatching]: early termination!" << std::endl;
            LOG_EVERY_N(INFO, 100) << "early termination: feature_type " << feature_type << ", " << num_rnd_que << ", " << t_sel_feature.toc();
        }

        sel_feature_idx.resize(num_sel_features);
        printf("gf_method: %s, num of all features: %lu, sel features: %lu\n", gf_method.c_str(), num_all_features, num_sel_features);
        // printf("logdet of selected sub H: %f\n", common::logDet(sub_mat_H));
    }

    void writeFeature(const PointICovCloud &laser_cloud,
                    const std::vector<size_t> &sel_feature_idx,
                    const std::vector<PointPlaneFeature> &surf_map_features)
    {
        boost::filesystem::create_directory(std::string(OUTPUT_FOLDER + "/gf_pcd").c_str());
        std::string filename1 = OUTPUT_FOLDER + "/gf_pcd/map_feature_" + "origin" + "_" + std::to_string(FLAGS_gf_ratio_ini) + ".pcd";
        std::string filename2 = OUTPUT_FOLDER + "/gf_pcd/map_feature_" + FLAGS_gf_method + "_" + std::to_string(FLAGS_gf_ratio_ini) + ".pcd";

        pcl::PCDWriter pcd_writer;
        pcd_writer.write(filename1.c_str(), laser_cloud);

        common::PointICloud sel_map_feature;
        for (const size_t &idx : sel_feature_idx)
        {
            const PointPlaneFeature &feature = surf_map_features[idx];
            common::PointI point;
            point.x = feature.point_[0];
            point.y = feature.point_[1];
            point.z = feature.point_[2];
            point.intensity = feature.laser_idx_;
            sel_map_feature.points.push_back(point);
        }
        sel_map_feature.height = 1;
        sel_map_feature.width = sel_map_feature.points.size();
        pcd_writer.write(filename2.c_str(), sel_map_feature);
    }

    void pubFeature(const PointICovCloud &laser_cloud,
                    const std::vector<size_t> &sel_feature_idx,
                    const std::vector<PointPlaneFeature> &surf_map_features,
                    const ros::Publisher &pub_laser_cloud,
                    const double &time_laser_odometry)
    {
        common::PointICloud sel_map_feature;
        for (const size_t &idx : sel_feature_idx)
        {
            const PointPlaneFeature &feature = surf_map_features[idx];
            common::PointI point;
            point.x = feature.point_[0];
            point.y = feature.point_[1];
            point.z = feature.point_[2];
            point.intensity = feature.laser_idx_;
            sel_map_feature.points.push_back(point);
        }
        sel_map_feature.height = 1;
        sel_map_feature.width = sel_map_feature.points.size();
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(sel_map_feature, cloud_msg);
        cloud_msg.header.stamp = ros::Time().fromSec(time_laser_odometry);
        cloud_msg.header.frame_id = "/world";
        pub_laser_cloud.publish(cloud_msg);       
    }    

    ceres::LossFunction *loss_function_;
    common::RandomGeneratorInt<size_t> rgi_;

};

//
