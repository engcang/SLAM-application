#include <csignal>
#include <fstream>
#include <mutex>

#include <chrono>
#include <std_msgs/Float32.h>
using namespace std::chrono;

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <boost/filesystem.hpp>

#include <pcl/filters/voxel_grid.h>

#include "ig_lio/lio.h"
#include "ig_lio/logger.hpp"
#include "ig_lio/pointcloud_preprocess.h"
#include "ig_lio/timer.h"

namespace fs = boost::filesystem;

LidarType lidar_type = LidarType::LIVOX;
constexpr double kAccScale = 9.80665;
bool enable_acc_correct = true;
bool enable_undistort = true;
bool enable_ahrs_initalization = false;
Eigen::Matrix4d T_imu_lidar;

// parameters used to synchronize livox time with the external imu
double timediff_lidar_wrt_imu = 0.0;
double lidar_timestamp = 0.0;
double imu_timestamp = 0.0;
bool timediff_correct_flag = false;
std::mutex buff_mutex;

// data deque
std::deque<std::pair<double, pcl::PointCloud<PointType>::Ptr>> cloud_buff;
std::deque<sensor_msgs::Imu> imu_buff;
std::deque<nav_msgs::Odometry> gnss_buff;

// ros visualization
ros::Publisher odom_pub;
ros::Publisher current_scan_pub;
ros::Publisher keyframe_scan_pub;
ros::Publisher path_pub;
ros::Publisher calc_time_pub;
nav_msgs::Path path_array;

Timer timer;
std::shared_ptr<PointCloudPreprocess> cloud_preprocess_ptr;
SensorMeasurement sensor_measurement;
std::shared_ptr<LIO> lio_ptr;
pcl::VoxelGrid<PointType> voxel_filter;
std::fstream odom_stream;

void ImuCallBack(const sensor_msgs::Imu::ConstPtr& msg_ptr) {
  static double last_imu_timestamp = 0.0;
  static sensor_msgs::Imu last_imu = *msg_ptr;
  // parameters for EMA filter
  static double a = 0.8;
  static double b = 1.0 - a;

  sensor_msgs::Imu imu_msg = *msg_ptr;
  imu_timestamp = imu_msg.header.stamp.toSec();

  if (abs(timediff_lidar_wrt_imu) > 0.1 && timediff_correct_flag) {
    imu_msg.header.stamp =
        ros::Time().fromSec(imu_timestamp + timediff_lidar_wrt_imu);
  }

  {
    std::lock_guard<std::mutex> lock(buff_mutex);

    if (imu_timestamp < last_imu_timestamp) {
      LOG(WARNING) << "imu loop back, clear buffer";
      imu_buff.clear();
    }
    last_imu_timestamp = imu_timestamp;

    // EMA filter for accelerometer
    imu_msg.linear_acceleration.x =
        msg_ptr->linear_acceleration.x * a + last_imu.linear_acceleration.x * b;
    imu_msg.linear_acceleration.y =
        msg_ptr->linear_acceleration.y * a + last_imu.linear_acceleration.y * b;
    imu_msg.linear_acceleration.z =
        msg_ptr->linear_acceleration.z * a + last_imu.linear_acceleration.z * b;
    last_imu = *msg_ptr;

    // Some Livox datasets miss the gravitational constant in the accelerometer
    if (enable_acc_correct) {
      imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.x * kAccScale;
      imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.y * kAccScale;
      imu_msg.linear_acceleration.z = imu_msg.linear_acceleration.z * kAccScale;
    }

    imu_buff.push_back(imu_msg);
  }
}

// process Velodyne and Outser
void CloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  static double last_lidar_timestamp = 0.0;
  timer.Evaluate(
      [&]() {
        lidar_timestamp = msg->header.stamp.toSec();

        CloudPtr cloud_ptr(new CloudType());
        cloud_preprocess_ptr->Process(msg, cloud_ptr);

        {
          std::lock_guard<std::mutex> lock(buff_mutex);

          if (lidar_timestamp < last_lidar_timestamp) {
            LOG(WARNING) << "lidar loop back, clear buffer";
            cloud_buff.clear();
          }
          last_lidar_timestamp = lidar_timestamp;

          cloud_buff.push_back(
              std::make_pair(msg->header.stamp.toSec(), cloud_ptr));
        }

        // LOG(INFO) << "lidar buff size: " << cloud_buff.size();
      },
      "Cloud Preprocess (Standard)");
}

// process livox
void LivoxCloudCallBack(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
  static double last_lidar_timestamp = 0.0;
  static CloudPtr temp_cloud_ptr(new CloudType());
  static bool first_scan_flag = true;
  static double first_scan_timestamp = 0.0;

  timer.Evaluate(
      [&]() {
        lidar_timestamp = msg->header.stamp.toSec();

        {
          std::lock_guard<std::mutex> lock(buff_mutex);

          // livox synchronizes with external IMU timestamps
          if (!timediff_correct_flag &&
              abs(lidar_timestamp - imu_timestamp) > 1.0 && !imu_buff.empty()) {
            timediff_correct_flag = true;
            timediff_lidar_wrt_imu = lidar_timestamp + 0.1 - imu_timestamp;
            // clear unnecessary imu data after the synchronization
            imu_buff.clear();
            LOG(INFO) << "timediff_lidar_wrt_imu: " << timediff_lidar_wrt_imu
                      << std::endl;
          }
        }

        // livox has been synchronize with IMU
        if (!timediff_correct_flag &&
            abs(lidar_timestamp - imu_timestamp) < 1.0) {
          timediff_correct_flag = true;
        }
        if (!timediff_correct_flag) {
          LOG(INFO) << "Livox LiDAR has not Sync with other sensor!!!"
                    << std::endl;
          return;
        }

        {
          std::lock_guard<std::mutex> lock(buff_mutex);

          // prevent timestamp disorder
          if (lidar_timestamp < last_lidar_timestamp) {
            LOG(WARNING) << "lidar loop back, clear buffer";
            cloud_buff.clear();
            last_lidar_timestamp = lidar_timestamp;
          }

          if (first_scan_flag) {
            first_scan_timestamp = lidar_timestamp;
            first_scan_flag = false;
          }

          cloud_preprocess_ptr->Process(
              msg, temp_cloud_ptr, first_scan_timestamp);

          first_scan_flag = true;
          last_lidar_timestamp = lidar_timestamp;

          CloudPtr cloud_ptr(new CloudType(*temp_cloud_ptr));
          cloud_buff.push_back(std::make_pair(first_scan_timestamp, cloud_ptr));
          temp_cloud_ptr->clear();
        }
      },
      "Cloud Preprocess (Livox)");
}

bool SyncMeasurements() {
  static bool measurement_pushed = false;
  static bool process_lidar = false;
  static SensorMeasurement local_sensor_measurement;
  static double lidar_mean_scantime = 0.0;
  static size_t lidar_scan_num = 0;

  if (cloud_buff.empty() || imu_buff.empty()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(buff_mutex);

  double lidar_end_time = 0.0;
  if (!measurement_pushed) {
    if (!process_lidar) {
      CloudPtr cloud_sort(new CloudType());
      *cloud_sort = *cloud_buff.front().second;
      std::sort(cloud_sort->points.begin(),
                cloud_sort->points.end(),
                [](const PointType& x, const PointType& y) -> bool {
                  return (x.curvature < y.curvature);
                });
      local_sensor_measurement.cloud_ptr_ = cloud_sort;
      local_sensor_measurement.bag_time_ = cloud_buff.front().first;
      if (!local_sensor_measurement.cloud_ptr_->points.empty()) {
        local_sensor_measurement.lidar_start_time_ =
            cloud_buff.front().first +
            local_sensor_measurement.cloud_ptr_->points.front().curvature /
                (double)(1000);
      } else {
        local_sensor_measurement.lidar_start_time_ = cloud_buff.front().first;
      }

      if (local_sensor_measurement.cloud_ptr_->size() <= 1) {
        LOG(WARNING) << "Too Few Points in Cloud!!!" << std::endl;
        lidar_end_time =
            local_sensor_measurement.lidar_start_time_ + lidar_mean_scantime;
      } else if (local_sensor_measurement.cloud_ptr_->points.back().curvature /
                     (double)(1000) <
                 0.5 * lidar_mean_scantime) {
        lidar_end_time =
            local_sensor_measurement.lidar_start_time_ + lidar_mean_scantime;
      } else {
        lidar_scan_num++;
        lidar_end_time =
            local_sensor_measurement.bag_time_ +
            local_sensor_measurement.cloud_ptr_->points.back().curvature /
                (double)(1000);
        lidar_mean_scantime +=
            ((local_sensor_measurement.cloud_ptr_->points.back().curvature -
              local_sensor_measurement.cloud_ptr_->points.front().curvature) /
                 (double)(1000) -
             lidar_mean_scantime) /
            (double)(lidar_scan_num);
      }

      if (enable_undistort) {
        local_sensor_measurement.lidar_end_time_ = lidar_end_time;
      } else {
        local_sensor_measurement.lidar_end_time_ =
            local_sensor_measurement.bag_time_;
      }

      process_lidar = true;
    }

    bool get_gnss_measurement = false;
    while (!gnss_buff.empty()) {
      if (gnss_buff.front().header.stamp.toSec() >
          sensor_measurement.bag_time_) {
        if (gnss_buff.front().header.stamp.toSec() >
            local_sensor_measurement.bag_time_) {
          LOG(INFO) << "gnss too new" << std::endl;
          break;
        }

        if ((int)(gnss_buff.front().twist.covariance[0]) == 1) {
          sensor_measurement.gnss_status_ = GNSSStatus::RTK_FIXED;
        } else {
          sensor_measurement.gnss_status_ = GNSSStatus::NONE;
        }

        sensor_measurement.measurement_type_ = MeasurementType::GNSS;
        sensor_measurement.bag_time_ = gnss_buff.front().header.stamp.toSec();
        sensor_measurement.lidar_start_time_ =
            gnss_buff.front().header.stamp.toSec();
        sensor_measurement.lidar_end_time_ =
            gnss_buff.front().header.stamp.toSec();

        sensor_measurement.has_gnss_ori_ = false;
        Eigen::Vector3d temp_t =
            Eigen::Vector3d(gnss_buff.front().pose.pose.position.x,
                            gnss_buff.front().pose.pose.position.y,
                            gnss_buff.front().pose.pose.position.z);

        if ((gnss_buff.front().pose.pose.orientation.w *
                 gnss_buff.front().pose.pose.orientation.w +
             gnss_buff.front().pose.pose.orientation.x *
                 gnss_buff.front().pose.pose.orientation.x +
             gnss_buff.front().pose.pose.orientation.y *
                 gnss_buff.front().pose.pose.orientation.y +
             gnss_buff.front().pose.pose.orientation.z *
                 gnss_buff.front().pose.pose.orientation.z) < 1) {
          sensor_measurement.gnss_pose_.block<3, 3>(0, 0) =
              Eigen::Matrix3d::Identity();
          sensor_measurement.gnss_pose_.block<3, 1>(0, 3) = temp_t;
          LOG(INFO) << "get gnss measurement." << std::endl;
        } else {
          Eigen::Quaterniond temp_q(gnss_buff.front().pose.pose.orientation.w,
                                    gnss_buff.front().pose.pose.orientation.x,
                                    gnss_buff.front().pose.pose.orientation.y,
                                    gnss_buff.front().pose.pose.orientation.z);

          sensor_measurement.gnss_pose_.block<3, 3>(0, 0) =
              temp_q.toRotationMatrix();
          sensor_measurement.gnss_pose_.block<3, 1>(0, 3) = temp_t;
          sensor_measurement.has_gnss_ori_ = true;
          LOG(INFO) << "get gnss measurement with ori." << std::endl;
        }

        get_gnss_measurement = true;

        break;
      } else {
        gnss_buff.pop_front();
        LOG(INFO) << "gnss too old" << std::endl;
      }
    }

    if (!get_gnss_measurement) {
      sensor_measurement = local_sensor_measurement;
      sensor_measurement.measurement_type_ = MeasurementType::LIDAR;
    }

    measurement_pushed = true;

    if (sensor_measurement.measurement_type_ == MeasurementType::LIDAR) {
      cloud_buff.pop_front();
      process_lidar = false;
    } else if (sensor_measurement.measurement_type_ == MeasurementType::GNSS) {
      gnss_buff.pop_front();
    }
  }

  if (imu_buff.back().header.stamp.toSec() <
      sensor_measurement.lidar_end_time_) {
    return false;
  }

  sensor_measurement.imu_buff_.clear();
  while (!imu_buff.empty()) {
    double imu_time = imu_buff.front().header.stamp.toSec();
    if (imu_time < sensor_measurement.lidar_end_time_) {
      sensor_measurement.imu_buff_.push_back(imu_buff.front());
      imu_buff.pop_front();
    } else {
      break;
    }
  }
  sensor_measurement.imu_buff_.push_back(imu_buff.front());

  measurement_pushed = false;
  return true;
}

// The main process of iG-LIO
void Process() {
  // Step 1: Time synchronization
  if (!SyncMeasurements()) {
    return;
  }

  // Step 2: Use AHRS or static initialization
  // If the imu message has orientation channel, LIO can be initialized via AHRS
  if (!lio_ptr->IsInit()) {
    if (enable_ahrs_initalization) {
      lio_ptr->AHRSInitialization(sensor_measurement);
    } else {
      lio_ptr->StaticInitialization(sensor_measurement);
    }
    return;
  }

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  // Step 3: Prediction
  for (size_t i = 0; i < sensor_measurement.imu_buff_.size(); ++i) {
    double time;
    if (i == sensor_measurement.imu_buff_.size() - 1) {
      // Ensure that the integration is intergrated to the measurement time.
      time = sensor_measurement.lidar_end_time_;
    } else {
      time = sensor_measurement.imu_buff_.at(i).header.stamp.toSec();
    }
    Eigen::Vector3d acc(
        sensor_measurement.imu_buff_.at(i).linear_acceleration.x,
        sensor_measurement.imu_buff_.at(i).linear_acceleration.y,
        sensor_measurement.imu_buff_.at(i).linear_acceleration.z);
    Eigen::Vector3d gyr(sensor_measurement.imu_buff_.at(i).angular_velocity.x,
                        sensor_measurement.imu_buff_.at(i).angular_velocity.y,
                        sensor_measurement.imu_buff_.at(i).angular_velocity.z);
    lio_ptr->Predict(time, acc, gyr);
  }

  // Too little points for measurement update!
  if (sensor_measurement.cloud_ptr_->size() <= 1) {
    LOG(WARNING) << "no point, skip this scan";
    return;
  }

  // Setp 4: Measurement Update
  timer.Evaluate([&] { lio_ptr->MeasurementUpdate(sensor_measurement); },
                 "measurement update");

  LOG(INFO) << "iter_num: " << lio_ptr->GetFinalIterations() << std::endl
            << "ba: " << lio_ptr->GetCurrentBa().transpose()
            << " ba_norm: " << lio_ptr->GetCurrentBa().norm()
            << " bg: " << lio_ptr->GetCurrentBg().transpose() * 180.0 / M_PI
            << " bg_norm: " << lio_ptr->GetCurrentBg().norm() * 180.0 / M_PI
            << std::endl;

  // Setp 5: Send to rviz for visualization
  Eigen::Matrix4d result_pose = lio_ptr->GetCurrentPose();
  // odometry message
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "world";
  odom_msg.header.stamp = ros::Time(sensor_measurement.lidar_end_time_);
  Eigen::Quaterniond temp_q(result_pose.block<3, 3>(0, 0));
  odom_msg.pose.pose.orientation.x = temp_q.x();
  odom_msg.pose.pose.orientation.y = temp_q.y();
  odom_msg.pose.pose.orientation.z = temp_q.z();
  odom_msg.pose.pose.orientation.w = temp_q.w();
  odom_msg.pose.pose.position.x = result_pose(0, 3);
  odom_msg.pose.pose.position.y = result_pose(1, 3);
  odom_msg.pose.pose.position.z = result_pose(2, 3);
  odom_pub.publish(odom_msg);
  // tf message
  static tf::TransformBroadcaster tf_broadcaster;
  tf::Quaternion q_tf(temp_q.x(), temp_q.y(), temp_q.z(), temp_q.w());
  tf::Vector3 t_tf(result_pose(0, 3), result_pose(1, 3), result_pose(2, 3));
  tf_broadcaster.sendTransform(tf::StampedTransform(
      tf::Transform(q_tf, t_tf), odom_msg.header.stamp, "world", "base_link"));
  // publish dense scan
  CloudPtr trans_cloud(new CloudType());
  pcl::transformPointCloud(
      *sensor_measurement.cloud_ptr_, *trans_cloud, result_pose);
  sensor_msgs::PointCloud2 scan_msg;
  pcl::toROSMsg(*trans_cloud, scan_msg);
  scan_msg.header.frame_id = "world";
  scan_msg.header.stamp = ros::Time(sensor_measurement.lidar_end_time_);
  current_scan_pub.publish(scan_msg);
  // publish keyframe path and scan
  static bool is_first_keyframe = true;
  static Eigen::Matrix4d last_keyframe = result_pose;
  Eigen::Matrix4d delta_p = last_keyframe.inverse() * result_pose;
  if (is_first_keyframe || delta_p.block<3, 1>(0, 3).norm() > 1.0 ||
      Sophus::SO3d(delta_p.block<3, 3>(0, 0)).log().norm() > 0.18) {
    if (is_first_keyframe) {
      is_first_keyframe = false;
    }

    last_keyframe = result_pose;

    // publish downsample scan
    CloudPtr cloud_DS(new CloudType());
    voxel_filter.setInputCloud(sensor_measurement.cloud_ptr_);
    voxel_filter.filter(*cloud_DS);
    CloudPtr trans_cloud_DS(new CloudType());
    pcl::transformPointCloud(*cloud_DS, *trans_cloud_DS, result_pose);
    sensor_msgs::PointCloud2 keyframe_scan_msg;
    pcl::toROSMsg(*trans_cloud_DS, keyframe_scan_msg);
    keyframe_scan_msg.header.frame_id = "world";
    keyframe_scan_msg.header.stamp =
        ros::Time(sensor_measurement.lidar_end_time_);
    keyframe_scan_pub.publish(keyframe_scan_msg);

    // publich path
    path_array.header.stamp = ros::Time(sensor_measurement.lidar_end_time_);
    path_array.header.frame_id = "world";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(sensor_measurement.lidar_end_time_);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = result_pose(0, 3);
    pose_stamped.pose.position.y = result_pose(1, 3);
    pose_stamped.pose.position.z = result_pose(2, 3);
    pose_stamped.pose.orientation.w = temp_q.w();
    pose_stamped.pose.orientation.x = temp_q.x();
    pose_stamped.pose.orientation.y = temp_q.y();
    pose_stamped.pose.orientation.z = temp_q.z();
    path_array.poses.push_back(pose_stamped);
    path_pub.publish(path_array);
  }

  // Setp 6: Save trajectory for evo evaluation
  static size_t delay_count = 0;
  if (delay_count > 50) {
    // normal (for NCLT, AVIA dataset)
    Eigen::Matrix4d lio_pose = result_pose;

    // // newer college
    // // The ground turth is in the camera frame
    // // clang-format off
    // Eigen::Matrix4d T_imu_cam;
    // T_imu_cam <<
    //   0.70992163, 0.02460003, 0.70385092, 0.04249269,
    //   -0.70414167, 0.00493623, 0.71004236, 0.06466842,
    //   0.01399269, -0.99968519,  0.02082624, -0.01845775,
    //   0, 0, 0, 1;
    // // clang-format on
    // Eigen::Matrix4d lio_pose = result_pose * T_imu_cam;

    // botian graden
    // The ground turth is in the velodyne frame:
    // 1. if use livox, please uncomment below extrinsics conversion
    // Eigen::Matrix4d T_bmi088_velodyne;
    // // clang-format off
    // T_bmi088_velodyne << 0.999719900795050, 0.021250293957184,
    // 0.010369165229080,
    //     -0.035251075432380, -0.021115170891513, 0.999693088316336,
    //     -0.012938293597011, 0.029257546097129, -0.010640917363274,
    //     0.012715836666423, 0.999862162753536, 0.148908460351328,
    //     0, 0, 0, 1.0;
    // // clang-format on
    // Eigen::Matrix4d lio_pose = result_pose * T_bmi088_velodyne;
    // // 2.if use velodyne, please uncomment below extrinsics conversion
    // Eigen::Matrix4d lio_pose = result_pose * T_imu_lidar;

    Eigen::Quaterniond lio_q(lio_pose.block<3, 3>(0, 0));
    odom_stream << std::fixed << std::setprecision(6)
                << sensor_measurement.lidar_end_time_ << " "
                << std::setprecision(15) << lio_pose(0, 3) << " "
                << lio_pose(1, 3) << " " << lio_pose(2, 3) << " " << lio_q.x()
                << " " << lio_q.y() << " " << lio_q.z() << " " << lio_q.w()
                << std::endl;
  } else {
    delay_count++;
  }
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>( t2 - t1 ).count() / 1000.0;
  std_msgs::Float32 calc_time;
  calc_time.data = duration;
  calc_time_pub.publish(calc_time);
}

bool FLAG_EXIT = false;
void SigHandle(int sig) {
  FLAG_EXIT = true;
  ROS_WARN("catch sig %d", sig);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ig_lio_node");
  ros::NodeHandle nh;

  std::string package_path = ros::package::getPath("ig_lio");
  Logger logger(argc, argv, package_path);

  // init topic
  // imu
  std::string lidar_topic, imu_topic;
  nh.param<std::string>("imu_topic", imu_topic, "/imu/data");
  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 10000, ImuCallBack);
  // lidar
  nh.param<std::string>("lidar_topic", lidar_topic, "velodyne_points");
  std::string lidar_type_string;
  nh.param<std::string>("lidar_type", lidar_type_string, "velodyne");
  if (lidar_type_string == "velodyne") {
    lidar_type = LidarType::VELODYNE;
  } else if (lidar_type_string == "ouster") {
    lidar_type = LidarType::OUSTER;
  } else if (lidar_type_string == "livox") {
    lidar_type = LidarType::LIVOX;
  } else {
    LOG(ERROR) << "erro lidar type!";
    exit(0);
  }
  ros::Subscriber cloud_sub;
  if (lidar_type == LidarType::LIVOX) {
    cloud_sub = nh.subscribe(lidar_topic, 10000, LivoxCloudCallBack);
  } else {
    cloud_sub = nh.subscribe(lidar_topic, 10000, CloudCallBack);
  }

  // load param
  // 1. pointcloud_preprocess
  double time_scale;
  int point_filter_num;
  nh.param<double>("time_scale", time_scale, 1.0);
  nh.param<int>("point_filter_num", point_filter_num, 1);
  LOG(INFO) << "time_scale: " << time_scale << std::endl
            << "point_filter_num: " << point_filter_num;
  PointCloudPreprocess::Config cloud_preprocess_config;
  cloud_preprocess_config.lidar_type = lidar_type;
  cloud_preprocess_config.point_filter_num = point_filter_num;
  cloud_preprocess_config.time_scale = time_scale;
  cloud_preprocess_ptr =
      std::make_shared<PointCloudPreprocess>(cloud_preprocess_config);

  // 2. init LIO
  double scan_resolution, voxel_map_resolution;
  int max_iterations;
  nh.param<double>("scan_resolution", scan_resolution, 1.0);
  nh.param<double>("voxel_map_resolution", voxel_map_resolution, 1.0);
  nh.param<int>("max_iterations", max_iterations, 30);

  double acc_cov, gyr_cov, bg_cov, ba_cov, init_ori_cov, init_pos_cov,
      init_vel_cov, init_ba_cov, init_bg_cov, gravity;
  nh.param<double>("acc_cov", acc_cov, 1.0);
  nh.param<double>("gyr_cov", gyr_cov, 1.0);
  nh.param<double>("ba_cov", ba_cov, 1.0);
  nh.param<double>("bg_cov", bg_cov, 1.0);
  nh.param<double>("init_ori_cov", init_ori_cov, 1.0);
  nh.param<double>("init_pos_cov", init_pos_cov, 1.0);
  nh.param<double>("init_vel_cov", init_vel_cov, 1.0);
  nh.param<double>("init_ba_cov", init_ba_cov, 1.0);
  nh.param<double>("init_bg_cov", init_bg_cov, 1.0);
  nh.param<double>("gravity", gravity, 1.0);

  double gicp_constraints_gain;
  double point2plane_constraints_gain;
  bool enable_outlier_rejection;
  nh.param<double>("gicp_constraints_gain", gicp_constraints_gain, 1.0);
  nh.param<double>(
      "point2plane_constraints_gain", point2plane_constraints_gain, 1.0);
  nh.param<bool>("enable_undistort", enable_undistort, true);
  nh.param<bool>("enable_outlier_rejection", enable_outlier_rejection, false);
  nh.param<bool>("enable_acc_correct", enable_acc_correct, false);
  nh.param<bool>("enable_ahrs_initalization", enable_ahrs_initalization, false);

  double min_radius, max_radius;
  nh.param<double>("min_radius", min_radius, 1.0);
  nh.param<double>("max_radius", max_radius, 1.0);

  LOG(INFO) << "scan_resoultion: " << scan_resolution << std::endl
            << "voxel_map_resolution: " << voxel_map_resolution << std::endl
            << "max_iterations: " << max_iterations << std::endl
            << "acc_cov: " << acc_cov << std::endl
            << "gyr_cov: " << gyr_cov << std::endl
            << "ba_cov: " << ba_cov << std::endl
            << "bg_cov: " << bg_cov << std::endl
            << "gravity: " << gravity << std::endl
            << "init_ori_cov: " << init_ori_cov << std::endl
            << "init_pos_cov: " << init_pos_cov << std::endl
            << "init_vel_cov: " << init_vel_cov << std::endl
            << "init_ba_cov: " << init_ba_cov << std::endl
            << "init_bg_cov: " << init_bg_cov << std::endl
            << "gicp_constraints_gain: " << gicp_constraints_gain << std::endl
            << "point2plane_constraints_gain: " << point2plane_constraints_gain
            << std::endl
            << "enable_undistort: " << enable_undistort << std::endl
            << "enable_acc_correct: " << enable_acc_correct << std::endl
            << "enable_outlier_rejection: " << enable_outlier_rejection
            << std::endl
            << "enable_ahrs_initalization: " << enable_ahrs_initalization
            << std::endl
            << "min_radius: " << min_radius << std::endl
            << "max_radius: " << max_radius;

  // 3. load extrinsic
  T_imu_lidar = Eigen::Matrix4d::Identity();
  std::vector<double> t_imu_lidar_v;
  std::vector<double> R_imu_lidar_v;
  nh.param<std::vector<double>>(
      "t_imu_lidar", t_imu_lidar_v, std::vector<double>());
  T_imu_lidar.block<3, 1>(0, 3) =
      Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
          t_imu_lidar_v.data(), 3, 1);
  nh.param<std::vector<double>>(
      "R_imu_lidar", R_imu_lidar_v, std::vector<double>());
  T_imu_lidar.block<3, 3>(0, 0) =
      Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
          R_imu_lidar_v.data(), 3, 3);
  LOG(INFO) << "Extrinsic: " << std::endl << T_imu_lidar << std::endl;

  LIO::Config lio_config;
  lio_config.acc_cov = acc_cov;
  lio_config.gyr_cov = gyr_cov;
  lio_config.ba_cov = ba_cov;
  lio_config.bg_cov = bg_cov;

  lio_config.gravity = gravity;
  lio_config.init_ori_cov = init_ori_cov;
  lio_config.init_pos_cov = init_pos_cov;
  lio_config.init_vel_cov = init_vel_cov;
  lio_config.init_ba_cov = init_ba_cov;
  lio_config.init_bg_cov = init_bg_cov;

  lio_config.gicp_constraint_gain = gicp_constraints_gain;
  lio_config.point2plane_constraint_gain = point2plane_constraints_gain;
  lio_config.enable_outlier_rejection = enable_outlier_rejection;
  lio_config.enable_undistort = enable_undistort;
  lio_config.max_iterations = max_iterations;
  lio_config.current_scan_resolution = scan_resolution;
  lio_config.voxel_map_resolution = voxel_map_resolution;
  lio_config.min_radius = min_radius;
  lio_config.max_radius = max_radius;

  lio_config.T_imu_lidar = T_imu_lidar;

  lio_ptr = std::make_shared<LIO>(lio_config);

  // init ros topic
  odom_pub = nh.advertise<nav_msgs::Odometry>("/lio_odom", 10000);
  current_scan_pub =
      nh.advertise<sensor_msgs::PointCloud2>("current_scan", 10000);
  keyframe_scan_pub =
      nh.advertise<sensor_msgs::PointCloud2>("keyframe_scan", 10000);
  path_pub = nh.advertise<nav_msgs::Path>("/path", 10000, true);
  calc_time_pub = nh.advertise<std_msgs::Float32>("/calc_time", 10000);

  voxel_filter.setLeafSize(0.5, 0.5, 0.5);

  // save trajectory
  fs::path result_path = fs::path(package_path) / "result" / "lio_odom.txt";
  if (!fs::exists(result_path.parent_path())) {
    fs::create_directories(result_path.parent_path());	
  }

  odom_stream.open(result_path, std::ios::out);
  if (!odom_stream.is_open()) {
    LOG(ERROR) << "failed to open: " << result_path;
    exit(0);
  }

  // run !!!
  signal(SIGINT, SigHandle);
  ros::Rate rate(5000);
  while (ros::ok()) {
    if (FLAG_EXIT) {
      break;
    }
    ros::spinOnce();
    Process();
    rate.sleep();
  }

  timer.PrintAll();
}
